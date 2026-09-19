// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Achievements.h"
#include "BuildVersion.h"
#include "Common.h"
#include "ControlServer.h"
#include "Counters.h"
#include "GS/GS.h"
#include "Host.h"
#include "MTGS.h"
#include "SIO/Pad/Pad.h"
#include "SIO/Pad/PadBase.h"
#include "SIO/Pad/PadDualshock2.h"
#include "SIO/Sio.h"
#include "VMManager.h"
#include "vtlb.h"

#include "common/Error.h"
#include "common/FileSystem.h"
#include "common/Image.h"
#include "common/Path.h"
#include "common/StringUtil.h"
#include "common/Threading.h"

#include "fmt/format.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cerrno>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

#if defined(_WIN32)
#define read_portable(a, b, c) (recv(a, (char*)b, c, 0))
#define write_portable(a, b, c) (send(a, (const char*)b, c, 0))
#define safe_close_portable(a) \
	do \
	{ \
		if ((a) != INVALID_SOCKET) \
		{ \
			closesocket((a)); \
			(a) = INVALID_SOCKET; \
		} \
	} while (0)
#include "common/RedtapeWindows.h"
#include <WinSock2.h>
#include <ws2tcpip.h>
#else
// MSG_NOSIGNAL on every POSIX path: a client disconnecting mid-reply would otherwise
// raise SIGPIPE and take the whole emulator down with it.
#define read_portable(a, b, c) (read(a, b, c))
#define write_portable(a, b, c) (send(a, b, c, MSG_NOSIGNAL))
#define safe_close_portable(a) \
	do \
	{ \
		if ((a) >= 0) \
		{ \
			close((a)); \
			(a) = -1; \
		} \
	} while (0)
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace ControlServer
{
#ifdef _WIN32
	using socket_t = SOCKET;
	static constexpr socket_t INVALID_SOCKET_VALUE = INVALID_SOCKET;
#else
	using socket_t = int;
	static constexpr socket_t INVALID_SOCKET_VALUE = -1;
#endif

	// A 16MB read covers the whole of EE main RAM in two requests; the 64MB line cap lets
	// a caller base64 one of those back at us without the framing falling over.
	static constexpr size_t MAX_LINE_BYTES = 64 * 1024 * 1024;
	static constexpr u32 MAX_READ_BYTES = 16 * 1024 * 1024;
	static constexpr u32 MAX_READ_MANY_ENTRIES = 1024;
	static constexpr u64 MAX_READ_MANY_TOTAL = 64 * 1024 * 1024;
	static constexpr u32 MAX_STEP_FRAMES = 100000;
	static constexpr u32 MAX_IMAGE_DIMENSION = 8192;

	static constexpr u32 MIN_TIMEOUT_MS = 1;
	static constexpr u32 MAX_TIMEOUT_MS = 120000;
	static constexpr u32 DEFAULT_TIMEOUT_MS = 5000;
	static constexpr u32 DEFAULT_STEP_TIMEOUT_MS = 15000;
	// Added per frame on top of the base when no explicit timeout is given. A debug build
	// runs well under realtime, so a fixed default makes large steps fail for no reason
	// other than being large.
	static constexpr u32 STEP_TIMEOUT_MS_PER_FRAME = 100;

	static constexpr size_t PAD_BIND_COUNT = static_cast<size_t>(PadDualshock2::Inputs::LENGTH);
	static constexpr u32 MAX_INPUT_QUEUE_FRAMES = 4096;
	static constexpr u32 MAX_INPUT_REPEAT = 4096;
	static constexpr u8 PAD_ANALOG_NEUTRAL = 0x7f;
	static constexpr u32 DEFAULT_RESET_TIMEOUT_MS = 30000;
	// Zstd-compressing a ~40MB state and writing it is not instant, and a cold read of one
	// off a spinning disk is slower still.
	static constexpr u32 DEFAULT_SAVE_STATE_TIMEOUT_MS = 60000;

	// A GS dump is armed now and written by the renderer over the next several vsyncs, so
	// the op has to drive frames until it closes. The budget is generous because the number
	// of vsyncs it takes is not fixed -- see OpGsDump().
	static constexpr u32 DEFAULT_GS_DUMP_TIMEOUT_MS = 30000;
	static constexpr u32 GS_DUMP_POLL_FRAMES = 2;
	static constexpr u32 GS_DUMP_MAX_FRAMES = 120;

	// Matches SaveState_SaveScreenshot(). A fixed size keeps captures comparable between
	// runs, which matters more for a harness than matching the window.
	static constexpr u32 SCREENSHOT_WIDTH = 640;
	static constexpr u32 SCREENSHOT_HEIGHT = 480;

	static constexpr size_t RECV_CHUNK_BYTES = 64 * 1024;
	static constexpr size_t SEND_CHUNK_BYTES = 1024 * 1024;

	enum class ErrorCode
	{
		None,
		// Protocol tier: the request itself was not usable.
		BadRequest,
		UnknownOp,
		ProtocolError,
		// Operation tier: the request was fine, the operation was not possible.
		NoVM,
		BadState,
		BadAddress,
		TooLarge,
		Busy,
		Timeout,
		Interrupted,
		Unsupported,
		Internal,
	};

	static const char* ErrorCodeName(ErrorCode code)
	{
		switch (code)
		{
			case ErrorCode::None: return "none";
			case ErrorCode::BadRequest: return "bad_request";
			case ErrorCode::UnknownOp: return "unknown_op";
			case ErrorCode::ProtocolError: return "protocol_error";
			case ErrorCode::NoVM: return "no_vm";
			case ErrorCode::BadState: return "bad_state";
			case ErrorCode::BadAddress: return "bad_address";
			case ErrorCode::TooLarge: return "too_large";
			case ErrorCode::Busy: return "busy";
			case ErrorCode::Timeout: return "timeout";
			case ErrorCode::Interrupted: return "interrupted";
			case ErrorCode::Unsupported: return "unsupported";
			case ErrorCode::Internal: return "internal";
			default: return "internal";
		}
	}

	static const char* VMStateName(VMState state)
	{
		switch (state)
		{
			case VMState::Shutdown: return "shutdown";
			case VMState::Initializing: return "initializing";
			case VMState::Running: return "running";
			case VMState::Paused: return "paused";
			case VMState::Resetting: return "resetting";
			case VMState::Stopping: return "stopping";
			default: return "unknown";
		}
	}

	/// A unit of work handed to the CPU thread, owned jointly by the server thread that
	/// created it and by the lambda that will run it. Both halves hold a shared_ptr, so a
	/// request that times out can walk away without leaving the lambda writing into a
	/// stack frame that no longer exists.
	struct Job
	{
		std::mutex mtx;
		std::condition_variable cv;
		bool done = false;
		ErrorCode error = ErrorCode::None;
		std::string message;

		// Screenshot payload, filled in on the CPU thread before Complete().
		u32 img_width = 0;
		u32 img_height = 0;
		std::vector<u32> pixels;

		// gs_dump: whether the renderer still holds the dump file open. Written on the CPU
		// thread before Complete(), same as the screenshot payload above.
		bool gs_dump_running = false;

		void Complete(ErrorCode code, std::string msg = {})
		{
			{
				std::lock_guard<std::mutex> lock(mtx);
				// First completion wins; a late one after a timeout is dropped on the floor.
				if (done)
					return;
				done = true;
				error = code;
				message = std::move(msg);
			}
			cv.notify_all();
		}

		/// Collects the result under the lock. The payload is only touched on success,
		/// because after a timeout the CPU-thread half may still be writing to it.
		ErrorCode Take(std::string* out_message, u32* out_width = nullptr, u32* out_height = nullptr,
			std::vector<u32>* out_pixels = nullptr)
		{
			std::lock_guard<std::mutex> lock(mtx);
			if (out_message)
				*out_message = std::move(message);
			if (error == ErrorCode::None)
			{
				if (out_width)
					*out_width = img_width;
				if (out_height)
					*out_height = img_height;
				if (out_pixels)
					*out_pixels = std::move(pixels);
			}
			return error;
		}

		/// Only meaningful once Take() has returned None, for the same reason the screenshot
		/// payload is: before that the CPU-thread half may still be writing it.
		bool TakeGsDumpRunning()
		{
			std::lock_guard<std::mutex> lock(mtx);
			return gs_dump_running;
		}
	};

	static std::atomic_bool s_end{true};
	static std::thread s_thread;
	static int s_port = 0;
	static socket_t s_sock = INVALID_SOCKET_VALUE;
	static socket_t s_msgsock = INVALID_SOCKET_VALUE;

	static std::mutex s_pending_mtx;
	static std::vector<std::weak_ptr<Job>> s_pending;

	/// One frame of pad state, absolute rather than a delta: every bind is written every
	/// injected frame, so a button is released simply by not naming it.
	struct PadFrame
	{
		u8 port = 0;
		std::array<float, PAD_BIND_COUNT> values{};
	};

	static std::mutex s_input_mtx;
	static std::deque<PadFrame> s_input_queue;
	// Ports the queue has written to, so they can all be released when it drains.
	static u32 s_input_ports_touched = 0;

	static std::mutex s_step_mtx;
	static std::shared_ptr<Job> s_active_step;
	static bool s_expect_auto_pause = false;

	static std::atomic<u64> s_vsync_count{0};

	static void MainLoop();
	static void ClientLoop();
	static bool AcceptClient();
} // namespace ControlServer

#ifdef _WIN32

// Duplicated from PINE.cpp rather than shared: WSAStartup is refcounted, so two
// independent callers are harmless, and this keeps the module free of PINE internals.
static bool InitializeControlServerWinsock()
{
	static bool initialized = false;
	if (initialized)
		return true;

	WSADATA wsa = {};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		return false;

	initialized = true;
	std::atexit([]() { WSACleanup(); });
	return true;
}

#endif

// ---------------------------------------------------------------------------
// Job plumbing
// ---------------------------------------------------------------------------

namespace ControlServer
{
	static void RegisterPendingJob(const std::shared_ptr<Job>& job)
	{
		std::lock_guard<std::mutex> lock(s_pending_mtx);
		std::erase_if(s_pending, [](const std::weak_ptr<Job>& weak) { return weak.expired(); });
		s_pending.push_back(job);
	}

	static void UnregisterPendingJob(const std::shared_ptr<Job>& job)
	{
		std::lock_guard<std::mutex> lock(s_pending_mtx);
		std::erase_if(s_pending, [&job](const std::weak_ptr<Job>& weak) {
			const std::shared_ptr<Job> held = weak.lock();
			return !held || held == job;
		});
	}

	static void CancelAllPendingJobs(ErrorCode code, const char* message)
	{
		{
			std::lock_guard<std::mutex> lock(s_step_mtx);
			s_active_step.reset();
			s_expect_auto_pause = false;
		}

		{
			// Leave s_input_ports_touched alone: the release still needs to happen.
			std::lock_guard<std::mutex> lock(s_input_mtx);
			s_input_queue.clear();
		}

		std::vector<std::shared_ptr<Job>> jobs;
		{
			std::lock_guard<std::mutex> lock(s_pending_mtx);
			for (const std::weak_ptr<Job>& weak : s_pending)
			{
				if (std::shared_ptr<Job> held = weak.lock())
					jobs.push_back(std::move(held));
			}
			s_pending.clear();
		}

		for (const std::shared_ptr<Job>& job : jobs)
			job->Complete(code, message);
	}

	/// Queues func onto the CPU thread and waits, bounded, for it to complete job.
	///
	/// Deliberately never uses Host::RunOnCPUThread(..., block=true): Deinitialize() joins
	/// this thread from CPUThreadShutdown(), and a blocking queued call parked on an event
	/// loop that has already exited would wedge that join. Every wait here is bounded and
	/// every pending job is cancellable, so shutdown can always make progress.
	static void RunJob(const std::shared_ptr<Job>& job, u32 timeout_ms, std::function<void()> func)
	{
		RegisterPendingJob(job);
		Host::RunOnCPUThread(std::move(func), false);

		{
			std::unique_lock<std::mutex> lock(job->mtx);
			if (!job->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&job]() { return job->done; }))
			{
				job->done = true;
				job->error = ErrorCode::Timeout;
				job->message = "timed out waiting for the CPU thread";
			}
		}

		UnregisterPendingJob(job);
	}
} // namespace ControlServer

// ---------------------------------------------------------------------------
// base64
// ---------------------------------------------------------------------------

namespace ControlServer
{
	static constexpr char BASE64_ALPHABET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	static std::string EncodeBase64(const u8* data, size_t length)
	{
		std::string out;
		out.reserve(((length + 2) / 3) * 4);

		size_t i = 0;
		for (; i + 3 <= length; i += 3)
		{
			const u32 triple = (static_cast<u32>(data[i]) << 16) | (static_cast<u32>(data[i + 1]) << 8) |
							   static_cast<u32>(data[i + 2]);
			out.push_back(BASE64_ALPHABET[(triple >> 18) & 0x3F]);
			out.push_back(BASE64_ALPHABET[(triple >> 12) & 0x3F]);
			out.push_back(BASE64_ALPHABET[(triple >> 6) & 0x3F]);
			out.push_back(BASE64_ALPHABET[triple & 0x3F]);
		}

		if (i < length)
		{
			const size_t remaining = length - i;
			u32 triple = static_cast<u32>(data[i]) << 16;
			if (remaining == 2)
				triple |= static_cast<u32>(data[i + 1]) << 8;

			out.push_back(BASE64_ALPHABET[(triple >> 18) & 0x3F]);
			out.push_back(BASE64_ALPHABET[(triple >> 12) & 0x3F]);
			out.push_back((remaining == 2) ? BASE64_ALPHABET[(triple >> 6) & 0x3F] : '=');
			out.push_back('=');
		}

		return out;
	}

	static std::optional<std::vector<u8>> DecodeBase64(std::string_view str)
	{
		// Reverse table built once; 0xFF marks a character outside the alphabet.
		static const std::array<u8, 256> lookup = []() {
			std::array<u8, 256> table;
			table.fill(0xFF);
			for (u8 i = 0; i < 64; i++)
				table[static_cast<u8>(BASE64_ALPHABET[i])] = i;
			return table;
		}();

		// Padding is optional on input, but the remaining length still has to be a sane
		// quantum: 4n+1 characters cannot encode a whole number of bytes.
		while (!str.empty() && str.back() == '=')
			str.remove_suffix(1);
		if ((str.size() % 4) == 1)
			return std::nullopt;

		std::vector<u8> out;
		out.reserve((str.size() / 4) * 3 + 3);

		u32 accumulator = 0;
		u32 bits = 0;
		for (const char ch : str)
		{
			const u8 value = lookup[static_cast<u8>(ch)];
			if (value == 0xFF)
				return std::nullopt;

			accumulator = (accumulator << 6) | value;
			bits += 6;
			if (bits >= 8)
			{
				bits -= 8;
				out.push_back(static_cast<u8>((accumulator >> bits) & 0xFF));
			}
		}

		return out;
	}
} // namespace ControlServer

// ---------------------------------------------------------------------------
// Request parsing and reply construction
// ---------------------------------------------------------------------------

namespace ControlServer
{
	using JsonWriter = rapidjson::Writer<rapidjson::StringBuffer>;

	/// What a handler produces: either a payload writer, or a failure.
	struct ReplyContext
	{
		ErrorCode code = ErrorCode::None;
		std::string message;
		std::function<void(JsonWriter&)> payload;

		bool Ok() const { return code == ErrorCode::None; }

		void Fail(ErrorCode failure_code, std::string failure_message)
		{
			code = failure_code;
			message = std::move(failure_message);
			payload = {};
		}
	};

	static void WriteString(JsonWriter& writer, std::string_view str)
	{
		writer.String(str.data(), static_cast<rapidjson::SizeType>(str.size()));
	}

	static void WriteHexU32(JsonWriter& writer, u32 value)
	{
		const std::string str = fmt::format("0x{:08x}", value);
		WriteString(writer, str);
	}

	static const rapidjson::Value* FindMember(const rapidjson::Value& object, const char* name)
	{
		const auto it = object.FindMember(name);
		return (it != object.MemberEnd() && !it->value.IsNull()) ? &it->value : nullptr;
	}

	/// Accepts either a JSON integer or a hex string ("0x58BEB0", "58beb0").
	static bool ParseAddress(const rapidjson::Value& value, u32* out)
	{
		if (value.IsUint())
		{
			*out = value.GetUint();
			return true;
		}
		if (value.IsInt64())
		{
			const s64 raw = value.GetInt64();
			if (raw < 0 || raw > 0xFFFFFFFFLL)
				return false;
			*out = static_cast<u32>(raw);
			return true;
		}
		if (value.IsUint64())
		{
			const u64 raw = value.GetUint64();
			if (raw > 0xFFFFFFFFULL)
				return false;
			*out = static_cast<u32>(raw);
			return true;
		}
		if (value.IsString())
		{
			std::string_view str(value.GetString(), value.GetStringLength());
			if (str.size() > 2 && str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
				str = str.substr(2);
			if (str.empty())
				return false;

			const std::optional<u32> parsed = StringUtil::FromChars<u32>(str, 16);
			if (!parsed.has_value())
				return false;
			*out = parsed.value();
			return true;
		}
		return false;
	}

	static bool GetAddressField(const rapidjson::Value& request, const char* name, u32* out, ReplyContext& ctx)
	{
		const rapidjson::Value* value = FindMember(request, name);
		if (!value)
		{
			ctx.Fail(ErrorCode::BadRequest, fmt::format("'{}' is required", name));
			return false;
		}
		if (!ParseAddress(*value, out))
		{
			ctx.Fail(ErrorCode::BadRequest,
				fmt::format("'{}' must be a 32-bit address, as an integer or a hex string", name));
			return false;
		}
		return true;
	}

	static bool GetUintField(const rapidjson::Value& request, const char* name, u32 min_value, u32 max_value,
		u32 default_value, bool required, u32* out, ReplyContext& ctx)
	{
		const rapidjson::Value* value = FindMember(request, name);
		if (!value)
		{
			if (required)
			{
				ctx.Fail(ErrorCode::BadRequest, fmt::format("'{}' is required", name));
				return false;
			}
			*out = default_value;
			return true;
		}

		if (!value->IsInt64() && !value->IsUint64())
		{
			ctx.Fail(ErrorCode::BadRequest, fmt::format("'{}' must be an integer", name));
			return false;
		}

		const s64 raw = value->IsInt64() ? value->GetInt64() : static_cast<s64>(value->GetUint64());
		if (raw < static_cast<s64>(min_value) || raw > static_cast<s64>(max_value))
		{
			ctx.Fail(ErrorCode::BadRequest,
				fmt::format("'{}' must be between {} and {} (got {})", name, min_value, max_value, raw));
			return false;
		}

		*out = static_cast<u32>(raw);
		return true;
	}

	static bool GetBoolField(
		const rapidjson::Value& request, const char* name, bool default_value, bool* out, ReplyContext& ctx)
	{
		const rapidjson::Value* value = FindMember(request, name);
		if (!value)
		{
			*out = default_value;
			return true;
		}
		if (!value->IsBool())
		{
			ctx.Fail(ErrorCode::BadRequest, fmt::format("'{}' must be a boolean", name));
			return false;
		}
		*out = value->GetBool();
		return true;
	}

	static bool GetTimeoutField(const rapidjson::Value& request, u32 default_value, u32* out, ReplyContext& ctx)
	{
		return GetUintField(request, "timeout_ms", MIN_TIMEOUT_MS, MAX_TIMEOUT_MS, default_value, false, out, ctx);
	}

	/// Reads a required filesystem path. Only absolute paths are accepted: a relative one
	/// would resolve against the emulator's working directory, which the client has no way
	/// to know and which differs depending on how PCSX2 was launched.
	static bool GetPathField(const rapidjson::Value& request, std::string* out, ReplyContext& ctx)
	{
		const rapidjson::Value* value = FindMember(request, "path");
		if (!value)
		{
			ctx.Fail(ErrorCode::BadRequest, "'path' is required");
			return false;
		}
		if (!value->IsString())
		{
			ctx.Fail(ErrorCode::BadRequest, "'path' must be a string");
			return false;
		}

		const std::string_view str(value->GetString(), value->GetStringLength());
		if (str.empty())
		{
			ctx.Fail(ErrorCode::BadRequest, "'path' must not be empty");
			return false;
		}
		// rapidjson happily decodes \u0000, and an embedded NUL would silently truncate the
		// path on its way into the C string APIs below.
		if (str.find('\0') != std::string_view::npos)
		{
			ctx.Fail(ErrorCode::BadRequest, "'path' must not contain a null byte");
			return false;
		}
		if (!Path::IsAbsolute(str))
		{
			ctx.Fail(ErrorCode::BadRequest, fmt::format("'path' must be absolute (got '{}')", str));
			return false;
		}

		*out = Path::Canonicalize(str);
		return true;
	}

	static bool GetFormatField(const rapidjson::Value& request, bool* as_hex, ReplyContext& ctx)
	{
		*as_hex = false;

		const rapidjson::Value* value = FindMember(request, "format");
		if (!value)
			return true;
		if (!value->IsString())
		{
			ctx.Fail(ErrorCode::BadRequest, "'format' must be a string");
			return false;
		}

		const std::string_view str(value->GetString(), value->GetStringLength());
		if (str == "b64")
			return true;
		if (str == "hex")
		{
			*as_hex = true;
			return true;
		}

		ctx.Fail(ErrorCode::BadRequest, "'format' must be \"b64\" or \"hex\"");
		return false;
	}

	/// The vtlb vmap is allocated for the lifetime of the CPU thread (SysMemory::Allocate()
	/// in CPUThreadInitialize), not per-VM, so reading it from this thread cannot dangle --
	/// once a VM goes away the entries revert to handlers and the safe accessors report a
	/// clean failure. What it can do is tear while SysMemory::Reset() rewrites it, so limit
	/// memory access to the two states in which no reset is in flight.
	static bool CheckMemoryAccessible(ReplyContext& ctx)
	{
		if (!VMManager::HasValidVM())
		{
			ctx.Fail(ErrorCode::NoVM, "no virtual machine is running");
			return false;
		}

		const VMState state = VMManager::GetState();
		if (state != VMState::Running && state != VMState::Paused)
		{
			ctx.Fail(ErrorCode::BadState,
				fmt::format("memory is not accessible while the VM is '{}'", VMStateName(state)));
			return false;
		}

		return true;
	}
} // namespace ControlServer

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

namespace ControlServer
{
	static void OpPing(const rapidjson::Value&, ReplyContext& ctx)
	{
		ctx.payload = [](JsonWriter& writer) {
			writer.Key("protocol_version");
			writer.Uint(PROTOCOL_VERSION);
			writer.Key("pcsx2_version");
			WriteString(writer, BuildVersion::GitRev);
		};
	}

	static void OpStatus(const rapidjson::Value&, ReplyContext& ctx)
	{
		const bool has_vm = VMManager::HasValidVM();

		std::string title;
		std::string serial;
		u32 disc_crc = 0;
		u32 elf_crc = 0;
		if (has_vm)
		{
			title = VMManager::GetTitle(false);
			serial = VMManager::GetDiscSerial();
			disc_crc = VMManager::GetDiscCRC();
			elf_crc = VMManager::GetCurrentCRC();
		}
		const bool hardcore = Achievements::IsHardcoreModeActive();

		size_t input_queued = 0;
		{
			std::lock_guard<std::mutex> lock(s_input_mtx);
			input_queued = s_input_queue.size();
		}

		ctx.payload = [has_vm, title = std::move(title), serial = std::move(serial), disc_crc, elf_crc, hardcore,
						  input_queued](JsonWriter& writer) {
			writer.Key("has_vm");
			writer.Bool(has_vm);
			writer.Key("title");
			WriteString(writer, title);
			writer.Key("serial");
			WriteString(writer, serial);
			writer.Key("disc_crc");
			WriteHexU32(writer, disc_crc);
			writer.Key("elf_crc");
			WriteHexU32(writer, elf_crc);
			writer.Key("hardcore");
			writer.Bool(hardcore);
			writer.Key("input_queued");
			writer.Uint64(input_queued);
		};
	}

	static void OpReadBytes(const rapidjson::Value& request, ReplyContext& ctx)
	{
		u32 address = 0;
		if (!GetAddressField(request, "addr", &address, ctx))
			return;

		u32 length = 0;
		if (!GetUintField(request, "len", 1, MAX_READ_BYTES, 0, true, &length, ctx))
			return;

		bool as_hex = false;
		if (!GetFormatField(request, &as_hex, ctx))
			return;

		if (!CheckMemoryAccessible(ctx))
			return;

		std::vector<u8> buffer(length);
		if (!vtlb_memSafeReadBytes(address, buffer.data(), length))
		{
			// It copies page by page and can fail after partially filling the destination,
			// so the contents are not trustworthy -- drop them rather than return a torn read.
			ctx.Fail(ErrorCode::BadAddress,
				fmt::format("unreadable memory in [0x{:08x}, 0x{:08x})", address, address + length));
			return;
		}

		std::string encoded = as_hex ? StringUtil::EncodeHex(buffer.data(), static_cast<int>(length)) :
									   EncodeBase64(buffer.data(), length);

		ctx.payload = [address, length, as_hex, encoded = std::move(encoded)](JsonWriter& writer) {
			writer.Key("addr");
			WriteHexU32(writer, address);
			writer.Key("len");
			writer.Uint(length);
			writer.Key(as_hex ? "data_hex" : "data_b64");
			WriteString(writer, encoded);
		};
	}

	struct ReadManyEntry
	{
		bool ok = false;
		ErrorCode code = ErrorCode::None;
		std::string message;
		u32 address = 0;
		u32 length = 0;
		std::string encoded;
	};

	static void OpReadMany(const rapidjson::Value& request, ReplyContext& ctx)
	{
		const rapidjson::Value* reads = FindMember(request, "reads");
		if (!reads || !reads->IsArray())
		{
			ctx.Fail(ErrorCode::BadRequest, "'reads' is required and must be an array");
			return;
		}
		if (reads->Size() > MAX_READ_MANY_ENTRIES)
		{
			ctx.Fail(ErrorCode::TooLarge,
				fmt::format("'reads' has {} entries, the maximum is {}", reads->Size(), MAX_READ_MANY_ENTRIES));
			return;
		}

		bool as_hex = false;
		if (!GetFormatField(request, &as_hex, ctx))
			return;

		// Validate every entry before reading any of them, so a malformed request fails as
		// a request rather than as a half-completed batch.
		std::vector<ReadManyEntry> entries;
		entries.reserve(reads->Size());
		u64 total = 0;
		for (rapidjson::SizeType i = 0; i < reads->Size(); i++)
		{
			const rapidjson::Value& element = (*reads)[i];
			if (!element.IsObject())
			{
				ctx.Fail(ErrorCode::BadRequest, fmt::format("reads[{}] must be an object", i));
				return;
			}

			ReadManyEntry entry;
			ReplyContext element_ctx;
			if (!GetAddressField(element, "addr", &entry.address, element_ctx) ||
				!GetUintField(element, "len", 1, MAX_READ_BYTES, 0, true, &entry.length, element_ctx))
			{
				ctx.Fail(element_ctx.code, fmt::format("reads[{}]: {}", i, element_ctx.message));
				return;
			}

			total += entry.length;
			if (total > MAX_READ_MANY_TOTAL)
			{
				ctx.Fail(ErrorCode::TooLarge,
					fmt::format("'reads' totals more than the {} byte maximum", MAX_READ_MANY_TOTAL));
				return;
			}

			entries.push_back(std::move(entry));
		}

		if (!CheckMemoryAccessible(ctx))
			return;

		// Back-to-back on this thread, so the reads land within microseconds of each other
		// -- but this is not an atomic snapshot. Pause first if coherence matters.
		std::vector<u8> buffer;
		for (ReadManyEntry& entry : entries)
		{
			buffer.resize(entry.length);
			if (!vtlb_memSafeReadBytes(entry.address, buffer.data(), entry.length))
			{
				entry.ok = false;
				entry.code = ErrorCode::BadAddress;
				entry.message = fmt::format(
					"unreadable memory in [0x{:08x}, 0x{:08x})", entry.address, entry.address + entry.length);
				continue;
			}

			entry.ok = true;
			entry.encoded = as_hex ? StringUtil::EncodeHex(buffer.data(), static_cast<int>(entry.length)) :
									 EncodeBase64(buffer.data(), entry.length);
		}

		ctx.payload = [as_hex, entries = std::move(entries)](JsonWriter& writer) {
			writer.Key("results");
			writer.StartArray();
			for (const ReadManyEntry& entry : entries)
			{
				writer.StartObject();
				writer.Key("ok");
				writer.Bool(entry.ok);
				writer.Key("addr");
				WriteHexU32(writer, entry.address);
				writer.Key("len");
				writer.Uint(entry.length);
				if (entry.ok)
				{
					writer.Key(as_hex ? "data_hex" : "data_b64");
					WriteString(writer, entry.encoded);
				}
				else
				{
					writer.Key("error");
					writer.StartObject();
					writer.Key("code");
					WriteString(writer, ErrorCodeName(entry.code));
					writer.Key("message");
					WriteString(writer, entry.message);
					writer.EndObject();
				}
				writer.EndObject();
			}
			writer.EndArray();
		};
	}

	static void OpWriteBytes(const rapidjson::Value& request, ReplyContext& ctx)
	{
		u32 address = 0;
		if (!GetAddressField(request, "addr", &address, ctx))
			return;

		const rapidjson::Value* b64 = FindMember(request, "data_b64");
		const rapidjson::Value* hex = FindMember(request, "data_hex");
		if ((b64 != nullptr) == (hex != nullptr))
		{
			ctx.Fail(ErrorCode::BadRequest, "exactly one of 'data_b64' or 'data_hex' is required");
			return;
		}

		const rapidjson::Value* source = b64 ? b64 : hex;
		if (!source->IsString())
		{
			ctx.Fail(ErrorCode::BadRequest, "the payload field must be a string");
			return;
		}

		const std::string_view encoded(source->GetString(), source->GetStringLength());
		const std::optional<std::vector<u8>> decoded =
			b64 ? DecodeBase64(encoded) : StringUtil::DecodeHex(encoded);
		if (!decoded.has_value())
		{
			ctx.Fail(ErrorCode::BadRequest, b64 ? "'data_b64' is not valid base64" : "'data_hex' is not valid hex");
			return;
		}
		if (decoded->empty())
		{
			ctx.Fail(ErrorCode::BadRequest, "the payload is empty");
			return;
		}
		if (decoded->size() > MAX_READ_BYTES)
		{
			ctx.Fail(ErrorCode::TooLarge, fmt::format("the payload exceeds the {} byte maximum", MAX_READ_BYTES));
			return;
		}

		if (!CheckMemoryAccessible(ctx))
			return;

		const u32 length = static_cast<u32>(decoded->size());
		if (!vtlb_memSafeWriteBytes(address, decoded->data(), length))
		{
			// Unlike the read path there is nothing to discard: part of the range may
			// already have landed, so the caller has to treat this as indeterminate.
			ctx.Fail(ErrorCode::BadAddress,
				fmt::format("unwritable memory in [0x{:08x}, 0x{:08x}); the write may have partially landed",
					address, address + length));
			return;
		}

		ctx.payload = [address, length](JsonWriter& writer) {
			writer.Key("addr");
			WriteHexU32(writer, address);
			writer.Key("len");
			writer.Uint(length);
		};
	}

	static void OpSetPaused(const rapidjson::Value& request, ReplyContext& ctx, bool paused)
	{
		bool wait = true;
		if (!GetBoolField(request, "wait", true, &wait, ctx))
			return;

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, DEFAULT_TIMEOUT_MS, &timeout_ms, ctx))
			return;

		if (!VMManager::HasValidVM())
		{
			ctx.Fail(ErrorCode::NoVM, "no virtual machine is running");
			return;
		}

		if (!wait)
		{
			Host::RunOnCPUThread(
				[paused]() {
					if (VMManager::HasValidVM())
						VMManager::SetPaused(paused);
				},
				false);
			return;
		}

		const std::shared_ptr<Job> job = std::make_shared<Job>();
		RunJob(job, timeout_ms, [job, paused]() {
			if (!VMManager::HasValidVM())
			{
				job->Complete(ErrorCode::NoVM, "no virtual machine is running");
				return;
			}
			VMManager::SetPaused(paused);
			job->Complete(ErrorCode::None);
		});

		std::string message;
		const ErrorCode code = job->Take(&message);
		if (code != ErrorCode::None)
			ctx.Fail(code, std::move(message));
	}

	static void StartFrameAdvanceOnCPUThread(const std::shared_ptr<Job>& job, u32 frames)
	{
		if (!VMManager::HasValidVM())
		{
			job->Complete(ErrorCode::NoVM, "no virtual machine is running");
			return;
		}

		// FrameAdvance() silently no-ops in hardcore mode and shows an OSD toast, which
		// would otherwise surface here as an unexplained timeout.
		if (Achievements::IsHardcoreModeActive())
		{
			job->Complete(ErrorCode::Unsupported,
				"frame advance is blocked while achievements hardcore mode is active");
			return;
		}

		const VMState state = VMManager::GetState();
		if (state != VMState::Running && state != VMState::Paused)
		{
			job->Complete(ErrorCode::BadState, fmt::format("cannot step while the VM is '{}'", VMStateName(state)));
			return;
		}

		VMManager::FrameAdvance(frames);
		// Completion arrives later, via Internal::OnFrameAdvanceCompleted().
	}

	/// Runs a frame advance and waits for it. Shared by OpStep() and OpGsDump(), which has
	/// to advance frames itself when the VM is paused -- a paused machine emits no vsyncs,
	/// and without vsyncs the renderer never writes the dump it was handed.
	static ErrorCode RunStepFrames(u32 frames, u32 timeout_ms, std::string* message)
	{
		const std::shared_ptr<Job> job = std::make_shared<Job>();
		{
			std::lock_guard<std::mutex> lock(s_step_mtx);
			if (s_active_step)
			{
				*message = "a step is already in flight";
				return ErrorCode::Busy;
			}
			s_active_step = job;
		}

		RunJob(job, timeout_ms, [job, frames]() { StartFrameAdvanceOnCPUThread(job, frames); });

		// However it ended -- completion, interruption or timeout -- this step is over.
		{
			std::lock_guard<std::mutex> lock(s_step_mtx);
			if (s_active_step == job)
				s_active_step.reset();
		}

		return job->Take(message);
	}

	static void OpStep(const rapidjson::Value& request, ReplyContext& ctx)
	{
		u32 frames = 0;
		if (!GetUintField(request, "frames", 1, MAX_STEP_FRAMES, 1, false, &frames, ctx))
			return;

		// Clamped to MAX_TIMEOUT_MS by GetTimeoutField, so a very long step still needs an
		// explicit timeout -- or, better, to be issued as several shorter ones.
		const u64 scaled = u64(DEFAULT_STEP_TIMEOUT_MS) + u64(frames) * STEP_TIMEOUT_MS_PER_FRAME;
		const u32 default_timeout_ms = static_cast<u32>(std::min<u64>(scaled, MAX_TIMEOUT_MS));

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, default_timeout_ms, &timeout_ms, ctx))
			return;

		std::string message;
		const ErrorCode code = RunStepFrames(frames, timeout_ms, &message);
		if (code != ErrorCode::None)
		{
			ctx.Fail(code, std::move(message));
			return;
		}

		ctx.payload = [frames](JsonWriter& writer) {
			writer.Key("frames");
			writer.Uint(frames);
		};
	}

	static void OpReset(const rapidjson::Value& request, ReplyContext& ctx)
	{
		bool wait = true;
		if (!GetBoolField(request, "wait", true, &wait, ctx))
			return;

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, DEFAULT_RESET_TIMEOUT_MS, &timeout_ms, ctx))
			return;

		if (!VMManager::HasValidVM())
		{
			ctx.Fail(ErrorCode::NoVM, "no virtual machine is running");
			return;
		}

		if (!wait)
		{
			Host::RunOnCPUThread(
				[]() {
					if (VMManager::HasValidVM())
						VMManager::RequestReset();
				},
				false);
			return;
		}

		const std::shared_ptr<Job> job = std::make_shared<Job>();
		RunJob(job, timeout_ms, [job]() {
			if (!VMManager::HasValidVM())
			{
				job->Complete(ErrorCode::NoVM, "no virtual machine is running");
				return;
			}
			// Refused when a memory card is mid-write, which would risk data loss.
			if (!VMManager::RequestReset())
			{
				job->Complete(ErrorCode::BadState, "the reset request was refused");
				return;
			}
			job->Complete(ErrorCode::None);
		});

		std::string message;
		const ErrorCode code = job->Take(&message);
		if (code != ErrorCode::None)
			ctx.Fail(code, std::move(message));
	}

	static void OpScreenshot(const rapidjson::Value& request, ReplyContext& ctx)
	{
		u32 width = 0;
		u32 height = 0;
		u32 quality = 0;
		if (!GetUintField(request, "width", 0, MAX_IMAGE_DIMENSION, SCREENSHOT_WIDTH, false, &width, ctx) ||
			!GetUintField(request, "height", 0, MAX_IMAGE_DIMENSION, SCREENSHOT_HEIGHT, false, &height, ctx) ||
			!GetUintField(request, "quality", 1, 100, RGBA8Image::DEFAULT_SAVE_QUALITY, false, &quality, ctx))
		{
			return;
		}

		bool apply_aspect = true;
		bool crop_borders = false;
		if (!GetBoolField(request, "aspect", true, &apply_aspect, ctx) ||
			!GetBoolField(request, "crop", false, &crop_borders, ctx))
		{
			return;
		}

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, DEFAULT_TIMEOUT_MS, &timeout_ms, ctx))
			return;

		std::string path;
		if (const rapidjson::Value* path_value = FindMember(request, "path"))
		{
			if (!path_value->IsString())
			{
				ctx.Fail(ErrorCode::BadRequest, "'path' must be a string");
				return;
			}
			path.assign(path_value->GetString(), path_value->GetStringLength());
			if (path.empty())
			{
				ctx.Fail(ErrorCode::BadRequest, "'path' must not be empty");
				return;
			}
		}

		const std::shared_ptr<Job> job = std::make_shared<Job>();
		RunJob(job, timeout_ms, [job, width, height, apply_aspect, crop_borders]() {
			if (!VMManager::HasValidVM() || !MTGS::IsOpen())
			{
				job->Complete(ErrorCode::NoVM, "no virtual machine is running");
				return;
			}

			u32 out_width = 0;
			u32 out_height = 0;
			std::vector<u32> pixels;
			if (!MTGS::SaveMemorySnapshot(width, height, apply_aspect, crop_borders, &out_width, &out_height, &pixels))
			{
				job->Complete(ErrorCode::Internal, "the GS snapshot failed; the device may have been lost");
				return;
			}

			job->img_width = out_width;
			job->img_height = out_height;
			job->pixels = std::move(pixels);
			job->Complete(ErrorCode::None);
		});

		u32 out_width = 0;
		u32 out_height = 0;
		std::vector<u32> pixels;
		std::string message;
		const ErrorCode code = job->Take(&message, &out_width, &out_height, &pixels);
		if (code != ErrorCode::None)
		{
			ctx.Fail(code, std::move(message));
			return;
		}

		// Encode on this thread rather than in the job: libpng on a 640x480 frame is tens
		// of milliseconds, and there is no reason to stall emulation for it.
		const RGBA8Image image(out_width, out_height, std::move(pixels));

		if (!path.empty())
		{
			if (!image.SaveToFile(path.c_str(), static_cast<u8>(quality)))
			{
				ctx.Fail(ErrorCode::Internal, fmt::format("failed to write the screenshot to '{}'", path));
				return;
			}

			ctx.payload = [out_width, out_height, path = std::move(path)](JsonWriter& writer) {
				writer.Key("width");
				writer.Uint(out_width);
				writer.Key("height");
				writer.Uint(out_height);
				writer.Key("path");
				WriteString(writer, path);
			};
			return;
		}

		// The filename is only how the encoder is selected; nothing is written to disk.
		const std::optional<std::vector<u8>> encoded =
			image.SaveToBuffer("snapshot.png", static_cast<u8>(quality));
		if (!encoded.has_value())
		{
			ctx.Fail(ErrorCode::Internal, "failed to encode the screenshot as PNG");
			return;
		}

		std::string data = EncodeBase64(encoded->data(), encoded->size());
		ctx.payload = [out_width, out_height, data = std::move(data)](JsonWriter& writer) {
			writer.Key("width");
			writer.Uint(out_width);
			writer.Key("height");
			writer.Uint(out_height);
			writer.Key("format");
			WriteString(writer, "png");
			writer.Key("data_b64");
			WriteString(writer, data);
		};
	}

	/// Shared guard for both save-state ops. Both are refused for the same three reasons,
	/// and checking them here rather than letting VMManager report them as strings is what
	/// lets the client see a distinguishable error code.
	static bool CheckSaveStateAllowed(ReplyContext& ctx)
	{
		if (!VMManager::HasValidVM())
		{
			ctx.Fail(ErrorCode::NoVM, "no virtual machine is running");
			return false;
		}
		if (Achievements::IsHardcoreModeActive())
		{
			ctx.Fail(ErrorCode::Unsupported, "save states are blocked while achievements hardcore mode is active");
			return false;
		}
		// Racy by nature -- the card can go busy a moment later, and VMManager checks again
		// on the CPU thread. This only turns the common case into a clean, typed refusal.
		if (MemcardBusy::IsBusy())
		{
			ctx.Fail(ErrorCode::BadState, "the memory card is busy; the request was refused to prevent data loss");
			return false;
		}
		return true;
	}

	static void OpSaveState(const rapidjson::Value& request, ReplyContext& ctx)
	{
		std::string path;
		if (!GetPathField(request, &path, ctx))
			return;

		bool backup = false;
		if (!GetBoolField(request, "backup", false, &backup, ctx))
			return;

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, DEFAULT_SAVE_STATE_TIMEOUT_MS, &timeout_ms, ctx))
			return;

		if (!CheckSaveStateAllowed(ctx))
			return;

		const std::shared_ptr<Job> job = std::make_shared<Job>();
		RunJob(job, timeout_ms, [job, path, backup]() {
			if (!VMManager::HasValidVM())
			{
				job->Complete(ErrorCode::NoVM, "no virtual machine is running");
				return;
			}

			// zip_on_thread = false is what makes this synchronous. With it true the
			// compress-and-write moves to a detached thread and error_callback fires there,
			// long after SaveState() has returned -- there would be nothing left to report.
			// In this mode every failure path invokes the callback before SaveState()
			// returns, so capturing by reference is safe and the result is always known.
			std::string failure;
			VMManager::SaveState(
				path.c_str(), false, backup, [&failure](const std::string& error) { failure = error; });

			if (!failure.empty())
				job->Complete(ErrorCode::Internal, std::move(failure));
			else
				job->Complete(ErrorCode::None);
		});

		std::string message;
		const ErrorCode code = job->Take(&message);
		if (code != ErrorCode::None)
		{
			ctx.Fail(code, std::move(message));
			return;
		}

		ctx.payload = [path = std::move(path)](JsonWriter& writer) {
			writer.Key("path");
			WriteString(writer, path);
		};
	}

	static void OpLoadState(const rapidjson::Value& request, ReplyContext& ctx)
	{
		std::string path;
		if (!GetPathField(request, &path, ctx))
			return;

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, DEFAULT_SAVE_STATE_TIMEOUT_MS, &timeout_ms, ctx))
			return;

		if (!CheckSaveStateAllowed(ctx))
			return;

		// Worth the extra stat: VMManager::LoadState() resets the machine when the file
		// cannot be read, so without this a mistyped path would silently destroy the
		// session instead of coming back as an error.
		if (!FileSystem::FileExists(path.c_str()))
		{
			ctx.Fail(ErrorCode::BadRequest, fmt::format("no save state at '{}'", path));
			return;
		}

		const std::shared_ptr<Job> job = std::make_shared<Job>();
		RunJob(job, timeout_ms, [job, path]() {
			if (!VMManager::HasValidVM())
			{
				job->Complete(ErrorCode::NoVM, "no virtual machine is running");
				return;
			}

			Error error;
			if (!VMManager::LoadState(path.c_str(), &error))
			{
				// Hardcore mode and a busy memory card were already ruled out above, so a
				// failure this late means the state itself was unreadable -- and LoadState()
				// has reset the VM. Say so; the caller's whole scenario is gone.
				job->Complete(ErrorCode::Internal,
					fmt::format("failed to load '{}' ({}); the VM has been reset", path, error.GetDescription()));
				return;
			}
			job->Complete(ErrorCode::None);
		});

		std::string message;
		const ErrorCode code = job->Take(&message);
		if (code != ErrorCode::None)
		{
			ctx.Fail(code, std::move(message));
			return;
		}

		ctx.payload = [path = std::move(path)](JsonWriter& writer) {
			writer.Key("path");
			WriteString(writer, path);
		};
	}

	// GSQueueSnapshot() only honours a caller-supplied path when it ends in ".png", which it
	// then strips before appending its own extensions. Normalise whatever the caller passed,
	// with or without an extension, down to that base.
	static std::string StripGsDumpExtension(const std::string& path)
	{
		static constexpr std::string_view suffixes[] = {".gs.zst", ".gs.xz", ".gs", ".png"};
		for (const std::string_view suffix : suffixes)
		{
			if (path.size() > suffix.size() && StringUtil::EndsWithNoCase(path, suffix))
				return path.substr(0, path.size() - suffix.size());
		}
		return path;
	}

	/// The extension the renderer will actually append. It depends on a setting the caller
	/// cannot see, and probing the filesystem afterwards would pick the wrong file when a
	/// stale dump written under a different compression mode is sitting next to it.
	static const char* GsDumpExtension()
	{
		switch (GSConfig.GSDumpCompression)
		{
			case GSDumpCompressionMethod::Uncompressed:
				return ".gs";
			case GSDumpCompressionMethod::LZMA:
				return ".gs.xz";
			case GSDumpCompressionMethod::Zstandard:
			default:
				return ".gs.zst";
		}
	}

	/// Must be called on the CPU thread. m_dump belongs to the renderer, so the read bounces
	/// through the GS thread and waits rather than racing it.
	static bool IsGsDumpRunningOnCPUThread()
	{
		bool running = false;
		MTGS::RunOnGSThread([&running]() { running = GSIsDumpRunning(); });
		MTGS::WaitGS(false, false, false);
		return running;
	}

	static void OpGsDump(const rapidjson::Value& request, ReplyContext& ctx)
	{
		std::string path;
		if (!GetPathField(request, &path, ctx))
			return;

		u32 timeout_ms = 0;
		if (!GetTimeoutField(request, DEFAULT_GS_DUMP_TIMEOUT_MS, &timeout_ms, ctx))
			return;

		const std::string base = StripGsDumpExtension(path);
		if (Path::GetFileName(base).empty())
		{
			ctx.Fail(ErrorCode::BadRequest, fmt::format("'path' has no file name ('{}')", path));
			return;
		}

		// The renderer opens the file with fopen("wb"), which will not create the directory.
		if (const std::string_view directory = Path::GetDirectory(base); !directory.empty())
		{
			const std::string dir(directory);
			if (!FileSystem::DirectoryExists(dir.c_str()) && !FileSystem::CreateDirectoryPath(dir.c_str(), true))
			{
				ctx.Fail(ErrorCode::Internal, fmt::format("could not create directory '{}'", dir));
				return;
			}
		}

		const std::chrono::steady_clock::time_point deadline =
			std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

		// Phase 1: arm the capture. The renderer picks it up at its next vsync.
		{
			const std::shared_ptr<Job> job = std::make_shared<Job>();
			RunJob(job, timeout_ms, [job, png = base + ".png"]() {
				if (!VMManager::HasValidVM() || !MTGS::IsOpen())
				{
					job->Complete(ErrorCode::NoVM, "no virtual machine is running");
					return;
				}

				const VMState state = VMManager::GetState();
				if (state != VMState::Running && state != VMState::Paused)
				{
					job->Complete(ErrorCode::BadState,
						fmt::format("cannot dump while the VM is '{}'", VMStateName(state)));
					return;
				}

				// QueueSnapshot() silently drops the request when one is already pending,
				// which would otherwise look like a dump that succeeded but never appeared.
				if (IsGsDumpRunningOnCPUThread())
				{
					job->Complete(ErrorCode::Busy, "a GS dump is already in progress");
					return;
				}

				MTGS::RunOnGSThread([png]() { GSQueueSnapshot(png, 1); });
				MTGS::WaitGS(false, false, false);
				job->Complete(ErrorCode::None);
			});

			std::string message;
			if (const ErrorCode code = job->Take(&message); code != ErrorCode::None)
			{
				ctx.Fail(code, std::move(message));
				return;
			}
		}

		// Phase 2: drive frames until the renderer lets go of the file. A "single frame"
		// dump is not finished at the next vsync: GSDumpBase closes only after an even
		// number of fields have gone by with its last-frame flag set, and it starts with two
		// extra frames in hand -- about five vsyncs in practice, but that depends on
		// interlacing and on what the game is doing, so poll instead of guessing a count.
		const bool paused = (VMManager::GetState() == VMState::Paused);
		u32 frames_advanced = 0;
		for (;;)
		{
			const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
			if (now >= deadline)
			{
				ctx.Fail(ErrorCode::Timeout, fmt::format("the GS dump did not finish within {}ms", timeout_ms));
				return;
			}
			const u32 remaining_ms = static_cast<u32>(std::max<s64>(
				1, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()));

			if (paused)
			{
				if (frames_advanced >= GS_DUMP_MAX_FRAMES)
				{
					ctx.Fail(ErrorCode::Internal,
						fmt::format("the GS dump was still open after {} frames", frames_advanced));
					return;
				}

				std::string message;
				const ErrorCode code =
					RunStepFrames(GS_DUMP_POLL_FRAMES, std::min(remaining_ms, DEFAULT_STEP_TIMEOUT_MS), &message);
				if (code != ErrorCode::None)
				{
					ctx.Fail(code, std::move(message));
					return;
				}
				frames_advanced += GS_DUMP_POLL_FRAMES;
			}
			else
			{
				// Frames are already flowing on their own; just let a couple go by.
				std::this_thread::sleep_for(std::chrono::milliseconds(8));
			}

			const std::shared_ptr<Job> job = std::make_shared<Job>();
			RunJob(job, std::min(remaining_ms, DEFAULT_TIMEOUT_MS), [job]() {
				if (!VMManager::HasValidVM() || !MTGS::IsOpen())
				{
					job->Complete(ErrorCode::NoVM, "the VM went away while the dump was in flight");
					return;
				}
				job->gs_dump_running = IsGsDumpRunningOnCPUThread();
				job->Complete(ErrorCode::None);
			});

			std::string message;
			if (const ErrorCode code = job->Take(&message); code != ErrorCode::None)
			{
				ctx.Fail(code, std::move(message));
				return;
			}
			if (!job->TakeGsDumpRunning())
				break;
		}

		std::string dump_path = base + GsDumpExtension();
		if (!FileSystem::FileExists(dump_path.c_str()))
		{
			ctx.Fail(ErrorCode::Internal,
				fmt::format("the renderer released the dump but '{}' is not there", dump_path));
			return;
		}

		const s64 size = FileSystem::GetPathFileSize(dump_path.c_str());

		ctx.payload = [dump_path = std::move(dump_path), size, frames_advanced](JsonWriter& writer) {
			writer.Key("path");
			WriteString(writer, dump_path);
			writer.Key("size");
			writer.Int64(size);
			writer.Key("frames_advanced");
			writer.Uint(frames_advanced);
		};
	}

	struct PadButtonName
	{
		const char* name;
		PadDualshock2::Inputs bind;
	};

	static constexpr PadButtonName PAD_BUTTON_NAMES[] = {
		{"up", PadDualshock2::Inputs::PAD_UP},
		{"down", PadDualshock2::Inputs::PAD_DOWN},
		{"left", PadDualshock2::Inputs::PAD_LEFT},
		{"right", PadDualshock2::Inputs::PAD_RIGHT},
		{"triangle", PadDualshock2::Inputs::PAD_TRIANGLE},
		{"circle", PadDualshock2::Inputs::PAD_CIRCLE},
		{"cross", PadDualshock2::Inputs::PAD_CROSS},
		{"square", PadDualshock2::Inputs::PAD_SQUARE},
		{"select", PadDualshock2::Inputs::PAD_SELECT},
		{"start", PadDualshock2::Inputs::PAD_START},
		{"l1", PadDualshock2::Inputs::PAD_L1},
		{"l2", PadDualshock2::Inputs::PAD_L2},
		{"r1", PadDualshock2::Inputs::PAD_R1},
		{"r2", PadDualshock2::Inputs::PAD_R2},
		{"l3", PadDualshock2::Inputs::PAD_L3},
		{"r3", PadDualshock2::Inputs::PAD_R3},
		{"analog", PadDualshock2::Inputs::PAD_ANALOG},
	};

	static std::string PadButtonNameList()
	{
		std::string list;
		for (const PadButtonName& entry : PAD_BUTTON_NAMES)
		{
			if (!list.empty())
				list += ", ";
			list += entry.name;
		}
		return list;
	}

	/// Splits one signed stick axis into the two half-axis binds the pad actually exposes.
	/// 0x7f is neutral, so the two sides have different ranges -- 0x7f..0xff is one short.
	static void SetAnalogAxis(std::array<float, PAD_BIND_COUNT>& values, u8 raw,
		PadDualshock2::Inputs negative, PadDualshock2::Inputs positive)
	{
		if (raw > PAD_ANALOG_NEUTRAL)
		{
			values[positive] =
				static_cast<float>(raw - PAD_ANALOG_NEUTRAL) / static_cast<float>(0xff - PAD_ANALOG_NEUTRAL);
		}
		else if (raw < PAD_ANALOG_NEUTRAL)
		{
			values[negative] = static_cast<float>(PAD_ANALOG_NEUTRAL - raw) / static_cast<float>(PAD_ANALOG_NEUTRAL);
		}
	}

	static bool ParseAnalogField(const rapidjson::Value& frame, PadFrame* out, ReplyContext& ctx)
	{
		const rapidjson::Value* analog = FindMember(frame, "analog");
		if (!analog)
			return true;
		if (!analog->IsObject())
		{
			ctx.Fail(ErrorCode::BadRequest, "'analog' must be an object");
			return false;
		}

		u32 axes[4] = {PAD_ANALOG_NEUTRAL, PAD_ANALOG_NEUTRAL, PAD_ANALOG_NEUTRAL, PAD_ANALOG_NEUTRAL};
		static constexpr const char* AXIS_NAMES[4] = {"lx", "ly", "rx", "ry"};
		for (size_t i = 0; i < 4; i++)
		{
			if (!GetUintField(*analog, AXIS_NAMES[i], 0, 0xff, PAD_ANALOG_NEUTRAL, false, &axes[i], ctx))
				return false;
		}

		// ly/ry follow the pad's convention rather than a maths one: 0x00 is up, 0xff is down.
		SetAnalogAxis(out->values, static_cast<u8>(axes[0]), PadDualshock2::Inputs::PAD_L_LEFT,
			PadDualshock2::Inputs::PAD_L_RIGHT);
		SetAnalogAxis(out->values, static_cast<u8>(axes[1]), PadDualshock2::Inputs::PAD_L_UP,
			PadDualshock2::Inputs::PAD_L_DOWN);
		SetAnalogAxis(out->values, static_cast<u8>(axes[2]), PadDualshock2::Inputs::PAD_R_LEFT,
			PadDualshock2::Inputs::PAD_R_RIGHT);
		SetAnalogAxis(out->values, static_cast<u8>(axes[3]), PadDualshock2::Inputs::PAD_R_UP,
			PadDualshock2::Inputs::PAD_R_DOWN);
		return true;
	}

	static bool ParsePadFrame(
		const rapidjson::Value& frame, u8 port, PadFrame* out, u32* repeat, ReplyContext& ctx)
	{
		if (!frame.IsObject())
		{
			ctx.Fail(ErrorCode::BadRequest, "each entry of 'frames' must be an object");
			return false;
		}

		out->port = port;
		out->values.fill(0.0f);

		if (const rapidjson::Value* buttons = FindMember(frame, "buttons"))
		{
			if (!buttons->IsArray())
			{
				ctx.Fail(ErrorCode::BadRequest, "'buttons' must be an array of strings");
				return false;
			}
			for (const rapidjson::Value& button : buttons->GetArray())
			{
				if (!button.IsString())
				{
					ctx.Fail(ErrorCode::BadRequest, "'buttons' must be an array of strings");
					return false;
				}

				const std::string_view name(button.GetString(), button.GetStringLength());
				const PadButtonName* found = nullptr;
				for (const PadButtonName& entry : PAD_BUTTON_NAMES)
				{
					if (name == entry.name)
					{
						found = &entry;
						break;
					}
				}
				if (!found)
				{
					ctx.Fail(ErrorCode::BadRequest,
						fmt::format("unknown button '{}' (known: {})", name, PadButtonNameList()));
					return false;
				}
				out->values[static_cast<size_t>(found->bind)] = 1.0f;
			}
		}

		if (!ParseAnalogField(frame, out, ctx))
			return false;

		return GetUintField(frame, "repeat", 1, MAX_INPUT_REPEAT, 1, false, repeat, ctx);
	}

	static void OpSendInput(const rapidjson::Value& request, ReplyContext& ctx)
	{
		u32 port = 0;
		if (!GetUintField(request, "port", 0, Pad::NUM_CONTROLLER_PORTS - 1, 0, false, &port, ctx))
			return;

		const rapidjson::Value* frames = FindMember(request, "frames");
		if (!frames || !frames->IsArray())
		{
			ctx.Fail(ErrorCode::BadRequest, "'frames' is required and must be an array");
			return;
		}

		// Built fully before anything is queued, so a bad entry halfway down rejects the whole
		// request rather than leaving half a button sequence to play out.
		std::deque<PadFrame> parsed;
		for (const rapidjson::Value& frame : frames->GetArray())
		{
			PadFrame pad_frame;
			u32 repeat = 1;
			if (!ParsePadFrame(frame, static_cast<u8>(port), &pad_frame, &repeat, ctx))
				return;

			if (parsed.size() + repeat > MAX_INPUT_QUEUE_FRAMES)
			{
				ctx.Fail(ErrorCode::TooLarge,
					fmt::format("the queued sequence would exceed {} frames", MAX_INPUT_QUEUE_FRAMES));
				return;
			}
			for (u32 i = 0; i < repeat; i++)
				parsed.push_back(pad_frame);
		}

		size_t queued = 0;
		{
			std::lock_guard<std::mutex> lock(s_input_mtx);
			if (s_input_queue.size() + parsed.size() > MAX_INPUT_QUEUE_FRAMES)
			{
				ctx.Fail(ErrorCode::TooLarge,
					fmt::format("the input queue holds at most {} frames", MAX_INPUT_QUEUE_FRAMES));
				return;
			}
			s_input_queue.insert(s_input_queue.end(), parsed.begin(), parsed.end());
			queued = s_input_queue.size();
		}

		ctx.payload = [queued](JsonWriter& writer) {
			writer.Key("queued");
			writer.Uint64(queued);
		};
	}

	static void OpClearInput(const rapidjson::Value&, ReplyContext& ctx)
	{
		size_t dropped = 0;
		{
			std::lock_guard<std::mutex> lock(s_input_mtx);
			dropped = s_input_queue.size();
			s_input_queue.clear();
		}

		// The release itself happens on the next poll, which is also the next time the game
		// could read the pad -- so nothing stays held for a frame the game can observe.
		ctx.payload = [dropped](JsonWriter& writer) {
			writer.Key("dropped");
			writer.Uint64(dropped);
		};
	}

	static void Dispatch(std::string_view op, const rapidjson::Value& request, ReplyContext& ctx)
	{
		if (op == "ping")
			OpPing(request, ctx);
		else if (op == "status")
			OpStatus(request, ctx);
		else if (op == "read_bytes")
			OpReadBytes(request, ctx);
		else if (op == "read_many")
			OpReadMany(request, ctx);
		else if (op == "write_bytes")
			OpWriteBytes(request, ctx);
		else if (op == "pause")
			OpSetPaused(request, ctx, true);
		else if (op == "resume")
			OpSetPaused(request, ctx, false);
		else if (op == "step")
			OpStep(request, ctx);
		else if (op == "reset")
			OpReset(request, ctx);
		else if (op == "screenshot")
			OpScreenshot(request, ctx);
		else if (op == "save_state")
			OpSaveState(request, ctx);
		else if (op == "load_state")
			OpLoadState(request, ctx);
		else if (op == "gs_dump")
			OpGsDump(request, ctx);
		else if (op == "send_input")
			OpSendInput(request, ctx);
		else if (op == "clear_input")
			OpClearInput(request, ctx);
		else
			ctx.Fail(ErrorCode::UnknownOp, fmt::format("unknown op '{}'", op));
	}
} // namespace ControlServer

// ---------------------------------------------------------------------------
// Line handling
// ---------------------------------------------------------------------------

namespace ControlServer
{
	/// Writes the reply envelope. Every reply, successful or not, carries the frame and
	/// state block, so a caller never has to issue a separate status request to know what
	/// the machine was doing when its request landed.
	static void WriteReply(JsonWriter& writer, const rapidjson::Value* id, const ReplyContext& ctx)
	{
		writer.StartObject();

		writer.Key("id");
		if (id)
			id->Accept(writer);
		else
			writer.Null();

		writer.Key("ok");
		writer.Bool(ctx.Ok());

		// g_FrameCount increments at vsync *end* while a frame advance auto-pauses at vsync
		// start, so this trails s_vsync_count by one at the moment a step completes. It is
		// reported because it cross-references against PCSX2 logs and pnaches; s_vsync_count
		// is the one to actually count with.
		writer.Key("frame");
		writer.Uint(g_FrameCount);
		writer.Key("vsync");
		writer.Uint64(s_vsync_count.load(std::memory_order_acquire));
		writer.Key("state");
		WriteString(writer, VMStateName(VMManager::GetState()));

		if (ctx.Ok())
		{
			if (ctx.payload)
				ctx.payload(writer);
		}
		else
		{
			writer.Key("error");
			writer.StartObject();
			writer.Key("code");
			WriteString(writer, ErrorCodeName(ctx.code));
			writer.Key("message");
			WriteString(writer, ctx.message);
			writer.EndObject();
		}

		writer.EndObject();
	}

	static std::string BuildReply(const rapidjson::Value* id, const ReplyContext& ctx)
	{
		rapidjson::StringBuffer buffer;
		JsonWriter writer(buffer);
		WriteReply(writer, id, ctx);

		std::string out(buffer.GetString(), buffer.GetSize());
		out.push_back('\n');
		return out;
	}

	static std::string HandleLine(std::string_view line)
	{
		rapidjson::Document document;
		document.Parse<rapidjson::kParseStopWhenDoneFlag>(line.data(), line.size());

		ReplyContext ctx;
		const rapidjson::Value* id = nullptr;

		if (document.HasParseError() || !document.IsObject())
		{
			ctx.Fail(ErrorCode::BadRequest, "the request must be a single JSON object");
		}
		else
		{
			id = FindMember(document, "id");
			if (id && (id->IsObject() || id->IsArray()))
			{
				// Echoing an arbitrarily large structure back is not worth supporting.
				id = nullptr;
				ctx.Fail(ErrorCode::BadRequest, "'id' must be a scalar");
			}
			else
			{
				const rapidjson::Value* op = FindMember(document, "op");
				if (!op || !op->IsString())
					ctx.Fail(ErrorCode::BadRequest, "'op' is required and must be a string");
				else
					Dispatch(std::string_view(op->GetString(), op->GetStringLength()), document, ctx);
			}
		}

		return BuildReply(id, ctx);
	}

	static bool SendAll(const char* data, size_t length)
	{
		size_t sent = 0;
		while (sent < length)
		{
			const auto chunk = static_cast<int>(std::min<size_t>(length - sent, SEND_CHUNK_BYTES));
			const auto written = write_portable(s_msgsock, data + sent, chunk);
			if (written <= 0)
				return false;
			sent += static_cast<size_t>(written);
		}
		return true;
	}

	static bool SendFatalError(const char* message)
	{
		ReplyContext ctx;
		ctx.Fail(ErrorCode::ProtocolError, message);
		const std::string reply = BuildReply(nullptr, ctx);
		return SendAll(reply.data(), reply.size());
	}
} // namespace ControlServer

// ---------------------------------------------------------------------------
// Socket loops
// ---------------------------------------------------------------------------

namespace ControlServer
{
	bool AcceptClient()
	{
		s_msgsock = accept(s_sock, nullptr, nullptr);
		if (s_msgsock == INVALID_SOCKET_VALUE)
		{
#ifdef _WIN32
			const int error = WSAGetLastError();
			if (!(error == WSAECONNRESET || error == WSAEINTR || error == WSAEINPROGRESS || error == WSAEMFILE ||
					error == WSAEWOULDBLOCK) &&
				s_sock != INVALID_SOCKET_VALUE)
			{
				Console.Error("ControlServer: accept() returned error %d", error);
			}
#else
			if (!(errno == ECONNABORTED || errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) && s_sock >= 0)
				Console.Error("ControlServer: accept() returned error %d", errno);
#endif
			return false;
		}

		// Without this, Nagle plus delayed ACK adds up to 40ms to every round trip, which
		// would dominate the cost of the small reads this server exists to serve.
		const int nodelay = 1;
		setsockopt(s_msgsock, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));

#ifdef __APPLE__
		const int nosigpipe = 1;
		setsockopt(s_msgsock, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));
#endif

		Console.WriteLn("ControlServer: client connected.");
		return true;
	}

	void MainLoop()
	{
		Threading::SetNameOfCurrentThread("Control Server");

		while (!s_end.load(std::memory_order_acquire))
		{
			if (!AcceptClient())
				continue;

			ClientLoop();

			Console.WriteLn("ControlServer: client disconnected.");
			safe_close_portable(s_msgsock);
		}
	}

	void ClientLoop()
	{
		// Clients are served one at a time; a second connection waits in the listen backlog
		// until this one goes away.
		std::string received;
		std::vector<char> chunk(RECV_CHUNK_BYTES);

		while (!s_end.load(std::memory_order_acquire))
		{
			const auto length = read_portable(s_msgsock, chunk.data(), static_cast<int>(chunk.size()));
			if (length <= 0)
				return;

			received.append(chunk.data(), static_cast<size_t>(length));

			size_t start = 0;
			for (;;)
			{
				const size_t newline = received.find('\n', start);
				if (newline == std::string::npos)
					break;

				std::string_view line(received.data() + start, newline - start);
				if (!line.empty() && line.back() == '\r')
					line.remove_suffix(1);
				start = newline + 1;

				// Tolerate blank lines, so a client can use them as a keepalive.
				if (line.empty())
					continue;

				const std::string reply = HandleLine(line);
				if (!SendAll(reply.data(), reply.size()))
					return;
			}
			received.erase(0, start);

			if (received.size() > MAX_LINE_BYTES)
			{
				// Unrecoverable: we have no idea where the next line boundary is meant to be.
				SendFatalError("the request line exceeded the maximum length");
				return;
			}
		}
	}
} // namespace ControlServer

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool ControlServer::IsInitialized()
{
	return !s_end.load(std::memory_order_acquire);
}

int ControlServer::GetPort()
{
	return s_port;
}

bool ControlServer::Initialize(int port)
{
	if (port <= 0 || port > 65535)
	{
		Console.Error("ControlServer: %d is not a valid port.", port);
		return false;
	}

	s_end.store(false, std::memory_order_release);
	s_port = port;

#ifdef _WIN32
	if (!InitializeControlServerWinsock())
	{
		Console.Error("ControlServer: cannot initialize winsock, shutting down.");
		Deinitialize();
		return false;
	}
#endif

	s_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (s_sock == INVALID_SOCKET_VALUE)
	{
		Console.Error("ControlServer: cannot open socket, shutting down.");
		Deinitialize();
		return false;
	}

	// So restarting on the same port does not trip over a lingering TIME_WAIT.
	const int reuse = 1;
	setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));

	sockaddr_in server = {};
	server.sin_family = AF_INET;
	// Loopback only, never routable: there is no authentication, and write_bytes is an
	// arbitrary memory write into this process.
	server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	server.sin_port = htons(static_cast<u16>(port));

	if (bind(s_sock, reinterpret_cast<struct sockaddr*>(&server), sizeof(server)) != 0)
	{
		Console.Error("ControlServer: cannot bind to port %d; is another instance using it?", port);
		Deinitialize();
		return false;
	}

	if (listen(s_sock, 1) != 0)
	{
		Console.Error("ControlServer: cannot listen for connections, shutting down.");
		Deinitialize();
		return false;
	}

	s_thread = std::thread(&ControlServer::MainLoop);

	Console.WriteLn("ControlServer: listening on 127.0.0.1:%d.", port);
	return true;
}

void ControlServer::Deinitialize()
{
	s_end.store(true, std::memory_order_release);

	// Wake anything parked on a CPU-thread job *before* the join below. This runs from
	// CPUThreadShutdown(), so a server thread still waiting on the CPU thread would
	// otherwise deadlock that join -- and it would do so as a race, only sometimes.
	CancelAllPendingJobs(ErrorCode::Interrupted, "the server is shutting down");

	// shutdown() is needed as well as close(), otherwise accept() and recv() keep blocking.
#ifdef _WIN32
	if (s_sock != INVALID_SOCKET_VALUE)
		shutdown(s_sock, SD_BOTH);
	if (s_msgsock != INVALID_SOCKET_VALUE)
		shutdown(s_msgsock, SD_BOTH);
#else
	if (s_sock >= 0)
		shutdown(s_sock, SHUT_RDWR);
	if (s_msgsock >= 0)
		shutdown(s_msgsock, SHUT_RDWR);
#endif

	safe_close_portable(s_sock);
	safe_close_portable(s_msgsock);

	if (s_thread.joinable())
		s_thread.join();

	s_port = 0;
}

// ---------------------------------------------------------------------------
// VMManager hooks
// ---------------------------------------------------------------------------

void ControlServer::Internal::OnVSyncOnCPUThread()
{
	s_vsync_count.fetch_add(1, std::memory_order_acq_rel);
}

void ControlServer::Internal::OnPollInputOnCPUThread()
{
	PadFrame frame;
	{
		std::lock_guard<std::mutex> lock(s_input_mtx);
		if (!s_input_queue.empty())
		{
			frame = s_input_queue.front();
			s_input_queue.pop_front();
			s_input_ports_touched |= (1u << frame.port);
		}
		else if (s_input_ports_touched != 0)
		{
			// The queue just drained. InputManager only writes a bind when something actually
			// changed, so without an explicit release here the last injected frame's buttons
			// would stay held forever. This runs before the game reads the pad this frame, so
			// the release costs no emulated frame.
			const u32 ports = std::exchange(s_input_ports_touched, 0u);
			for (u32 port = 0; port < Pad::NUM_CONTROLLER_PORTS; port++)
			{
				if (ports & (1u << port))
					Pad::ResetControllerInputs(port);
			}
			return;
		}
		else
		{
			// Nothing injected and nothing to release: leave the real controller alone.
			return;
		}
	}

	// Bind indices are DualShock2's; writing them to a Guitar or a negcon would press
	// whatever happens to sit at those offsets.
	const PadBase* pad = Pad::GetPad(frame.port);
	if (!pad || pad->GetType() != Pad::ControllerType::DualShock2)
		return;

	for (size_t bind = 0; bind < PAD_BIND_COUNT; bind++)
		Pad::SetControllerState(frame.port, static_cast<u32>(bind), frame.values[bind]);
}

void ControlServer::Internal::OnFrameAdvanceCompleted()
{
	std::shared_ptr<Job> job;
	{
		std::lock_guard<std::mutex> lock(s_step_mtx);
		job = std::move(s_active_step);

		// SetState(Paused) follows immediately, but Complete() below wakes the socket thread,
		// and SetState() blocks in WaitVU()/WaitGS() long before it reaches OnVMStateChanged()
		// -- so the *next* step can already be registered by the time that pause lands. Mark
		// the pause as ours so it is not mistaken for an interruption of that next step.
		s_expect_auto_pause = true;
	}

	// Null when the frame advance came from the UI hotkey rather than from a step request.
	if (job)
		job->Complete(ErrorCode::None);
}

void ControlServer::Internal::OnVMStateChanged(VMState new_state)
{
	std::shared_ptr<Job> job;
	{
		std::lock_guard<std::mutex> lock(s_step_mtx);

		// Both hooks run on the CPU thread and nothing calls SetState() between them, so a
		// flag set by OnFrameAdvanceCompleted() can only belong to the pause right after it.
		const bool was_auto_pause = std::exchange(s_expect_auto_pause, false);
		if (was_auto_pause && new_state == VMState::Paused)
			return;

		// FrameAdvance() itself transitions to Running; that is the step starting, not ending.
		if (new_state == VMState::Running)
			return;

		job = std::move(s_active_step);
	}

	// A step that is still active here was interrupted by something else -- the user
	// pausing from the UI, a reset, or a shutdown. Our own completion already claimed the
	// job in OnFrameAdvanceCompleted() before SetState(Paused) got here.
	if (job)
	{
		job->Complete(ErrorCode::Interrupted,
			fmt::format("the VM entered state '{}' during the step", VMStateName(new_state)));
	}
}
