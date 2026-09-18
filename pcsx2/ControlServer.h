// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "common/Pcsx2Defs.h"

enum class VMState;

/// Loopback JSON control server.
///
/// Speaks a line-delimited JSON protocol over TCP on 127.0.0.1, intended for driving
/// the emulator from external tooling: bulk memory inspection, frame-accurate stepping
/// and screenshots. It complements PINE rather than replacing it -- PINE's binary
/// framing is cheaper on the wire, this one is cheaper to extend.
///
/// Security: there is no authentication, and write_bytes is an arbitrary memory write
/// into the emulator process. Disabled by default (EmuCore/EnableControlServer), and
/// the listener is only ever bound to loopback.
namespace ControlServer
{
	/// Reported by "ping". Bump on breaking wire changes.
	inline constexpr u32 PROTOCOL_VERSION = 1;

	inline constexpr int DEFAULT_PORT = 28015;

	bool IsInitialized();
	int GetPort();

	/// Starts the listener thread. CPU thread only.
	bool Initialize(int port = DEFAULT_PORT);

	/// Cancels in-flight requests, closes the sockets and joins the server thread.
	/// CPU thread only. Must run before MTGS::ShutdownThread() and SysMemory::Release(),
	/// otherwise a pending screenshot or memory read can outlive what it is reading.
	void Deinitialize();

	namespace Internal
	{
		/// Called once per vsync from VMManager::Internal::VSyncOnCPUThread().
		void OnVSyncOnCPUThread();
		void OnPollInputOnCPUThread();

		/// Called when a frame advance run reaches zero, immediately *before* the
		/// SetState(Paused) that ends it -- so that OnVMStateChanged() below does not
		/// mistake our own completion for an external interruption.
		void OnFrameAdvanceCompleted();

		/// Called from the tail of VMManager::SetState().
		void OnVMStateChanged(VMState new_state);
	} // namespace Internal
} // namespace ControlServer
