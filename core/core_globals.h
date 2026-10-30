/**************************************************************************/
/*  core_globals.h                                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include <cstdint>

// Home for state needed from global functions
// that cannot be stored in Engine or OS due to e.g. circular includes

class CoreGlobals {
public:
	static inline bool leak_reporting_enabled = true;
	static inline bool print_line_enabled = true;
	static inline bool print_error_enabled = true;
	static inline bool print_ready = false;

	// YGIAH (2026-10-31): Deterministic mode ("det-mode"), activated via
	// --deterministic / --det-seed N or GODOT_DETERMINISTIC=1 in Main::setup.
	// When true the engine removes or loudly rejects every nondeterminism that
	// leaked scheduler state into game state (godot-det-test attacks B1-B6):
	// WorkerThreadPool tasks run inline on the submitting thread, off-main-thread
	// MessageQueue submissions / ObjectDB registrations / server RID writes are
	// rejected, load_threaded_request runs synchronously, RandomPCG::randomize()
	// draws from the fixed sequence below, and frame pacing is pinned to a
	// fixed delta. Kept in CoreGlobals (not Engine) because ObjectDB/CallQueue/
	// RID_Alloc cannot afford an Engine dependency.
	static inline bool deterministic_mode = false;
	static inline uint64_t deterministic_seed = 0;
	static inline uint64_t deterministic_randomize_count = 0;
};
