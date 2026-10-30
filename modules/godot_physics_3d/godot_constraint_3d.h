/**************************************************************************/
/*  godot_constraint_3d.h                                                 */
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

#include "core/templates/rid.h"
#include "core/typedefs.h"
#include "core/variant/dictionary.h"

class GodotBody3D;
class GodotSoftBody3D;

class GodotConstraint3D {
	GodotBody3D **_body_ptr;
	int _body_count;
	uint64_t island_step;
	int priority;
	bool disabled_collisions_between_bodies;

	RID self;

protected:
	GodotConstraint3D(GodotBody3D **p_body_ptr = nullptr, int p_body_count = 0) {
		_body_ptr = p_body_ptr;
		_body_count = p_body_count;
		island_step = 0;
		priority = 1;
		disabled_collisions_between_bodies = true;
	}

public:
	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	_FORCE_INLINE_ uint64_t get_island_step() const { return island_step; }
	_FORCE_INLINE_ void set_island_step(uint64_t p_step) { island_step = p_step; }

	_FORCE_INLINE_ GodotBody3D **get_body_ptr() const { return _body_ptr; }
	_FORCE_INLINE_ int get_body_count() const { return _body_count; }

	virtual GodotSoftBody3D *get_soft_body_ptr(int p_index) const { return nullptr; }
	virtual int get_soft_body_count() const { return 0; }

	_FORCE_INLINE_ void set_priority(int p_priority) { priority = MAX(0, p_priority); }
	_FORCE_INLINE_ int get_priority() const { return priority; }

	_FORCE_INLINE_ void disable_collisions_between_bodies(const bool p_disabled) { disabled_collisions_between_bodies = p_disabled; }
	_FORCE_INLINE_ bool is_disabled_collisions_between_bodies() const { return disabled_collisions_between_bodies; }

	virtual bool setup(real_t p_step) = 0;
	virtual bool pre_solve(real_t p_step) = 0;
	virtual void solve(real_t p_step) = 0;

	// YGIAH (det-snapshot): canonical sort key so constraint order never
	// leaks pointer/hash order into the solver. r_key is a 5-element tuple:
	// {lo_body_ord, lo_subindex, hi_body_ord, hi_subindex, kind}.
	virtual void _det_get_key(uint64_t r_key[5]) const;
	// 0 = generic/joint, 1 = body-body contact pair, 2 = body-soft pair,
	// 3+ = area pairs. Used as last sort-key element and by state export.
	virtual uint32_t _det_kind() const { return 0; }

	// YGIAH (det-snapshot): solver-internal persistent state for
	// joint/constraints that cannot be re-derived from the scene. Body pairs
	// and joints override; an empty export means "nothing to serialize".
	virtual Dictionary _det_export_state() const { return Dictionary(); }
	virtual Error _det_import_state(const Dictionary &) { return ERR_UNAVAILABLE; }

	virtual ~GodotConstraint3D() {}
};

// YGIAH (det-snapshot): canonical comparator — (lo_ord, lo_sub, hi_ord,
// hi_sub, kind). Pointer/hash order must never decide solver order.
inline bool godot_det_constraint_less(GodotConstraint3D *a, GodotConstraint3D *b) {
	uint64_t ka[5], kb[5];
	a->_det_get_key(ka);
	b->_det_get_key(kb);
	for (int i = 0; i < 5; i++) {
		if (ka[i] != kb[i]) {
			return ka[i] < kb[i];
		}
	}
	// YGIAH (merge fix): same-key multi-constraints (e.g. two joints on one
	// body pair — adv_stress rb19/rb72 flake) still need a canonical order.
	// The constraint's self-RID id is allocated in creation order on the main
	// thread — deterministic, unlike the pointer (ASLR-decided, observed to
	// flip run-to-run and diverge the solver). Pointer remains only as the
	// unreachable total-order backstop for RID-less constraints.
	if (a->get_self() != b->get_self()) {
		return a->get_self() < b->get_self();
	}
	return a < b;
}

struct GodotDetConstraintLess {
	_FORCE_INLINE_ bool operator()(GodotConstraint3D *a, GodotConstraint3D *b) const {
		return godot_det_constraint_less(a, b);
	}
};

struct GodotDetCMapEntry {
	GodotConstraint3D *key;
	int value;
};

struct GodotDetCMapEntryLess {
	_FORCE_INLINE_ bool operator()(const GodotDetCMapEntry &a, const GodotDetCMapEntry &b) const {
		return godot_det_constraint_less(a.key, b.key);
	}
};
