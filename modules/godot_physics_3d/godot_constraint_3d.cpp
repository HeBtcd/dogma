// YGIAH (det-snapshot)
#include "godot_constraint_3d.h"
#include "godot_body_3d.h"

void GodotConstraint3D::_det_get_key(uint64_t r_key[5]) const {
	uint64_t lo = UINT64_MAX, hi = 0;
	int lo_sub = 0, hi_sub = 0;
	for (int i = 0; i < _body_count; i++) {
		const GodotBody3D *b = _body_ptr[i];
		uint64_t ord = b ? b->get_det_ordinal() : UINT64_MAX;
		if (ord < lo) {
			lo = ord;
			lo_sub = i;
		}
		if (ord > hi) {
			hi = ord;
			hi_sub = i;
		}
	}
	r_key[0] = lo;
	r_key[1] = (uint64_t)lo_sub;
	r_key[2] = hi;
	r_key[3] = (uint64_t)hi_sub;
	r_key[4] = _det_kind();
}
