/**************************************************************************/
/*  godot_space_3d.cpp                                                    */
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

#include "godot_space_3d.h"

#include "godot_area_pair_3d.h"
#include "godot_body_pair_3d.h"
#include "godot_collision_solver_3d.h"
#include "godot_joint_3d.h"
#include "godot_physics_server_3d.h"

#include "core/config/project_settings.h"
#include "core/object/object.h"
#include "scene/main/node.h"

#define TEST_MOTION_MARGIN_MIN_VALUE 0.0001
#define TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR 0.05

_FORCE_INLINE_ static bool _can_collide_with(GodotCollisionObject3D *p_object, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (!(p_object->get_collision_layer() & p_collision_mask)) {
		return false;
	}

	if (p_object->get_type() == GodotCollisionObject3D::TYPE_AREA && !p_collide_with_areas) {
		return false;
	}

	if (p_object->get_type() == GodotCollisionObject3D::TYPE_BODY && !p_collide_with_bodies) {
		return false;
	}

	if (p_object->get_type() == GodotCollisionObject3D::TYPE_SOFT_BODY && !p_collide_with_bodies) {
		return false;
	}

	return true;
}

int GodotPhysicsDirectSpaceState3D::intersect_point(const PS3DT::PointParameters &p_parameters, PS3DT::ShapeResult *r_results, int p_result_max) {
	ERR_FAIL_COND_V(space->locked, false);
	int amount = space->broadphase->cull_point(p_parameters.position, space->intersection_query_results, GodotSpace3D::INTERSECTION_QUERY_MAX, space->intersection_query_subindex_results);
	int cc = 0;

	//Transform3D ai = p_xform.affine_inverse();

	for (int i = 0; i < amount; i++) {
		if (cc >= p_result_max) {
			break;
		}

		if (!_can_collide_with(space->intersection_query_results[i], p_parameters.collision_mask, p_parameters.collide_with_bodies, p_parameters.collide_with_areas)) {
			continue;
		}

		//area can't be picked by ray (default)

		if (p_parameters.exclude.has(space->intersection_query_results[i]->get_self())) {
			continue;
		}

		const GodotCollisionObject3D *col_obj = space->intersection_query_results[i];
		int shape_idx = space->intersection_query_subindex_results[i];

		Transform3D inv_xform = col_obj->get_transform() * col_obj->get_shape_transform(shape_idx);
		inv_xform.affine_invert();

		if (!col_obj->get_shape(shape_idx)->intersect_point(inv_xform.xform(p_parameters.position))) {
			continue;
		}

		r_results[cc].collider_id = col_obj->get_instance_id();
		r_results[cc].rid = col_obj->get_self();
		r_results[cc].shape = shape_idx;

		cc++;
	}

	return cc;
}

bool GodotPhysicsDirectSpaceState3D::intersect_ray(const PS3DT::RayParameters &p_parameters, PS3DT::RayResult &r_result) {
	ERR_FAIL_COND_V(space->locked, false);

	Vector3 begin, end;
	Vector3 normal;
	begin = p_parameters.from;
	end = p_parameters.to;
	normal = (end - begin).normalized();

	int amount = space->broadphase->cull_segment(begin, end, space->intersection_query_results, GodotSpace3D::INTERSECTION_QUERY_MAX, space->intersection_query_subindex_results);

	//todo, create another array that references results, compute AABBs and check closest point to ray origin, sort, and stop evaluating results when beyond first collision

	bool collided = false;
	Vector3 res_point, res_normal;
	int res_face_index = -1;
	int res_shape = -1;
	const GodotCollisionObject3D *res_obj = nullptr;
	real_t min_d = 1e10;

	for (int i = 0; i < amount; i++) {
		if (!_can_collide_with(space->intersection_query_results[i], p_parameters.collision_mask, p_parameters.collide_with_bodies, p_parameters.collide_with_areas)) {
			continue;
		}

		if (p_parameters.pick_ray && !(space->intersection_query_results[i]->is_ray_pickable())) {
			continue;
		}

		if (p_parameters.exclude.has(space->intersection_query_results[i]->get_self())) {
			continue;
		}

		const GodotCollisionObject3D *col_obj = space->intersection_query_results[i];

		int shape_idx = space->intersection_query_subindex_results[i];
		Transform3D inv_xform = col_obj->get_shape_inv_transform(shape_idx) * col_obj->get_inv_transform();

		Vector3 local_from = inv_xform.xform(begin);
		Vector3 local_to = inv_xform.xform(end);

		const GodotShape3D *shape = col_obj->get_shape(shape_idx);

		Vector3 shape_point, shape_normal;
		int shape_face_index = -1;

		if (shape->intersect_point(local_from)) {
			if (p_parameters.hit_from_inside) {
				// Hit shape at starting point.
				min_d = 0;
				res_point = begin;
				res_normal = Vector3();
				res_shape = shape_idx;
				res_obj = col_obj;
				collided = true;
				break;
			} else {
				// Ignore shape when starting inside.
				continue;
			}
		}

		if (shape->intersect_segment(local_from, local_to, shape_point, shape_normal, shape_face_index, p_parameters.hit_back_faces)) {
			Transform3D xform = col_obj->get_transform() * col_obj->get_shape_transform(shape_idx);
			shape_point = xform.xform(shape_point);

			real_t ld = normal.dot(shape_point);

			if (ld < min_d) {
				min_d = ld;
				res_point = shape_point;
				res_normal = inv_xform.basis.xform_inv(shape_normal).normalized();
				res_face_index = shape_face_index;
				res_shape = shape_idx;
				res_obj = col_obj;
				collided = true;
			}
		}
	}

	if (!collided) {
		return false;
	}
	ERR_FAIL_NULL_V(res_obj, false); // Shouldn't happen but silences warning.

	r_result.collider_id = res_obj->get_instance_id();
	r_result.normal = res_normal;
	r_result.face_index = res_face_index;
	r_result.position = res_point;
	r_result.rid = res_obj->get_self();
	r_result.shape = res_shape;

	return true;
}

int GodotPhysicsDirectSpaceState3D::intersect_shape(const PS3DT::ShapeParameters &p_parameters, PS3DT::ShapeResult *r_results, int p_result_max) {
	if (p_result_max <= 0) {
		return 0;
	}

	GodotShape3D *shape = GodotPhysicsServer3D::godot_singleton->shape_owner.get_or_null(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, 0);

	AABB aabb = p_parameters.transform.xform(shape->get_aabb());

	int amount = space->broadphase->cull_aabb(aabb, space->intersection_query_results, GodotSpace3D::INTERSECTION_QUERY_MAX, space->intersection_query_subindex_results);

	int cc = 0;

	//Transform3D ai = p_xform.affine_inverse();

	for (int i = 0; i < amount; i++) {
		if (cc >= p_result_max) {
			break;
		}

		if (!_can_collide_with(space->intersection_query_results[i], p_parameters.collision_mask, p_parameters.collide_with_bodies, p_parameters.collide_with_areas)) {
			continue;
		}

		//area can't be picked by ray (default)

		if (p_parameters.exclude.has(space->intersection_query_results[i]->get_self())) {
			continue;
		}

		const GodotCollisionObject3D *col_obj = space->intersection_query_results[i];
		int shape_idx = space->intersection_query_subindex_results[i];

		if (!GodotCollisionSolver3D::solve_static(shape, p_parameters.transform, col_obj->get_shape(shape_idx), col_obj->get_transform() * col_obj->get_shape_transform(shape_idx), nullptr, nullptr, nullptr, p_parameters.margin, 0)) {
			continue;
		}

		if (r_results) {
			r_results[cc].collider_id = col_obj->get_instance_id();
			r_results[cc].rid = col_obj->get_self();
			r_results[cc].shape = shape_idx;
		}

		cc++;
	}

	return cc;
}

bool GodotPhysicsDirectSpaceState3D::cast_motion(const PS3DT::ShapeParameters &p_parameters, real_t &p_closest_safe, real_t &p_closest_unsafe, PS3DT::ShapeRestInfo *r_info) {
	GodotShape3D *shape = GodotPhysicsServer3D::godot_singleton->shape_owner.get_or_null(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	AABB aabb = p_parameters.transform.xform(shape->get_aabb());
	aabb = aabb.merge(AABB(aabb.position + p_parameters.motion, aabb.size)); //motion
	aabb = aabb.grow(p_parameters.margin);

	int amount = space->broadphase->cull_aabb(aabb, space->intersection_query_results, GodotSpace3D::INTERSECTION_QUERY_MAX, space->intersection_query_subindex_results);

	real_t best_safe = 1;
	real_t best_unsafe = 1;

	Transform3D xform_inv = p_parameters.transform.affine_inverse();
	GodotMotionShape3D mshape;
	mshape.shape = shape;
	mshape.motion = xform_inv.basis.xform(p_parameters.motion);

	bool best_first = true;

	Vector3 motion_normal = p_parameters.motion.normalized();

	Vector3 closest_A, closest_B;

	for (int i = 0; i < amount; i++) {
		if (!_can_collide_with(space->intersection_query_results[i], p_parameters.collision_mask, p_parameters.collide_with_bodies, p_parameters.collide_with_areas)) {
			continue;
		}

		if (p_parameters.exclude.has(space->intersection_query_results[i]->get_self())) {
			continue; //ignore excluded
		}

		const GodotCollisionObject3D *col_obj = space->intersection_query_results[i];
		int shape_idx = space->intersection_query_subindex_results[i];

		Vector3 point_A, point_B;
		Vector3 sep_axis = motion_normal;

		Transform3D col_obj_xform = col_obj->get_transform() * col_obj->get_shape_transform(shape_idx);
		//test initial overlap, does it collide if going all the way?
		if (GodotCollisionSolver3D::solve_distance(&mshape, p_parameters.transform, col_obj->get_shape(shape_idx), col_obj_xform, point_A, point_B, aabb, &sep_axis)) {
			continue;
		}

		//test initial overlap, ignore objects it's inside of.
		sep_axis = motion_normal;

		if (!GodotCollisionSolver3D::solve_distance(shape, p_parameters.transform, col_obj->get_shape(shape_idx), col_obj_xform, point_A, point_B, aabb, &sep_axis)) {
			continue;
		}

		//just do kinematic solving
		real_t low = 0.0;
		real_t hi = 1.0;
		real_t fraction_coeff = 0.5;
		for (int j = 0; j < 8; j++) { //steps should be customizable..
			real_t fraction = low + (hi - low) * fraction_coeff;

			mshape.motion = xform_inv.basis.xform(p_parameters.motion * fraction);

			Vector3 lA, lB;
			Vector3 sep = motion_normal; //important optimization for this to work fast enough
			bool collided = !GodotCollisionSolver3D::solve_distance(&mshape, p_parameters.transform, col_obj->get_shape(shape_idx), col_obj_xform, lA, lB, aabb, &sep);

			if (collided) {
				hi = fraction;
				if ((j == 0) || (low > 0.0)) { // Did it not collide before?
					// When alternating or first iteration, use dichotomy.
					fraction_coeff = 0.5;
				} else {
					// When colliding again, converge faster towards low fraction
					// for more accurate results with long motions that collide near the start.
					fraction_coeff = 0.25;
				}
			} else {
				point_A = lA;
				point_B = lB;
				low = fraction;
				if ((j == 0) || (hi < 1.0)) { // Did it collide before?
					// When alternating or first iteration, use dichotomy.
					fraction_coeff = 0.5;
				} else {
					// When not colliding again, converge faster towards high fraction
					// for more accurate results with long motions that collide near the end.
					fraction_coeff = 0.75;
				}
			}
		}

		if (low < best_safe) {
			best_first = true; //force reset
			best_safe = low;
			best_unsafe = hi;
		}

		if (r_info && (best_first || (point_A.distance_squared_to(point_B) < closest_A.distance_squared_to(closest_B) && low <= best_safe))) {
			closest_A = point_A;
			closest_B = point_B;
			r_info->collider_id = col_obj->get_instance_id();
			r_info->rid = col_obj->get_self();
			r_info->shape = shape_idx;
			r_info->point = closest_B;
			r_info->normal = (closest_A - closest_B).normalized();
			best_first = false;
			if (col_obj->get_type() == GodotCollisionObject3D::TYPE_BODY) {
				const GodotBody3D *body = static_cast<const GodotBody3D *>(col_obj);
				Vector3 rel_vec = closest_B - (body->get_transform().origin + body->get_center_of_mass());
				r_info->linear_velocity = body->get_linear_velocity() + (body->get_angular_velocity()).cross(rel_vec);
			}
		}
	}

	p_closest_safe = best_safe;
	p_closest_unsafe = best_unsafe;

	return true;
}

bool GodotPhysicsDirectSpaceState3D::collide_shape(const PS3DT::ShapeParameters &p_parameters, Vector3 *r_results, int p_result_max, int &r_result_count) {
	if (p_result_max <= 0) {
		return false;
	}

	GodotShape3D *shape = GodotPhysicsServer3D::godot_singleton->shape_owner.get_or_null(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	AABB aabb = p_parameters.transform.xform(shape->get_aabb());
	aabb = aabb.grow(p_parameters.margin);

	int amount = space->broadphase->cull_aabb(aabb, space->intersection_query_results, GodotSpace3D::INTERSECTION_QUERY_MAX, space->intersection_query_subindex_results);

	bool collided = false;
	r_result_count = 0;

	GodotPhysicsServer3D::CollCbkData cbk;
	cbk.max = p_result_max;
	cbk.amount = 0;
	cbk.ptr = r_results;
	GodotCollisionSolver3D::CallbackResult cbkres = GodotPhysicsServer3D::_shape_col_cbk;

	GodotPhysicsServer3D::CollCbkData *cbkptr = &cbk;

	for (int i = 0; i < amount; i++) {
		if (!_can_collide_with(space->intersection_query_results[i], p_parameters.collision_mask, p_parameters.collide_with_bodies, p_parameters.collide_with_areas)) {
			continue;
		}

		const GodotCollisionObject3D *col_obj = space->intersection_query_results[i];

		if (p_parameters.exclude.has(col_obj->get_self())) {
			continue;
		}

		int shape_idx = space->intersection_query_subindex_results[i];

		if (GodotCollisionSolver3D::solve_static(shape, p_parameters.transform, col_obj->get_shape(shape_idx), col_obj->get_transform() * col_obj->get_shape_transform(shape_idx), cbkres, cbkptr, nullptr, p_parameters.margin)) {
			collided = true;
		}
	}

	r_result_count = cbk.amount;

	return collided;
}

struct _RestResultData {
	const GodotCollisionObject3D *object = nullptr;
	int local_shape = 0;
	int shape = 0;
	Vector3 contact;
	Vector3 normal;
	real_t len = 0.0;
};

struct _RestCallbackData {
	const GodotCollisionObject3D *object = nullptr;
	int local_shape = 0;
	int shape = 0;

	real_t min_allowed_depth = 0.0;

	_RestResultData best_result;

	int max_results = 0;
	int result_count = 0;
	_RestResultData *other_results = nullptr;
};

static void _rest_cbk_result(const Vector3 &p_point_A, int p_index_A, const Vector3 &p_point_B, int p_index_B, const Vector3 &normal, void *p_userdata) {
	_RestCallbackData *rd = static_cast<_RestCallbackData *>(p_userdata);

	Vector3 contact_rel = p_point_B - p_point_A;
	real_t len = contact_rel.length();
	if (len < rd->min_allowed_depth) {
		return;
	}

	bool is_best_result = (len > rd->best_result.len);

	if (rd->other_results && rd->result_count > 0) {
		// Consider as new result by default.
		int prev_result_count = rd->result_count++;

		int result_index = 0;
		real_t tested_len = is_best_result ? rd->best_result.len : len;
		for (; result_index < prev_result_count - 1; ++result_index) {
			if (tested_len > rd->other_results[result_index].len) {
				// Reusing a previous result.
				rd->result_count--;
				break;
			}
		}

		if (result_index < rd->max_results - 1) {
			_RestResultData &result = rd->other_results[result_index];

			if (is_best_result) {
				// Keep the previous best result as separate result.
				result = rd->best_result;
			} else {
				// Keep this result as separate result.
				result.len = len;
				result.contact = p_point_B;
				result.normal = normal;
				result.object = rd->object;
				result.shape = rd->shape;
				result.local_shape = rd->local_shape;
			}
		} else {
			// Discarding this result.
			rd->result_count--;
		}
	} else if (is_best_result) {
		rd->result_count = 1;
	}

	if (!is_best_result) {
		return;
	}

	rd->best_result.len = len;
	rd->best_result.contact = p_point_B;
	rd->best_result.normal = normal;
	rd->best_result.object = rd->object;
	rd->best_result.shape = rd->shape;
	rd->best_result.local_shape = rd->local_shape;
}

bool GodotPhysicsDirectSpaceState3D::rest_info(const PS3DT::ShapeParameters &p_parameters, PS3DT::ShapeRestInfo *r_info) {
	GodotShape3D *shape = GodotPhysicsServer3D::godot_singleton->shape_owner.get_or_null(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	real_t margin = MAX(p_parameters.margin, TEST_MOTION_MARGIN_MIN_VALUE);

	AABB aabb = p_parameters.transform.xform(shape->get_aabb());
	aabb = aabb.grow(margin);

	int amount = space->broadphase->cull_aabb(aabb, space->intersection_query_results, GodotSpace3D::INTERSECTION_QUERY_MAX, space->intersection_query_subindex_results);

	_RestCallbackData rcd;

	// Allowed depth can't be lower than motion length, in order to handle contacts at low speed.
	real_t motion_length = p_parameters.motion.length();
	real_t min_contact_depth = margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;
	rcd.min_allowed_depth = MIN(motion_length, min_contact_depth);

	for (int i = 0; i < amount; i++) {
		if (!_can_collide_with(space->intersection_query_results[i], p_parameters.collision_mask, p_parameters.collide_with_bodies, p_parameters.collide_with_areas)) {
			continue;
		}

		const GodotCollisionObject3D *col_obj = space->intersection_query_results[i];

		if (p_parameters.exclude.has(col_obj->get_self())) {
			continue;
		}

		int shape_idx = space->intersection_query_subindex_results[i];

		rcd.object = col_obj;
		rcd.shape = shape_idx;
		bool sc = GodotCollisionSolver3D::solve_static(shape, p_parameters.transform, col_obj->get_shape(shape_idx), col_obj->get_transform() * col_obj->get_shape_transform(shape_idx), _rest_cbk_result, &rcd, nullptr, margin);
		if (!sc) {
			continue;
		}
	}

	if (rcd.best_result.len == 0 || !rcd.best_result.object) {
		return false;
	}

	r_info->collider_id = rcd.best_result.object->get_instance_id();
	r_info->shape = rcd.best_result.shape;
	r_info->normal = rcd.best_result.normal;
	r_info->point = rcd.best_result.contact;
	r_info->rid = rcd.best_result.object->get_self();
	if (rcd.best_result.object->get_type() == GodotCollisionObject3D::TYPE_BODY) {
		const GodotBody3D *body = static_cast<const GodotBody3D *>(rcd.best_result.object);
		Vector3 rel_vec = rcd.best_result.contact - (body->get_transform().origin + body->get_center_of_mass());
		r_info->linear_velocity = body->get_linear_velocity() + (body->get_angular_velocity()).cross(rel_vec);

	} else {
		r_info->linear_velocity = Vector3();
	}

	return true;
}

Vector3 GodotPhysicsDirectSpaceState3D::get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const {
	GodotCollisionObject3D *obj = GodotPhysicsServer3D::godot_singleton->area_owner.get_or_null(p_object);
	if (!obj) {
		obj = GodotPhysicsServer3D::godot_singleton->body_owner.get_or_null(p_object);
	}
	ERR_FAIL_NULL_V(obj, Vector3());

	ERR_FAIL_COND_V(obj->get_space() != space, Vector3());

	real_t min_distance = 1e20;
	Vector3 min_point;

	bool shapes_found = false;

	for (int i = 0; i < obj->get_shape_count(); i++) {
		if (obj->is_shape_disabled(i)) {
			continue;
		}

		Transform3D shape_xform = obj->get_transform() * obj->get_shape_transform(i);
		GodotShape3D *shape = obj->get_shape(i);

		Vector3 point = shape->get_closest_point_to(shape_xform.affine_inverse().xform(p_point));
		point = shape_xform.xform(point);

		real_t dist = point.distance_to(p_point);
		if (dist < min_distance) {
			min_distance = dist;
			min_point = point;
		}
		shapes_found = true;
	}

	if (!shapes_found) {
		return obj->get_transform().origin; //no shapes found, use distance to origin.
	} else {
		return min_point;
	}
}

GodotPhysicsDirectSpaceState3D::GodotPhysicsDirectSpaceState3D() {
	space = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GodotSpace3D::_cull_aabb_for_body(GodotBody3D *p_body, const AABB &p_aabb) {
	int amount = broadphase->cull_aabb(p_aabb, intersection_query_results, INTERSECTION_QUERY_MAX, intersection_query_subindex_results);

	for (int i = 0; i < amount; i++) {
		bool keep = true;

		if (intersection_query_results[i] == p_body) {
			keep = false;
		} else if (intersection_query_results[i]->get_type() == GodotCollisionObject3D::TYPE_AREA) {
			keep = false;
		} else if (intersection_query_results[i]->get_type() == GodotCollisionObject3D::TYPE_SOFT_BODY) {
			keep = false;
		} else if (!p_body->collides_with(static_cast<GodotBody3D *>(intersection_query_results[i]))) {
			keep = false;
		} else if (static_cast<GodotBody3D *>(intersection_query_results[i])->has_exception(p_body->get_self()) || p_body->has_exception(intersection_query_results[i]->get_self())) {
			keep = false;
		}

		if (!keep) {
			if (i < amount - 1) {
				SWAP(intersection_query_results[i], intersection_query_results[amount - 1]);
				SWAP(intersection_query_subindex_results[i], intersection_query_subindex_results[amount - 1]);
			}

			amount--;
			i--;
		}
	}

	return amount;
}

bool GodotSpace3D::test_body_motion(GodotBody3D *p_body, const PS3DT::MotionParameters &p_parameters, PS3DT::MotionResult *r_result) {
	//give me back regular physics engine logic
	//this is madness
	//and most people using this function will think
	//what it does is simpler than using physics
	//this took about a week to get right..
	//but is it right? who knows at this point..

	ERR_FAIL_COND_V(p_parameters.max_collisions < 0 || p_parameters.max_collisions > PS3DT::MotionResult::MAX_COLLISIONS, false);

	if (r_result) {
		*r_result = PS3DT::MotionResult();
	}

	AABB body_aabb;
	bool shapes_found = false;

	for (int i = 0; i < p_body->get_shape_count(); i++) {
		if (p_body->is_shape_disabled(i)) {
			continue;
		}

		if (!shapes_found) {
			body_aabb = p_body->get_shape_aabb(i);
			shapes_found = true;
		} else {
			body_aabb = body_aabb.merge(p_body->get_shape_aabb(i));
		}
	}

	if (!shapes_found) {
		if (r_result) {
			r_result->travel = p_parameters.motion;
		}

		return false;
	}

	real_t margin = MAX(p_parameters.margin, TEST_MOTION_MARGIN_MIN_VALUE);

	// Undo the currently transform the physics server is aware of and apply the provided one
	body_aabb = p_parameters.from.xform(p_body->get_inv_transform().xform(body_aabb));
	body_aabb = body_aabb.grow(margin);

	real_t min_contact_depth = margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;

	real_t motion_length = p_parameters.motion.length();
	Vector3 motion_normal = p_parameters.motion / motion_length;

	Transform3D body_transform = p_parameters.from;

	bool recovered = false;

	{
		//STEP 1, FREE BODY IF STUCK

		const int max_results = 32;
		int recover_attempts = 4;
		Vector3 sr[max_results * 2];
		real_t priorities[max_results];

		do {
			GodotPhysicsServer3D::CollCbkData cbk;
			cbk.max = max_results;
			cbk.amount = 0;
			cbk.ptr = sr;

			GodotPhysicsServer3D::CollCbkData *cbkptr = &cbk;
			GodotCollisionSolver3D::CallbackResult cbkres = GodotPhysicsServer3D::_shape_col_cbk;
			int priority_amount = 0;

			bool collided = false;

			int amount = _cull_aabb_for_body(p_body, body_aabb);

			for (int j = 0; j < p_body->get_shape_count(); j++) {
				if (p_body->is_shape_disabled(j)) {
					continue;
				}

				Transform3D body_shape_xform = body_transform * p_body->get_shape_transform(j);
				GodotShape3D *body_shape = p_body->get_shape(j);

				for (int i = 0; i < amount; i++) {
					const GodotCollisionObject3D *col_obj = intersection_query_results[i];
					if (p_parameters.exclude_bodies.has(col_obj->get_self())) {
						continue;
					}
					if (p_parameters.exclude_objects.has(col_obj->get_instance_id())) {
						continue;
					}

					int shape_idx = intersection_query_subindex_results[i];

					if (GodotCollisionSolver3D::solve_static(body_shape, body_shape_xform, col_obj->get_shape(shape_idx), col_obj->get_transform() * col_obj->get_shape_transform(shape_idx), cbkres, cbkptr, nullptr, margin)) {
						collided = cbk.amount > 0;
					}
					while (cbk.amount > priority_amount) {
						priorities[priority_amount] = col_obj->get_collision_priority();
						priority_amount++;
					}
				}
			}

			if (!collided) {
				break;
			}

			real_t inv_total_weight = 0.0;
			for (int i = 0; i < cbk.amount; i++) {
				inv_total_weight += priorities[i];
			}
			inv_total_weight = Math::is_zero_approx(inv_total_weight) ? 1.0 : (real_t)cbk.amount / inv_total_weight;

			recovered = true;

			Vector3 recover_motion;
			for (int i = 0; i < cbk.amount; i++) {
				Vector3 a = sr[i * 2 + 0];
				Vector3 b = sr[i * 2 + 1];

				// Compute plane on b towards a.
				Vector3 n = (a - b).normalized();
				real_t d = n.dot(b);

				// Compute depth on recovered motion.
				real_t depth = n.dot(a + recover_motion) - d;
				if (depth > min_contact_depth + CMP_EPSILON) {
					// Only recover if there is penetration.
					recover_motion -= n * (depth - min_contact_depth) * 0.4 * priorities[i] * inv_total_weight;
				}
			}

			if (recover_motion == Vector3()) {
				collided = false;
				break;
			}

			body_transform.origin += recover_motion;
			body_aabb.position += recover_motion;

			recover_attempts--;

		} while (recover_attempts);
	}

	real_t safe = 1.0;
	real_t unsafe = 1.0;
	int best_shape = -1;

	{
		// STEP 2 ATTEMPT MOTION

		AABB motion_aabb = body_aabb;
		motion_aabb.position += p_parameters.motion;
		motion_aabb = motion_aabb.merge(body_aabb);

		int amount = _cull_aabb_for_body(p_body, motion_aabb);

		for (int j = 0; j < p_body->get_shape_count(); j++) {
			if (p_body->is_shape_disabled(j)) {
				continue;
			}

			GodotShape3D *body_shape = p_body->get_shape(j);

			// Colliding separation rays allows to properly snap to the ground,
			// otherwise it's not needed in regular motion.
			if (!p_parameters.collide_separation_ray && (body_shape->get_type() == PS3DE::SHAPE_SEPARATION_RAY)) {
				// When slide on slope is on, separation ray shape acts like a regular shape.
				if (!static_cast<GodotSeparationRayShape3D *>(body_shape)->get_slide_on_slope()) {
					continue;
				}
			}

			Transform3D body_shape_xform = body_transform * p_body->get_shape_transform(j);

			Transform3D body_shape_xform_inv = body_shape_xform.affine_inverse();
			GodotMotionShape3D mshape;
			mshape.shape = body_shape;
			mshape.motion = body_shape_xform_inv.basis.xform(p_parameters.motion);

			bool stuck = false;

			real_t best_safe = 1;
			real_t best_unsafe = 1;

			for (int i = 0; i < amount; i++) {
				const GodotCollisionObject3D *col_obj = intersection_query_results[i];
				if (p_parameters.exclude_bodies.has(col_obj->get_self())) {
					continue;
				}
				if (p_parameters.exclude_objects.has(col_obj->get_instance_id())) {
					continue;
				}

				int shape_idx = intersection_query_subindex_results[i];

				//test initial overlap, does it collide if going all the way?
				Vector3 point_A, point_B;
				Vector3 sep_axis = motion_normal;

				Transform3D col_obj_xform = col_obj->get_transform() * col_obj->get_shape_transform(shape_idx);
				//test initial overlap, does it collide if going all the way?
				if (GodotCollisionSolver3D::solve_distance(&mshape, body_shape_xform, col_obj->get_shape(shape_idx), col_obj_xform, point_A, point_B, motion_aabb, &sep_axis)) {
					continue;
				}
				sep_axis = motion_normal;

				if (!GodotCollisionSolver3D::solve_distance(body_shape, body_shape_xform, col_obj->get_shape(shape_idx), col_obj_xform, point_A, point_B, motion_aabb, &sep_axis)) {
					stuck = true;
					break;
				}

				//just do kinematic solving
				real_t low = 0.0;
				real_t hi = 1.0;
				real_t fraction_coeff = 0.5;
				for (int k = 0; k < 8; k++) { //steps should be customizable..
					real_t fraction = low + (hi - low) * fraction_coeff;

					mshape.motion = body_shape_xform_inv.basis.xform(p_parameters.motion * fraction);

					Vector3 lA, lB;
					Vector3 sep = motion_normal; //important optimization for this to work fast enough
					bool collided = !GodotCollisionSolver3D::solve_distance(&mshape, body_shape_xform, col_obj->get_shape(shape_idx), col_obj_xform, lA, lB, motion_aabb, &sep);

					if (collided) {
						hi = fraction;
						if ((k == 0) || (low > 0.0)) { // Did it not collide before?
							// When alternating or first iteration, use dichotomy.
							fraction_coeff = 0.5;
						} else {
							// When colliding again, converge faster towards low fraction
							// for more accurate results with long motions that collide near the start.
							fraction_coeff = 0.25;
						}
					} else {
						point_A = lA;
						point_B = lB;
						low = fraction;
						if ((k == 0) || (hi < 1.0)) { // Did it collide before?
							// When alternating or first iteration, use dichotomy.
							fraction_coeff = 0.5;
						} else {
							// When not colliding again, converge faster towards high fraction
							// for more accurate results with long motions that collide near the end.
							fraction_coeff = 0.75;
						}
					}
				}

				if (low < best_safe) {
					best_safe = low;
					best_unsafe = hi;
				}
			}

			if (stuck) {
				safe = 0;
				unsafe = 0;
				best_shape = j; //sadly it's the best
				break;
			}
			if (best_safe == 1.0) {
				continue;
			}
			if (best_safe < safe) {
				safe = best_safe;
				unsafe = best_unsafe;
				best_shape = j;
			}
		}
	}

	bool collided = false;
	if ((p_parameters.recovery_as_collision && recovered) || (safe < 1)) {
		if (safe >= 1) {
			best_shape = -1; //no best shape with cast, reset to -1
		}

		//it collided, let's get the rest info in unsafe advance
		Transform3D ugt = body_transform;
		ugt.origin += p_parameters.motion * unsafe;

		_RestResultData results[PS3DT::MotionResult::MAX_COLLISIONS];

		_RestCallbackData rcd;
		if (p_parameters.max_collisions > 1) {
			rcd.max_results = p_parameters.max_collisions;
			rcd.other_results = results;
		}

		// Allowed depth can't be lower than motion length, in order to handle contacts at low speed.
		rcd.min_allowed_depth = MIN(motion_length, min_contact_depth);

		body_aabb.position += p_parameters.motion * unsafe;
		int amount = _cull_aabb_for_body(p_body, body_aabb);

		int from_shape = best_shape != -1 ? best_shape : 0;
		int to_shape = best_shape != -1 ? best_shape + 1 : p_body->get_shape_count();

		for (int j = from_shape; j < to_shape; j++) {
			if (p_body->is_shape_disabled(j)) {
				continue;
			}

			Transform3D body_shape_xform = ugt * p_body->get_shape_transform(j);
			GodotShape3D *body_shape = p_body->get_shape(j);

			for (int i = 0; i < amount; i++) {
				const GodotCollisionObject3D *col_obj = intersection_query_results[i];
				if (p_parameters.exclude_bodies.has(col_obj->get_self())) {
					continue;
				}
				if (p_parameters.exclude_objects.has(col_obj->get_instance_id())) {
					continue;
				}

				int shape_idx = intersection_query_subindex_results[i];

				rcd.object = col_obj;
				rcd.shape = shape_idx;
				rcd.local_shape = j;
				bool sc = GodotCollisionSolver3D::solve_static(body_shape, body_shape_xform, col_obj->get_shape(shape_idx), col_obj->get_transform() * col_obj->get_shape_transform(shape_idx), _rest_cbk_result, &rcd, nullptr, margin);
				if (!sc) {
					continue;
				}
			}
		}

		if (rcd.result_count > 0) {
			if (r_result) {
				for (int collision_index = 0; collision_index < rcd.result_count; ++collision_index) {
					const _RestResultData &result = (collision_index > 0) ? rcd.other_results[collision_index - 1] : rcd.best_result;

					PS3DT::MotionCollision &collision = r_result->collisions[collision_index];

					collision.collider = result.object->get_self();
					collision.collider_id = result.object->get_instance_id();
					collision.collider_shape = result.shape;
					collision.local_shape = result.local_shape;
					collision.normal = result.normal;
					collision.position = result.contact;
					collision.depth = result.len;

					const GodotBody3D *body = static_cast<const GodotBody3D *>(result.object);

					Vector3 rel_vec = result.contact - (body->get_transform().origin + body->get_center_of_mass());
					collision.collider_velocity = body->get_linear_velocity() + (body->get_angular_velocity()).cross(rel_vec);
					collision.collider_angular_velocity = body->get_angular_velocity();
				}

				r_result->travel = safe * p_parameters.motion;
				r_result->remainder = p_parameters.motion - safe * p_parameters.motion;
				r_result->travel += (body_transform.get_origin() - p_parameters.from.get_origin());

				r_result->collision_safe_fraction = safe;
				r_result->collision_unsafe_fraction = unsafe;

				r_result->collision_count = rcd.result_count;
				r_result->collision_depth = rcd.best_result.len;
			}

			collided = true;
		}
	}

	if (!collided && r_result) {
		r_result->travel = p_parameters.motion;
		r_result->remainder = Vector3();
		r_result->travel += (body_transform.get_origin() - p_parameters.from.get_origin());

		r_result->collision_safe_fraction = 1.0;
		r_result->collision_unsafe_fraction = 1.0;
		r_result->collision_depth = 0.0;
	}

	return collided;
}

// Assumes a valid collision pair, this should have been checked beforehand in the BVH or octree.
void *GodotSpace3D::_broadphase_pair(GodotCollisionObject3D *A, int p_subindex_A, GodotCollisionObject3D *B, int p_subindex_B, void *p_self) {
	GodotCollisionObject3D::Type type_A = A->get_type();
	GodotCollisionObject3D::Type type_B = B->get_type();
	if (type_A > type_B) {
		SWAP(A, B);
		SWAP(p_subindex_A, p_subindex_B);
		SWAP(type_A, type_B);
	}
	// YGIAH (det-snapshot): canonical orientation — within same-type pairs
	// the lower-ordinal object is always A, so recreated pairs match the
	// captured A/B direction bit-for-bit regardless of broadphase order.
	if (type_A == type_B && A->get_det_ordinal() > B->get_det_ordinal()) {
		SWAP(A, B);
		SWAP(p_subindex_A, p_subindex_B);
	}

	GodotSpace3D *self = static_cast<GodotSpace3D *>(p_self);

	self->collision_pairs++;

	if (type_A == GodotCollisionObject3D::TYPE_AREA) {
		GodotArea3D *area = static_cast<GodotArea3D *>(A);
		if (type_B == GodotCollisionObject3D::TYPE_AREA) {
			GodotArea3D *area_b = static_cast<GodotArea3D *>(B);
			GodotArea2Pair3D *area2_pair = memnew(GodotArea2Pair3D(area_b, p_subindex_B, area, p_subindex_A));
			return area2_pair;
		} else if (type_B == GodotCollisionObject3D::TYPE_SOFT_BODY) {
			GodotSoftBody3D *softbody = static_cast<GodotSoftBody3D *>(B);
			GodotAreaSoftBodyPair3D *soft_area_pair = memnew(GodotAreaSoftBodyPair3D(softbody, p_subindex_B, area, p_subindex_A));
			return soft_area_pair;
		} else {
			GodotBody3D *body = static_cast<GodotBody3D *>(B);
			GodotAreaPair3D *area_pair = memnew(GodotAreaPair3D(body, p_subindex_B, area, p_subindex_A));
			return area_pair;
		}
	} else if (type_A == GodotCollisionObject3D::TYPE_BODY) {
		if (type_B == GodotCollisionObject3D::TYPE_SOFT_BODY) {
			GodotBodySoftBodyPair3D *soft_pair = memnew(GodotBodySoftBodyPair3D(static_cast<GodotBody3D *>(A), p_subindex_A, static_cast<GodotSoftBody3D *>(B)));
			return soft_pair;
		} else {
			GodotBodyPair3D *b = memnew(GodotBodyPair3D(static_cast<GodotBody3D *>(A), p_subindex_A, static_cast<GodotBody3D *>(B), p_subindex_B));
			// YGIAH (det-snapshot): if this pair existed at capture time, its
			// warm-start accumulators were staged by _det_import_state().
			if (!self->pending_pair_state.is_empty()) {
				PendingPairKey k;
				k.ord_a = A->get_det_ordinal();
				k.sub_a = p_subindex_A;
				k.ord_b = B->get_det_ordinal();
				k.sub_b = p_subindex_B;
				Dictionary *st = self->pending_pair_state.getptr(k);
				if (st) {
					Error imp_err = b->_det_import_state(*st);
					if (imp_err != OK) {
						WARN_PRINT("det-snapshot: pair create-time import failed ord_a=" + itos(k.ord_a) + " ord_b=" + itos(k.ord_b));
					}
					self->pending_pair_state.erase(k);
				}
			}
			return b;
		}
	} else {
		// Soft Body/Soft Body, not supported.
	}

	return nullptr;
}

void GodotSpace3D::_broadphase_unpair(GodotCollisionObject3D *A, int p_subindex_A, GodotCollisionObject3D *B, int p_subindex_B, void *p_data, void *p_self) {
	if (!p_data) {
		return;
	}

	GodotSpace3D *self = static_cast<GodotSpace3D *>(p_self);
	self->collision_pairs--;
	GodotConstraint3D *c = static_cast<GodotConstraint3D *>(p_data);
	memdelete(c);
}

const SelfList<GodotBody3D>::List &GodotSpace3D::get_active_body_list() const {
	return active_list;
}

void GodotSpace3D::body_add_to_active_list(SelfList<GodotBody3D> *p_body) {
	active_list.add(p_body);
}

void GodotSpace3D::body_remove_from_active_list(SelfList<GodotBody3D> *p_body) {
	active_list.remove(p_body);
}

void GodotSpace3D::body_add_to_mass_properties_update_list(SelfList<GodotBody3D> *p_body) {
	mass_properties_update_list.add(p_body);
}

void GodotSpace3D::body_remove_from_mass_properties_update_list(SelfList<GodotBody3D> *p_body) {
	mass_properties_update_list.remove(p_body);
}

GodotBroadPhase3D *GodotSpace3D::get_broadphase() {
	return broadphase;
}

void GodotSpace3D::add_object(GodotCollisionObject3D *p_object) {
	ERR_FAIL_COND(objects.has(p_object));
	objects.insert(p_object);
	// YGIAH (det-snapshot): arrival order within the space. _det_import_state
	// may overwrite it with the captured ordinal afterwards.
	p_object->set_det_ordinal(_det_next_ordinal());
}

void GodotSpace3D::remove_object(GodotCollisionObject3D *p_object) {
	ERR_FAIL_COND(!objects.has(p_object));
	objects.erase(p_object);
}

const HashSet<GodotCollisionObject3D *> &GodotSpace3D::get_objects() const {
	return objects;
}

void GodotSpace3D::body_add_to_state_query_list(SelfList<GodotBody3D> *p_body) {
	state_query_list.add(p_body);
}

void GodotSpace3D::body_remove_from_state_query_list(SelfList<GodotBody3D> *p_body) {
	state_query_list.remove(p_body);
}

void GodotSpace3D::area_add_to_monitor_query_list(SelfList<GodotArea3D> *p_area) {
	monitor_query_list.add(p_area);
}

void GodotSpace3D::area_remove_from_monitor_query_list(SelfList<GodotArea3D> *p_area) {
	monitor_query_list.remove(p_area);
}

void GodotSpace3D::area_add_to_moved_list(SelfList<GodotArea3D> *p_area) {
	area_moved_list.add(p_area);
}

void GodotSpace3D::area_remove_from_moved_list(SelfList<GodotArea3D> *p_area) {
	area_moved_list.remove(p_area);
}

const SelfList<GodotArea3D>::List &GodotSpace3D::get_moved_area_list() const {
	return area_moved_list;
}

const SelfList<GodotSoftBody3D>::List &GodotSpace3D::get_active_soft_body_list() const {
	return active_soft_body_list;
}

void GodotSpace3D::soft_body_add_to_active_list(SelfList<GodotSoftBody3D> *p_soft_body) {
	active_soft_body_list.add(p_soft_body);
}

void GodotSpace3D::soft_body_remove_from_active_list(SelfList<GodotSoftBody3D> *p_soft_body) {
	active_soft_body_list.remove(p_soft_body);
}

void GodotSpace3D::call_queries() {
	while (state_query_list.first()) {
		GodotBody3D *b = state_query_list.first()->self();
		state_query_list.remove(state_query_list.first());
		b->call_queries();
	}

	while (monitor_query_list.first()) {
		GodotArea3D *a = monitor_query_list.first()->self();
		monitor_query_list.remove(monitor_query_list.first());
		a->call_queries();
	}
}

void GodotSpace3D::setup() {
	contact_debug_count = 0;
	while (mass_properties_update_list.first()) {
		mass_properties_update_list.first()->self()->update_mass_properties();
		mass_properties_update_list.remove(mass_properties_update_list.first());
	}
}

void GodotSpace3D::update() {
	broadphase->update();
}

void GodotSpace3D::set_param(PS3DE::SpaceParameter p_param, real_t p_value) {
	switch (p_param) {
		case PS3DE::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
			contact_recycle_radius = p_value;
			break;
		case PS3DE::SPACE_PARAM_CONTACT_MAX_SEPARATION:
			contact_max_separation = p_value;
			break;
		case PS3DE::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
			contact_max_allowed_penetration = p_value;
			break;
		case PS3DE::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
			contact_bias = p_value;
			break;
		case PS3DE::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			body_linear_velocity_sleep_threshold = p_value;
			break;
		case PS3DE::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			body_angular_velocity_sleep_threshold = p_value;
			break;
		case PS3DE::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			body_time_to_sleep = p_value;
			break;
		case PS3DE::SPACE_PARAM_SOLVER_ITERATIONS:
			solver_iterations = p_value;
			break;
	}
}

real_t GodotSpace3D::get_param(PS3DE::SpaceParameter p_param) const {
	switch (p_param) {
		case PS3DE::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
			return contact_recycle_radius;
		case PS3DE::SPACE_PARAM_CONTACT_MAX_SEPARATION:
			return contact_max_separation;
		case PS3DE::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
			return contact_max_allowed_penetration;
		case PS3DE::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
			return contact_bias;
		case PS3DE::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			return body_linear_velocity_sleep_threshold;
		case PS3DE::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			return body_angular_velocity_sleep_threshold;
		case PS3DE::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			return body_time_to_sleep;
		case PS3DE::SPACE_PARAM_SOLVER_ITERATIONS:
			return solver_iterations;
	}
	return 0;
}

void GodotSpace3D::lock() {
	locked = true;
}

void GodotSpace3D::unlock() {
	locked = false;
}

bool GodotSpace3D::is_locked() const {
	return locked;
}

GodotPhysicsDirectSpaceState3D *GodotSpace3D::get_direct_state() {
	return direct_access;
}

GodotSpace3D::GodotSpace3D() {
	body_linear_velocity_sleep_threshold = GLOBAL_GET("physics/3d/sleep_threshold_linear");
	body_angular_velocity_sleep_threshold = GLOBAL_GET("physics/3d/sleep_threshold_angular");
	body_time_to_sleep = GLOBAL_GET("physics/3d/time_before_sleep");
	solver_iterations = GLOBAL_GET("physics/3d/solver/solver_iterations");
	contact_recycle_radius = GLOBAL_GET("physics/3d/solver/contact_recycle_radius");
	contact_max_separation = GLOBAL_GET("physics/3d/solver/contact_max_separation");
	contact_max_allowed_penetration = GLOBAL_GET("physics/3d/solver/contact_max_allowed_penetration");
	contact_bias = GLOBAL_GET("physics/3d/solver/default_contact_bias");

	broadphase = GodotBroadPhase3D::create_func();
	broadphase->set_pair_callback(_broadphase_pair, this);
	broadphase->set_unpair_callback(_broadphase_unpair, this);

	direct_access = memnew(GodotPhysicsDirectSpaceState3D);
	direct_access->space = this;
}

GodotSpace3D::~GodotSpace3D() {
	memdelete(broadphase);
	memdelete(direct_access);
}

/* --------- YGIAH (det-snapshot) --------- */

Dictionary GodotSpace3D::_det_export_state() const {
	Dictionary d;
	d["object_ordinal_counter"] = (int64_t)object_ordinal_counter;
	d["solver_iterations"] = solver_iterations;
	d["contact_recycle_radius"] = contact_recycle_radius;
	d["contact_max_separation"] = contact_max_separation;
	d["contact_max_allowed_penetration"] = contact_max_allowed_penetration;
	d["contact_bias"] = contact_bias;
	d["body_linear_velocity_sleep_threshold"] = body_linear_velocity_sleep_threshold;
	d["body_angular_velocity_sleep_threshold"] = body_angular_velocity_sleep_threshold;
	d["body_time_to_sleep"] = body_time_to_sleep;
	d["last_step"] = last_step;
	d["island_count"] = island_count;
	d["active_objects"] = active_objects;
	d["collision_pairs"] = collision_pairs;

	// Objects in canonical ordinal order.
	LocalVector<GodotCollisionObject3D *> sorted_objects;
	for (GodotCollisionObject3D *obj : objects) {
		sorted_objects.push_back(obj);
	}
	struct _DetOrdLess {
		_FORCE_INLINE_ bool operator()(const GodotCollisionObject3D *a, const GodotCollisionObject3D *b) const {
			return a->get_det_ordinal() < b->get_det_ordinal();
		}
	};
	sorted_objects.sort_custom<_DetOrdLess>();

	Array bodies;
	HashMap<GodotCollisionObject3D *, int> obj_index;
	for (uint32_t i = 0; i < sorted_objects.size(); i++) {
		obj_index[sorted_objects[i]] = (int)i;
	}
	for (GodotCollisionObject3D *obj : sorted_objects) {
		Dictionary od;
		od["type"] = (int)obj->get_type();
		od["ord"] = (int64_t)obj->get_det_ordinal();
		od["instance_id"] = (int64_t)(uint64_t)obj->get_instance_id();
		{
			// Broadphase AABBs per shape — the grow margin in aabb_cache is
			// accumulated state and must be imported verbatim, or pair
			// formation after restore diverges from the baseline.
			Array aabbs;
			for (int i = 0; i < obj->get_shape_count(); i++) {
				aabbs.push_back(obj->get_shape_aabb(i));
			}
			od["aabbs"] = aabbs;
			Vector<AABB> pair_aabbs;
			obj->det_export_shape_pair_aabbs(pair_aabbs);
			Array pa;
			for (const AABB &a : pair_aabbs) {
				pa.push_back(a);
			}
			od["pair_aabbs"] = pa;
		}
		{
			// Rebuilt objects carry fresh ObjectIDs; match them by the
			// stable scene path of their owner node instead.
			Object *owner = ObjectDB::get_instance(obj->get_instance_id());
			Node *owner_node = Object::cast_to<Node>(owner);
			if (owner_node && owner_node->is_inside_tree()) {
				od["path"] = String(owner_node->get_path());
			} else {
				od["owner"] = owner ? (owner->get_class() + (Object::cast_to<Node>(owner) ? "!intree" : "!node")) : String("null");
			}
		}
		switch (obj->get_type()) {
			case GodotCollisionObject3D::TYPE_BODY: {
				GodotBody3D *bd = static_cast<GodotBody3D *>(obj);
				od["body"] = bd->_det_export_state();
			} break;
			default:
				// Areas and soft bodies: scalar data only (documented gap).
				od["unsupported"] = true;
				WARN_PRINT_ONCE("det-snapshot: non-body physics object state is not serialized (area/soft body).");
				break;
		}
		bodies.push_back(od);
	}
	d["objects"] = bodies;

	// Contact pairs: dedupe over per-body constraint maps.
	Array pairs;
	HashSet<GodotConstraint3D *> seen;
	for (GodotCollisionObject3D *obj : sorted_objects) {
		if (obj->get_type() != GodotCollisionObject3D::TYPE_BODY) {
			continue;
		}
		GodotBody3D *body = static_cast<GodotBody3D *>(obj);
		for (const KeyValue<GodotConstraint3D *, int> &E : body->get_constraint_map()) {
			GodotConstraint3D *c = E.key;
			if (seen.has(c) || c->_det_kind() != 1) {
				continue;
			}
			seen.insert(c);
			pairs.push_back(static_cast<GodotBodyPair3D *>(c)->_det_export_state());
		}
	}
	d["pairs"] = pairs;

	// Joints: dedupe over per-body constraint maps, then emit in canonical
	// key order so same-key multi-joints map to a stable occurrence index.
	// A rebuilt joint recomputes its anchor frames/pivots against
	// then-current body transforms, so solver-internal joint state must
	// round-trip verbatim.
	{
		Vector<GodotConstraint3D *> jc;
		for (GodotCollisionObject3D *obj : sorted_objects) {
			if (obj->get_type() != GodotCollisionObject3D::TYPE_BODY) {
				continue;
			}
			GodotBody3D *body = static_cast<GodotBody3D *>(obj);
			for (const KeyValue<GodotConstraint3D *, int> &E : body->get_constraint_map()) {
				GodotConstraint3D *c = E.key;
				if (seen.has(c) || c->_det_kind() != 2) {
					continue;
				}
				seen.insert(c);
				jc.push_back(c);
			}
		}
		struct _DetJLess {
			_FORCE_INLINE_ bool operator()(GodotConstraint3D *a, GodotConstraint3D *b) const {
				uint64_t ka[5], kb[5];
				a->_det_get_key(ka);
				b->_det_get_key(kb);
				for (int i = 0; i < 5; i++) {
					if (ka[i] != kb[i]) {
						return ka[i] < kb[i];
					}
				}
				return a < b;
			}
		};
		jc.sort_custom<_DetJLess>();
		Array joints;
		for (GodotConstraint3D *c : jc) {
			Dictionary st = c->_det_export_state();
			if (st.is_empty()) {
				WARN_PRINT_ONCE("det-snapshot: joint has no state serializer (type unknown); skipping.");
				continue;
			}
			uint64_t k[5];
			c->_det_get_key(k);
			Dictionary jd;
			jd["ord_a"] = (int64_t)k[0];
			jd["sub_a"] = (int64_t)k[1];
			jd["ord_b"] = (int64_t)k[2];
			jd["sub_b"] = (int64_t)k[3];
			jd["jtype"] = (int)(static_cast<GodotJoint3D *>(c)->get_type());
			jd["st"] = st;
			joints.push_back(jd);
		}
		d["joints"] = joints;
	}

	// YGIAH: the solver iterates active_list for island generation, and the
	// list's wake order is historical (prepend on activation), not ordinal.
	// Export it as indices into "objects" so import can rebuild the exact
	// iteration order — solver float accumulation depends on it.
	Array active_order;
	for (const SelfList<GodotBody3D> *E = active_list.first(); E; E = E->next()) {
		int *pi = obj_index.getptr(E->self());
		if (pi) {
			active_order.push_back(*pi);
		}
	}
	d["active_order"] = active_order;
	return d;
}

Error GodotSpace3D::_det_import_state(const Dictionary &d) {
	object_ordinal_counter = (uint64_t)(int64_t)d["object_ordinal_counter"];
	solver_iterations = d["solver_iterations"];
	contact_recycle_radius = d["contact_recycle_radius"];
	contact_max_separation = d["contact_max_separation"];
	contact_max_allowed_penetration = d["contact_max_allowed_penetration"];
	contact_bias = d["contact_bias"];
	body_linear_velocity_sleep_threshold = d["body_linear_velocity_sleep_threshold"];
	body_angular_velocity_sleep_threshold = d["body_angular_velocity_sleep_threshold"];
	body_time_to_sleep = d["body_time_to_sleep"];
	last_step = d["last_step"];
	island_count = d["island_count"];
	active_objects = d["active_objects"];
	collision_pairs = d["collision_pairs"];

	// Bodies first: their fields, then stage pair state for re-creation.
	Array objs = d["objects"];
	HashMap<uint64_t, GodotCollisionObject3D *> by_ord;
	HashMap<String, GodotCollisionObject3D *> by_path;
	for (GodotCollisionObject3D *obj : objects) {
		by_ord[obj->get_det_ordinal()] = obj;
		Object *owner = ObjectDB::get_instance(obj->get_instance_id());
		Node *owner_node = Object::cast_to<Node>(owner);
		if (owner_node && owner_node->is_inside_tree()) {
			by_path[String(owner_node->get_path())] = obj;
		}
	}
	LocalVector<GodotCollisionObject3D *> matched;
	matched.resize(objs.size());
	for (int i = 0; i < objs.size(); i++) {
		Dictionary od = objs[i];
		uint64_t ord = (uint64_t)(int64_t)od["ord"];
		// Rebuilt objects have fresh ObjectIDs, so match by the stable scene
		// path first and fall back to the deterministic ordinal.
		GodotCollisionObject3D *obj = nullptr;
		if ((int64_t)od["instance_id"] != 0) {
			// Node-owned object: the scene path is the only reliable key —
			// ordinals may legitimately disagree after a rebuild, and matching
			// by ordinal would silently apply state to the wrong body.
			if (od.has("path")) {
				GodotCollisionObject3D **pf = by_path.getptr(String(od["path"]));
				if (pf) {
					obj = *pf;
				}
			}
		} else {
			// Engine-internal object (default area etc.): deterministic
			// creation order makes the ordinal authoritative.
			GodotCollisionObject3D **found = by_ord.getptr(ord);
			if (found) {
				obj = *found;
			}
		}
		if (!obj) {
			WARN_PRINT("det-snapshot: physics object ordinal " + itos(ord) + " type=" + itos((int)od["type"]) + " path=" + String(od.get("path", "?")) + " iid=" + itos((int64_t)od["instance_id"]) + " has no restored counterpart (live=" + itos(objects.size()) + " bypath=" + itos(by_path.size()) + ").");
			continue;
		}
		if (!od.get("unsupported", false) && obj->get_type() == GodotCollisionObject3D::TYPE_BODY && od.has("body")) {
			static_cast<GodotBody3D *>(obj)->_det_import_state(od["body"]);
		} else {
			obj->set_det_ordinal(ord);
		}
		matched[i] = obj;
	}

	// YGIAH: import setters (collision layer/mask/priority) run
	// _shapes_changed() -> wakeup()/wakeup_neighbours(), which can re-activate
	// bodies whose own _det_import_state already applied active=false (joint
	// neighbours imported later). Enforce the captured active flag in a second
	// pass so active_list membership matches the snapshot exactly.
	for (int i = 0; i < objs.size(); i++) {
		Dictionary od = objs[i];
		GodotCollisionObject3D *o = matched[i];
		if (!o || o->get_type() != GodotCollisionObject3D::TYPE_BODY || !od.has("body")) {
			continue;
		}
		GodotBody3D *body = static_cast<GodotBody3D *>(o);
		Dictionary bd = od["body"];
		bool want_active = bd.get("active", false);
		if (body->is_active() != want_active) {
			body->set_active(want_active);
		}
	}

	// YGIAH: rebuild the exported active_list order. set_active() prepends,
	// so toggling active bodies in REVERSED export order yields the captured
	// list order — the solver's island generation/solve order depends on it.
	Array active_order = d.get("active_order", Array());
	if (!active_order.is_empty()) {
		LocalVector<GodotBody3D *> reorder;
		for (int i = active_order.size() - 1; i >= 0; i--) {
			int idx = (int)active_order[i];
			if (idx < 0 || idx >= objs.size()) {
				continue;
			}
			GodotCollisionObject3D *o = matched[idx];
			if (o && o->get_type() == GodotCollisionObject3D::TYPE_BODY) {
				reorder.push_back(static_cast<GodotBody3D *>(o));
			}
		}
		for (GodotBody3D *body : reorder) {
			if (!body->is_active()) {
				continue;
			}
			body->set_active(false);
			body->set_active(true);
		}
	}

	// YGIAH: pairs formed during the scene rebuild bound their A/B roles by
	// ARRIVAL ordinal, which can disagree with the captured ordinal order —
	// GodotBodyPair3D::_det_import_state() then fails its orientation assert
	// and leaves factory-zero warm-start state. Unpair everything so every
	// pair is re-created through _broadphase_pair() at the next update():
	// ordinals are captured by then, so orientation is canonical and the
	// staged state below is consumed at creation.
	{
		LocalVector<GodotBroadPhase3D::DetPairEntry> live_pairs;
		broadphase->det_for_each_pair(live_pairs);
		for (const GodotBroadPhase3D::DetPairEntry &e : live_pairs) {
			GodotConstraint3D *c = static_cast<GodotConstraint3D *>(e.userdata);
			if (c && c->_det_kind() == 1) {
				broadphase->det_unpair(e.id_a, e.id_b);
			}
		}
	}

	// Stage pair warm-start state; the broadphase applies it as pairs are
	// re-created by the shape flush below and on the next update().
	pending_pair_state.clear();
	Array pairs = d["pairs"];
	for (int i = 0; i < pairs.size(); i++) {
		Dictionary pd = pairs[i];
		PendingPairKey k;
		k.ord_a = (uint64_t)(int64_t)pd["ord_a"];
		k.sub_a = pd["sub_a"];
		k.ord_b = (uint64_t)(int64_t)pd["ord_b"];
		k.sub_b = pd["sub_b"];
		pending_pair_state[k] = pd;
	}

	// YGIAH: the scene rebuild queued every object into the server's
	// pending_shape_update_list. Flushing those entries now recomputes the
	// broadphase AABBs from the imported transforms — re-creating pairs with
	// the staged warm-start state — and prevents the deferred _shape_changed()
	// at the next PhysicsServer3D::step() from running _shapes_changed() ->
	// wakeup()/wakeup_neighbours(), which would re-activate bodies captured
	// asleep and diverge the first post-restore step.
	for (uint32_t i = 0; i < matched.size(); i++) {
		if (matched[i]) {
			Dictionary od = objs[i];
			Array aa = od.get("aabbs", Array());
			Vector<AABB> aabbs;
			for (const Variant &v : aa) {
				aabbs.push_back(v);
			}
			Array pa = od.get("pair_aabbs", Array());
			Vector<AABB> pair_aabbs;
			for (const Variant &v : pa) {
				pair_aabbs.push_back(v);
			}
			matched[i]->det_flush_shape_update(aabbs, pair_aabbs);
		}
	}

	// YGIAH: pairs created during the scene REBUILD predated the
	// pending_pair_state staging above, so _broadphase_pair() never offered
	// them their staged warm-start state. Backfill: apply staged state to any
	// existing body pair that still has one pending. Pairs that survive from
	// rebuild AABBs are the common case; pairs re-created by the flush above
	// already consumed (and erased) their entry at creation.
	for (GodotCollisionObject3D *obj : objects) {
		if (obj->get_type() != GodotCollisionObject3D::TYPE_BODY) {
			continue;
		}
		for (const KeyValue<GodotConstraint3D *, int> &E : static_cast<GodotBody3D *>(obj)->get_constraint_map()) {
			if (E.key->_det_kind() != 1) {
				continue;
			}
			uint64_t k[5];
			E.key->_det_get_key(k);
			PendingPairKey pk;
			pk.ord_a = k[0];
			pk.sub_a = (int)k[1];
			pk.ord_b = k[2];
			pk.sub_b = (int)k[3];
			Dictionary *st = pending_pair_state.getptr(pk);
			if (st) {
				Error imp_err = static_cast<GodotBodyPair3D *>(E.key)->_det_import_state(*st);
				if (imp_err != OK) {
					GodotBodyPair3D *bp = static_cast<GodotBodyPair3D *>(E.key);
					WARN_PRINT("det-snapshot: pair backfill failed ord_a=" + itos(pk.ord_a) + " ord_b=" + itos(pk.ord_b) +
							" actual_a=" + itos(bp->get_body_ptr()[0]->get_det_ordinal()) +
							" actual_b=" + itos(bp->get_body_ptr()[1]->get_det_ordinal()));
				}
				pending_pair_state.erase(pk);
			}
		}
	}

	// YGIAH: joints already exist by now (configured during the scene
	// rebuild / still live in-place). Their solver-internal anchors were
	// recomputed against current body transforms — overwrite them with the
	// captured joint state. Both sides sort by the same det key, so the
	// i-th live joint of a given key pairs with the i-th captured entry.
	{
		Array joints = d.get("joints", Array());
		if (!joints.is_empty()) {
			HashMap<String, Vector<Dictionary>> captured;
			for (int i = 0; i < joints.size(); i++) {
				Dictionary jd = joints[i];
				String k = itos((int64_t)jd["ord_a"]) + ":" + itos((int64_t)jd["sub_a"]) + "-" +
						itos((int64_t)jd["ord_b"]) + ":" + itos((int64_t)jd["sub_b"]);
				captured[k].push_back(jd);
			}
			Vector<GodotConstraint3D *> live;
			HashSet<GodotConstraint3D *> seen_j;
			for (GodotCollisionObject3D *obj : objects) {
				if (obj->get_type() != GodotCollisionObject3D::TYPE_BODY) {
					continue;
				}
				for (const KeyValue<GodotConstraint3D *, int> &E : static_cast<GodotBody3D *>(obj)->get_constraint_map()) {
					if (!seen_j.has(E.key) && E.key->_det_kind() == 2) {
						seen_j.insert(E.key);
						live.push_back(E.key);
					}
				}
			}
			struct _DetJLess2 {
				_FORCE_INLINE_ bool operator()(GodotConstraint3D *a, GodotConstraint3D *b) const {
					uint64_t ka[5], kb[5];
					a->_det_get_key(ka);
					b->_det_get_key(kb);
					for (int i = 0; i < 5; i++) {
						if (ka[i] != kb[i]) {
							return ka[i] < kb[i];
						}
					}
					return a < b;
				}
			};
			live.sort_custom<_DetJLess2>();
			HashMap<String, int> used;
			for (GodotConstraint3D *c : live) {
				uint64_t k[5];
				c->_det_get_key(k);
				String ks = itos(k[0]) + ":" + itos(k[1]) + "-" + itos(k[2]) + ":" + itos(k[3]);
				Vector<Dictionary> *list = captured.getptr(ks);
				if (!list) {
					continue;
				}
				int idx = used.has(ks) ? used[ks] : 0;
				used[ks] = idx + 1;
				if (idx < (int)list->size()) {
					Error imp_err = c->_det_import_state((*list)[idx]["st"]);
					if (imp_err != OK) {
						WARN_PRINT("det-snapshot: joint state import failed key=" + ks);
					}
				}
			}
		}
	}
	return OK;
}
