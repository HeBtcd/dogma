/**************************************************************************/
/*  tween.cpp                                                             */
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

#include "tween.h"

#include "core/object/class_db.h"
#include "scene/animation/easing_equations.h"
#include "scene/main/node.h"
#include "scene/main/scene_tree.h"
#include "scene/resources/animation.h"

#define CHECK_VALID() \
	ERR_FAIL_COND_V_MSG(!valid, nullptr, "Tween invalid. Either finished or created outside scene tree."); \
	ERR_FAIL_COND_V_MSG(started, nullptr, "Can't append to a Tween that has started. Use stop() first.");

Tween::interpolater Tween::interpolaters[Tween::TRANS_MAX][Tween::EASE_MAX] = {
	{ &Linear::in, &Linear::in, &Linear::in, &Linear::in }, // Linear is the same for each easing.
	{ &Sine::in, &Sine::out, &Sine::in_out, &Sine::out_in },
	{ &Quint::in, &Quint::out, &Quint::in_out, &Quint::out_in },
	{ &Quart::in, &Quart::out, &Quart::in_out, &Quart::out_in },
	{ &Quad::in, &Quad::out, &Quad::in_out, &Quad::out_in },
	{ &Expo::in, &Expo::out, &Expo::in_out, &Expo::out_in },
	{ &Elastic::in, &Elastic::out, &Elastic::in_out, &Elastic::out_in },
	{ &Cubic::in, &Cubic::out, &Cubic::in_out, &Cubic::out_in },
	{ &Circ::in, &Circ::out, &Circ::in_out, &Circ::out_in },
	{ &Bounce::in, &Bounce::out, &Bounce::in_out, &Bounce::out_in },
	{ &Back::in, &Back::out, &Back::in_out, &Back::out_in },
	{ &Spring::in, &Spring::out, &Spring::in_out, &Spring::out_in },
};

void Tweener::set_tween(const Ref<Tween> &p_tween) {
	tween_id = p_tween->get_instance_id();
}

void Tweener::start() {
	elapsed_time = 0;
	finished = false;
}

Ref<Tween> Tweener::_get_tween() {
	return ObjectDB::get_ref<Tween>(tween_id);
}

void Tweener::_finish() {
	finished = true;
	emit_signal(SceneStringName(finished));
}

void Tweener::_bind_methods() {
	ADD_SIGNAL(MethodInfo("finished"));
}

void Tween::_start_tweeners() {
	if (tweeners.is_empty()) {
		dead = true;
		ERR_FAIL_MSG("Tween without commands, aborting.");
	}

	for (Ref<Tweener> &tweener : tweeners[current_step]) {
		tweener->start();
	}
}

void Tween::_stop_internal(bool p_reset) {
	running = false;
	if (p_reset) {
		started = false;
		dead = false;
		total_time = 0;
	}
}

RequiredResult<PropertyTweener> Tween::tween_property(RequiredParam<const Object> p_target, const NodePath &p_property, Variant p_to, double p_duration) {
	EXTRACT_PARAM_OR_FAIL_V(target, p_target, nullptr);
	CHECK_VALID();

	Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The tweened property \"%s\" does not exist in object \"%s\".", p_property, target));
#else
	const Variant &prop_value = target->get_indexed(property_subnames);
#endif

	if (!Animation::validate_type_match(prop_value, p_to)) {
		return nullptr;
	}

	Ref<PropertyTweener> tweener;
	tweener.instantiate(target, property_subnames, p_to, p_duration);
	append(tweener);
	return tweener;
}

RequiredResult<IntervalTweener> Tween::tween_interval(double p_time) {
	CHECK_VALID();

	Ref<IntervalTweener> tweener;
	tweener.instantiate(p_time);
	append(tweener);
	return tweener;
}

RequiredResult<CallbackTweener> Tween::tween_callback(const Callable &p_callback) {
	CHECK_VALID();

	Ref<CallbackTweener> tweener;
	tweener.instantiate(p_callback);
	append(tweener);
	return tweener;
}

RequiredResult<MethodTweener> Tween::tween_method(const Callable &p_callback, const Variant p_from, Variant p_to, double p_duration) {
	CHECK_VALID();

	if (!Animation::validate_type_match(p_from, p_to)) {
		return nullptr;
	}

	Ref<MethodTweener> tweener;
	tweener.instantiate(p_callback, p_from, p_to, p_duration);
	append(tweener);
	return tweener;
}

RequiredResult<SubtweenTweener> Tween::tween_subtween(RequiredParam<Tween> p_subtween) {
	CHECK_VALID();

	// Ensure that the subtween being added is not null.
	EXTRACT_PARAM_OR_FAIL_V(subtween, p_subtween, nullptr);

	Ref<SubtweenTweener> tweener;
	tweener.instantiate(subtween);

	// Remove the tween from its parent tree, if it has one.
	// If the user created this tween without a parent tree attached,
	// then this step isn't necessary.
	if (tweener->subtween->parent_tree != nullptr) {
		tweener->subtween->parent_tree->remove_tween(tweener->subtween);
	}
	subtweens.push_back(subtween);
	append(tweener);
	return tweener;
}

RequiredResult<AwaitTweener> Tween::tween_await(const Signal &p_signal) {
	CHECK_VALID();

	Ref<AwaitTweener> tweener = memnew(AwaitTweener(p_signal));
	append(tweener);
	return tweener;
}

void Tween::append(Ref<Tweener> p_tweener) {
	p_tweener->set_tween(this);

	if (parallel_enabled) {
		current_step = MAX(current_step, 0);
	} else {
		current_step++;
	}
	parallel_enabled = default_parallel;

	tweeners.resize(current_step + 1);
	tweeners[current_step].push_back(p_tweener);
}

void Tween::stop() {
	_stop_internal(true);
}

void Tween::pause() {
	_stop_internal(false);
}

void Tween::play() {
	ERR_FAIL_COND_MSG(!valid, "Tween invalid. Either finished or created outside scene tree.");
	ERR_FAIL_COND_MSG(dead, "Can't play finished Tween, use stop() first to reset its state.");
	running = true;
}

void Tween::kill() {
	running = false; // For the sake of is_running().
	valid = false;
	dead = true;

	// Kill all subtweens of this tween.
	for (Ref<Tween> &st : subtweens) {
		st->kill();
	}
}

bool Tween::has_tweeners() const {
	return !tweeners.is_empty();
}

bool Tween::is_running() {
	return running;
}

bool Tween::is_valid() {
	return valid;
}

void Tween::clear() {
	valid = false;
	tweeners.clear();
}

RequiredResult<Tween> Tween::bind_node(RequiredParam<const Node> p_node) {
	EXTRACT_PARAM_OR_FAIL_V(node, p_node, this);

	bound_node = node->get_instance_id();
	is_bound = true;
	return this;
}

RequiredResult<Tween> Tween::set_process_mode(TweenProcessMode p_mode) {
	process_mode = p_mode;
	return this;
}

Tween::TweenProcessMode Tween::get_process_mode() const {
	return process_mode;
}

RequiredResult<Tween> Tween::set_pause_mode(TweenPauseMode p_mode) {
	pause_mode = p_mode;
	return this;
}

Tween::TweenPauseMode Tween::get_pause_mode() const {
	return pause_mode;
}

RequiredResult<Tween> Tween::set_ignore_time_scale(bool p_ignore) {
	ignore_time_scale = p_ignore;
	return this;
}

bool Tween::is_ignoring_time_scale() const {
	return ignore_time_scale;
}

RequiredResult<Tween> Tween::set_parallel(bool p_parallel) {
	default_parallel = p_parallel;
	parallel_enabled = p_parallel;
	return this;
}

RequiredResult<Tween> Tween::set_loops(int p_loops) {
	loops = p_loops;
	return this;
}

int Tween::get_loops_left() const {
	if (loops <= 0) {
		return -1; // Infinite loop.
	} else {
		return loops - loops_done;
	}
}

RequiredResult<Tween> Tween::set_speed_scale(float p_speed) {
	speed_scale = p_speed;
	return this;
}

RequiredResult<Tween> Tween::set_trans(TransitionType p_trans) {
	default_transition = p_trans;
	return this;
}

Tween::TransitionType Tween::get_trans() const {
	return default_transition;
}

RequiredResult<Tween> Tween::set_ease(EaseType p_ease) {
	default_ease = p_ease;
	return this;
}

Tween::EaseType Tween::get_ease() const {
	return default_ease;
}

RequiredResult<Tween> Tween::parallel() {
	parallel_enabled = true;
	return this;
}

RequiredResult<Tween> Tween::chain() {
	parallel_enabled = false;
	return this;
}

bool Tween::custom_step(double p_delta) {
	ERR_FAIL_COND_V_MSG(in_step, true, "Can't call custom_step() during another Tween step.");

	bool r = running;
	running = true;
	bool ret = step(p_delta);
	running = running && r; // Running might turn false when Tween finished.
	return ret;
}

bool Tween::step(double p_delta) {
	if (dead) {
		return false;
	}

	if (is_bound) {
		Node *node = get_bound_node();
		if (node) {
			if (!node->is_inside_tree()) {
				return true;
			}
		} else {
			return false;
		}
	}

	if (!running) {
		return true;
	}
	in_step = true;

	if (!started) {
		if (tweeners.is_empty()) {
			String tween_id;
			Node *node = get_bound_node();
			if (node) {
				tween_id = vformat("Tween (bound to %s)", node->is_inside_tree() ? (String)node->get_path() : (String)node->get_name());
			} else {
				tween_id = to_string();
			}
			in_step = false;
			ERR_FAIL_V_MSG(false, tween_id + ": started with no Tweeners.");
		}
		current_step = 0;
		loops_done = 0;
		total_time = 0;
		_start_tweeners();
		started = true;
	}

	double rem_delta = p_delta * speed_scale;
	bool step_active = false;
	total_time += rem_delta;

#ifdef DEBUG_ENABLED
	double initial_delta = rem_delta;
	bool potential_infinite = false;
#endif

	while (running && rem_delta > 0) {
		double step_delta = rem_delta;
		step_active = false;

		for (Ref<Tweener> &tweener : tweeners[current_step]) {
			// Modified inside Tweener.step().
			double temp_delta = rem_delta;
			// Turns to true if any Tweener returns true (i.e. is still not finished).
			step_active = tweener->step(temp_delta) || step_active;
			step_delta = MIN(temp_delta, step_delta);
		}

		rem_delta = step_delta;

		if (!step_active) {
			emit_signal(SNAME("step_finished"), current_step);
			current_step++;

			if (current_step == (int)tweeners.size()) {
				loops_done++;
				if (loops_done == loops) {
					running = false;
					dead = true;
					emit_signal(SceneStringName(finished));
					break;
				} else {
					emit_signal(SNAME("loop_finished"), loops_done);
					current_step = 0;
					_start_tweeners();
#ifdef DEBUG_ENABLED
					if (loops <= 0 && Math::is_equal_approx(rem_delta, initial_delta)) {
						if (!potential_infinite) {
							potential_infinite = true;
						} else {
							// Looped twice without using any time, this is 100% certain infinite loop.
							in_step = false;
							ERR_FAIL_V_MSG(false, "Infinite loop detected. Check set_loops() description for more info.");
						}
					}
#endif
				}
			} else {
				_start_tweeners();
			}
		}
	}
	in_step = false;
	return true;
}

bool Tween::can_process(bool p_tree_paused) const {
	if (is_bound && pause_mode == TWEEN_PAUSE_BOUND) {
		Node *node = get_bound_node();
		if (node) {
			return node->is_inside_tree() && node->can_process();
		}
	}

	return !p_tree_paused || pause_mode == TWEEN_PAUSE_PROCESS;
}

Node *Tween::get_bound_node() const {
	if (is_bound) {
		return ObjectDB::get_instance<Node>(bound_node);
	} else {
		return nullptr;
	}
}

double Tween::get_total_time() const {
	return total_time;
}

real_t Tween::run_equation(TransitionType p_trans_type, EaseType p_ease_type, real_t p_time, real_t p_initial, real_t p_delta, real_t p_duration) {
	if (p_duration == 0) {
		// Special case to avoid dividing by 0 in equations.
		return p_initial + p_delta;
	}

	interpolater func = interpolaters[p_trans_type][p_ease_type];
	return func(p_time, p_initial, p_delta, p_duration);
}

Variant Tween::interpolate_variant(const Variant &p_initial_val, const Variant &p_delta_val, double p_time, double p_duration, TransitionType p_trans, EaseType p_ease) {
	ERR_FAIL_INDEX_V(p_trans, TransitionType::TRANS_MAX, Variant());
	ERR_FAIL_INDEX_V(p_ease, EaseType::EASE_MAX, Variant());

	Variant ret = Animation::add_variant(p_initial_val, p_delta_val);
	ret = Animation::interpolate_variant(p_initial_val, ret, run_equation(p_trans, p_ease, p_time, 0.0, 1.0, p_duration), p_initial_val.is_string());
	return ret;
}

String Tween::_to_string() {
	String ret = Object::_to_string();
	Node *node = get_bound_node();
	if (node) {
		ret += vformat(" (bound to %s)", node->get_name());
	}
	return ret;
}

void Tween::_bind_methods() {
	ClassDB::bind_method(D_METHOD("tween_property", "object", "property", "final_val", "duration"), &Tween::tween_property);
	ClassDB::bind_method(D_METHOD("tween_interval", "time"), &Tween::tween_interval);
	ClassDB::bind_method(D_METHOD("tween_callback", "callback"), &Tween::tween_callback);
	ClassDB::bind_method(D_METHOD("tween_method", "method", "from", "to", "duration"), &Tween::tween_method);
	ClassDB::bind_method(D_METHOD("tween_subtween", "subtween"), &Tween::tween_subtween);
	ClassDB::bind_method(D_METHOD("tween_await", "signal"), &Tween::tween_await);

	ClassDB::bind_method(D_METHOD("custom_step", "delta"), &Tween::custom_step);
	ClassDB::bind_method(D_METHOD("stop"), &Tween::stop);
	ClassDB::bind_method(D_METHOD("pause"), &Tween::pause);
	ClassDB::bind_method(D_METHOD("play"), &Tween::play);
	ClassDB::bind_method(D_METHOD("kill"), &Tween::kill);
	ClassDB::bind_method(D_METHOD("get_total_elapsed_time"), &Tween::get_total_time);

	ClassDB::bind_method(D_METHOD("has_tweeners"), &Tween::has_tweeners);
	ClassDB::bind_method(D_METHOD("is_running"), &Tween::is_running);
	ClassDB::bind_method(D_METHOD("is_valid"), &Tween::is_valid);
	ClassDB::bind_method(D_METHOD("bind_node", "node"), &Tween::bind_node);
	ClassDB::bind_method(D_METHOD("set_process_mode", "mode"), &Tween::set_process_mode);
	ClassDB::bind_method(D_METHOD("set_pause_mode", "mode"), &Tween::set_pause_mode);
	ClassDB::bind_method(D_METHOD("set_ignore_time_scale", "ignore"), &Tween::set_ignore_time_scale, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("set_parallel", "parallel"), &Tween::set_parallel, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("set_loops", "loops"), &Tween::set_loops, DEFVAL(0));
	ClassDB::bind_method(D_METHOD("get_loops_left"), &Tween::get_loops_left);
	ClassDB::bind_method(D_METHOD("set_speed_scale", "speed"), &Tween::set_speed_scale);
	ClassDB::bind_method(D_METHOD("set_trans", "trans"), &Tween::set_trans);
	ClassDB::bind_method(D_METHOD("set_ease", "ease"), &Tween::set_ease);

	ClassDB::bind_method(D_METHOD("parallel"), &Tween::parallel);
	ClassDB::bind_method(D_METHOD("chain"), &Tween::chain);

	ClassDB::bind_static_method("Tween", D_METHOD("interpolate_value", "initial_value", "delta_value", "elapsed_time", "duration", "trans_type", "ease_type"), &Tween::interpolate_variant);

	ADD_SIGNAL(MethodInfo("step_finished", PropertyInfo(Variant::INT, "idx")));
	ADD_SIGNAL(MethodInfo("loop_finished", PropertyInfo(Variant::INT, "loop_count")));
	ADD_SIGNAL(MethodInfo("finished"));

	BIND_ENUM_CONSTANT(TWEEN_PROCESS_PHYSICS);
	BIND_ENUM_CONSTANT(TWEEN_PROCESS_IDLE);

	BIND_ENUM_CONSTANT(TWEEN_PAUSE_BOUND);
	BIND_ENUM_CONSTANT(TWEEN_PAUSE_STOP);
	BIND_ENUM_CONSTANT(TWEEN_PAUSE_PROCESS);

	BIND_ENUM_CONSTANT(TRANS_LINEAR);
	BIND_ENUM_CONSTANT(TRANS_SINE);
	BIND_ENUM_CONSTANT(TRANS_QUINT);
	BIND_ENUM_CONSTANT(TRANS_QUART);
	BIND_ENUM_CONSTANT(TRANS_QUAD);
	BIND_ENUM_CONSTANT(TRANS_EXPO);
	BIND_ENUM_CONSTANT(TRANS_ELASTIC);
	BIND_ENUM_CONSTANT(TRANS_CUBIC);
	BIND_ENUM_CONSTANT(TRANS_CIRC);
	BIND_ENUM_CONSTANT(TRANS_BOUNCE);
	BIND_ENUM_CONSTANT(TRANS_BACK);
	BIND_ENUM_CONSTANT(TRANS_SPRING);

	BIND_ENUM_CONSTANT(EASE_IN);
	BIND_ENUM_CONSTANT(EASE_OUT);
	BIND_ENUM_CONSTANT(EASE_IN_OUT);
	BIND_ENUM_CONSTANT(EASE_OUT_IN);
}

Tween::Tween() {
	ERR_FAIL_MSG("Tween can't be created directly. Use create_tween() method.");
}

Tween::Tween(SceneTree *p_parent_tree) {
	parent_tree = p_parent_tree;
	valid = true;
}

double PropertyTweener::_get_custom_interpolated_value(const Variant &p_value) {
	const Variant *argptr = &p_value;

	Variant result;
	Callable::CallError ce;
	custom_method.callp(&argptr, 1, result, ce);
	if (ce.error != Callable::CallError::CALL_OK) {
		ERR_FAIL_V_MSG(false, "Error calling custom method from PropertyTweener: " + Variant::get_callable_error_text(custom_method, &argptr, 1, ce) + ".");
	} else if (result.get_type() != Variant::FLOAT) {
		ERR_FAIL_V_MSG(false, vformat("Wrong return type in PropertyTweener custom method. Expected float, got %s.", Variant::get_type_name(result.get_type())));
	}
	return result;
}

RequiredResult<PropertyTweener> PropertyTweener::from(const Variant &p_value) {
	Ref<Tween> tween = _get_tween();
	ERR_FAIL_COND_V(tween.is_null(), nullptr);

	Variant from_value = p_value;
	if (!Animation::validate_type_match(final_val, from_value)) {
		return nullptr;
	}

	initial_val = from_value;
	do_continue = false;
	return this;
}

RequiredResult<PropertyTweener> PropertyTweener::from_current() {
	do_continue = false;
	return this;
}

RequiredResult<PropertyTweener> PropertyTweener::as_relative() {
	relative = true;
	return this;
}

RequiredResult<PropertyTweener> PropertyTweener::set_trans(Tween::TransitionType p_trans) {
	trans_type = p_trans;
	return this;
}

RequiredResult<PropertyTweener> PropertyTweener::set_ease(Tween::EaseType p_ease) {
	ease_type = p_ease;
	return this;
}

RequiredResult<PropertyTweener> PropertyTweener::set_custom_interpolator(const Callable &p_method) {
	custom_method = p_method;
	return this;
}

RequiredResult<PropertyTweener> PropertyTweener::set_delay(double p_delay) {
	delay = p_delay;
	return this;
}

void PropertyTweener::start() {
	Tweener::start();

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		return;
	}

	if (do_continue) {
		if (Math::is_zero_approx(delay)) {
			initial_val = target_instance->get_indexed(property);
		} else {
			do_continue_delayed = true;
		}
	}

	if (relative) {
		final_val = Animation::add_variant(initial_val, base_final_val);
	}

	delta_val = Animation::subtract_variant(final_val, initial_val);
}

bool PropertyTweener::step(double &r_delta) {
	if (finished) {
		// This is needed in case there's a parallel Tweener with longer duration.
		return false;
	}

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		_finish();
		return false;
	}
	elapsed_time += r_delta;

	if (elapsed_time < delay) {
		r_delta = 0;
		return true;
	} else if (do_continue_delayed && !Math::is_zero_approx(delay)) {
		initial_val = target_instance->get_indexed(property);
		delta_val = Animation::subtract_variant(final_val, initial_val);
		do_continue_delayed = false;
	}

	Ref<Tween> tween = _get_tween();

	double time = MIN(elapsed_time - delay, duration);
	if (time < duration) {
		if (custom_method.is_valid()) {
			const Variant t = tween->interpolate_variant(0.0, 1.0, time, duration, trans_type, ease_type);
			double result = _get_custom_interpolated_value(t);
			target_instance->set_indexed(property, Animation::interpolate_variant(initial_val, final_val, result));
		} else {
			target_instance->set_indexed(property, tween->interpolate_variant(initial_val, delta_val, time, duration, trans_type, ease_type));
		}
		r_delta = 0;
		return true;
	} else {
		if (custom_method.is_valid()) {
			double final_t = _get_custom_interpolated_value(1.0);
			target_instance->set_indexed(property, Animation::interpolate_variant(initial_val, final_val, final_t));
		} else {
			target_instance->set_indexed(property, final_val);
		}
		r_delta = elapsed_time - delay - duration;
		_finish();
		return false;
	}
}

void PropertyTweener::set_tween(const Ref<Tween> &p_tween) {
	Tweener::set_tween(p_tween);
	if (trans_type == Tween::TRANS_MAX) {
		trans_type = p_tween->get_trans();
	}
	if (ease_type == Tween::EASE_MAX) {
		ease_type = p_tween->get_ease();
	}
}

void PropertyTweener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("from", "value"), &PropertyTweener::from);
	ClassDB::bind_method(D_METHOD("from_current"), &PropertyTweener::from_current);
	ClassDB::bind_method(D_METHOD("as_relative"), &PropertyTweener::as_relative);
	ClassDB::bind_method(D_METHOD("set_trans", "trans"), &PropertyTweener::set_trans);
	ClassDB::bind_method(D_METHOD("set_ease", "ease"), &PropertyTweener::set_ease);
	ClassDB::bind_method(D_METHOD("set_custom_interpolator", "interpolator_method"), &PropertyTweener::set_custom_interpolator);
	ClassDB::bind_method(D_METHOD("set_delay", "delay"), &PropertyTweener::set_delay);
}

PropertyTweener::PropertyTweener(const Object *p_target, const Vector<StringName> &p_property, const Variant &p_to, double p_duration) {
	target = p_target->get_instance_id();
	property = p_property;
	initial_val = p_target->get_indexed(property);
	base_final_val = p_to;
	final_val = base_final_val;
	duration = p_duration;

	if (p_target->is_ref_counted()) {
		ref_copy = p_target;
	}
}

PropertyTweener::PropertyTweener() {
	if (!det_restore_in_progress) {
		ERR_FAIL_MSG("PropertyTweener can't be created directly. Use the tween_property() method in Tween.");
	}
}

bool IntervalTweener::step(double &r_delta) {
	if (finished) {
		return false;
	}

	elapsed_time += r_delta;

	if (elapsed_time < duration) {
		r_delta = 0;
		return true;
	} else {
		r_delta = elapsed_time - duration;
		_finish();
		return false;
	}
}

IntervalTweener::IntervalTweener(double p_time) {
	duration = p_time;
}

IntervalTweener::IntervalTweener() {
	if (!det_restore_in_progress) {
		ERR_FAIL_MSG("IntervalTweener can't be created directly. Use the tween_interval() method in Tween.");
	}
}

RequiredResult<CallbackTweener> CallbackTweener::set_delay(double p_delay) {
	delay = p_delay;
	return this;
}

bool CallbackTweener::step(double &r_delta) {
	if (finished) {
		return false;
	}

	if (!callback.is_valid()) {
		_finish();
		return false;
	}

	elapsed_time += r_delta;
	if (elapsed_time >= delay) {
		Variant result;
		Callable::CallError ce;
		callback.callp(nullptr, 0, result, ce);
		if (ce.error != Callable::CallError::CALL_OK) {
			ERR_FAIL_V_MSG(false, "Error calling method from CallbackTweener: " + Variant::get_callable_error_text(callback, nullptr, 0, ce) + ".");
		}

		r_delta = elapsed_time - delay;
		_finish();
		return false;
	}

	r_delta = 0;
	return true;
}

void CallbackTweener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_delay", "delay"), &CallbackTweener::set_delay);
}

CallbackTweener::CallbackTweener(const Callable &p_callback) {
	callback = p_callback;

	Object *callback_instance = p_callback.get_object();
	if (callback_instance && callback_instance->is_ref_counted()) {
		ref_copy = callback_instance;
	}
}

CallbackTweener::CallbackTweener() {
	if (!det_restore_in_progress) {
		ERR_FAIL_MSG("CallbackTweener can't be created directly. Use the tween_callback() method in Tween.");
	}
}

RequiredResult<MethodTweener> MethodTweener::set_delay(double p_delay) {
	delay = p_delay;
	return this;
}

RequiredResult<MethodTweener> MethodTweener::set_trans(Tween::TransitionType p_trans) {
	trans_type = p_trans;
	return this;
}

RequiredResult<MethodTweener> MethodTweener::set_ease(Tween::EaseType p_ease) {
	ease_type = p_ease;
	return this;
}

bool MethodTweener::step(double &r_delta) {
	if (finished) {
		return false;
	}

	if (!callback.is_valid()) {
		_finish();
		return false;
	}

	elapsed_time += r_delta;

	if (elapsed_time < delay) {
		r_delta = 0;
		return true;
	}

	Ref<Tween> tween = _get_tween();

	Variant current_val;
	double time = MIN(elapsed_time - delay, duration);
	if (time < duration) {
		current_val = tween->interpolate_variant(initial_val, delta_val, time, duration, trans_type, ease_type);
	} else {
		current_val = final_val;
	}
	const Variant **argptr = (const Variant **)alloca(sizeof(Variant *));
	argptr[0] = &current_val;

	Variant result;
	Callable::CallError ce;
	callback.callp(argptr, 1, result, ce);
	if (ce.error != Callable::CallError::CALL_OK) {
		ERR_FAIL_V_MSG(false, "Error calling method from MethodTweener: " + Variant::get_callable_error_text(callback, argptr, 1, ce) + ".");
	}

	if (time < duration) {
		r_delta = 0;
		return true;
	} else {
		r_delta = elapsed_time - delay - duration;
		_finish();
		return false;
	}
}

void MethodTweener::set_tween(const Ref<Tween> &p_tween) {
	Tweener::set_tween(p_tween);
	if (trans_type == Tween::TRANS_MAX) {
		trans_type = p_tween->get_trans();
	}
	if (ease_type == Tween::EASE_MAX) {
		ease_type = p_tween->get_ease();
	}
}

void MethodTweener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_delay", "delay"), &MethodTweener::set_delay);
	ClassDB::bind_method(D_METHOD("set_trans", "trans"), &MethodTweener::set_trans);
	ClassDB::bind_method(D_METHOD("set_ease", "ease"), &MethodTweener::set_ease);
}

MethodTweener::MethodTweener(const Callable &p_callback, const Variant &p_from, const Variant &p_to, double p_duration) {
	callback = p_callback;
	initial_val = p_from;
	delta_val = Animation::subtract_variant(p_to, p_from);
	final_val = p_to;
	duration = p_duration;

	Object *callback_instance = p_callback.get_object();
	if (callback_instance && callback_instance->is_ref_counted()) {
		ref_copy = callback_instance;
	}
}

MethodTweener::MethodTweener() {
	if (!det_restore_in_progress) {
		ERR_FAIL_MSG("MethodTweener can't be created directly. Use the tween_method() method in Tween.");
	}
}

void SubtweenTweener::start() {
	Tweener::start();

	// Reset the subtween.
	subtween->stop();

	// It's possible that a subtween could be killed before it is started;
	// if so, we just want to skip it entirely.
	if (subtween->is_valid()) {
		subtween->play();
	} else {
		_finish();
	}
}

bool SubtweenTweener::step(double &r_delta) {
	if (finished) {
		return false;
	}

	elapsed_time += r_delta;

	if (elapsed_time < delay) {
		r_delta = 0;
		return true;
	}

	if (!subtween->step(r_delta)) {
		r_delta = elapsed_time - delay - subtween->get_total_time();
		_finish();
		return false;
	}

	r_delta = 0;
	return true;
}

RequiredResult<SubtweenTweener> SubtweenTweener::set_delay(double p_delay) {
	delay = p_delay;
	return this;
}

void SubtweenTweener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_delay", "delay"), &SubtweenTweener::set_delay);
}

SubtweenTweener::SubtweenTweener(const Ref<Tween> &p_subtween) {
	subtween = p_subtween;
}

SubtweenTweener::SubtweenTweener() {
	if (!det_restore_in_progress) {
		ERR_FAIL_MSG("SubtweenTweener can't be created directly. Use the tween_subtween() method in Tween.");
	}
}

Ref<AwaitTweener> AwaitTweener::set_timeout(double p_timeout) {
	timeout = p_timeout;
	return this;
}

void AwaitTweener::start() {
	Tweener::start();
	received = false;
}

bool AwaitTweener::step(double &r_delta) {
	if (finished) {
		return false;
	}

	if (!signal.get_object() || !signal.is_connected(target_callable)) { // In case the object was destroyed before emitting.
		_finish();
		return false;
	}

	elapsed_time += r_delta;

	if (timeout >= 0 && elapsed_time >= timeout) {
		_finish();
		r_delta = elapsed_time - timeout;
		return false;
	}

	r_delta = 0; // "Consume" all remaining time to prevent infinite loops.
	if (received) {
		_finish();
		return false;
	}
	return true;
}

AwaitTweener::AwaitTweener(const Signal &p_signal) {
	signal = p_signal;
	target_callable = Callable(this, SNAME("_signal_callback"));
	signal.connect(target_callable);

	Object *signal_instance = p_signal.get_object();
	if (signal_instance && signal_instance->is_ref_counted()) {
		ref_copy = signal_instance;
	}
}

AwaitTweener::AwaitTweener() {
	if (!det_restore_in_progress) {
		ERR_FAIL_MSG("AwaitTweener can't be created directly. Use the tween_await() method in Tween.");
	}
}

void AwaitTweener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_timeout", "timeout"), &AwaitTweener::set_timeout);
	ClassDB::bind_vararg_method(METHOD_FLAGS_DEFAULT, "_signal_callback", &AwaitTweener::_signal_received, MethodInfo("_signal_callback"));
}

void AwaitTweener::_signal_received(const Variant **p_args, int p_argcount, Callable::CallError &r_error) {
	received = true;
}

/* --------- YGIAH (det-snapshot) --------- */

bool Tweener::det_restore_in_progress = false;

static Dictionary _det_serialize_callable(const Callable &p_c) {
	Dictionary d;
	if (!p_c.is_valid()) {
		d["null"] = true;
		return d;
	}
	ObjectID oid = p_c.get_object_id();
	if (p_c.is_custom() && oid.is_null()) {
		// Function-pointer/lambda callables cannot be reconstructed.
		d["unsupported"] = true;
		WARN_PRINT("det-snapshot: custom callable without object target cannot be serialized.");
		return d;
	}
	d["oid"] = (int64_t)(uint64_t)oid;
	d["method"] = p_c.get_method();
	if (p_c.get_bound_arguments_count() > 0) {
		d["binds"] = p_c.get_bound_arguments();
	}
	return d;
}

static Callable _det_deserialize_callable(const Dictionary &d) {
	if (d.get("null", false) || d.get("unsupported", false)) {
		return Callable();
	}
	Callable c(ObjectID((uint64_t)(int64_t)d["oid"]), d["method"]);
	if (d.has("binds")) {
		c = c.bindv(d["binds"]);
	}
	return c;
}

String Tweener::_det_type() const {
	return String(get_class());
}

Dictionary Tween::_det_capture() const {
	Dictionary d;
	d["iid"] = (int64_t)(uint64_t)get_instance_id();
	d["process_mode"] = (int)process_mode;
	d["pause_mode"] = (int)pause_mode;
	d["default_transition"] = (int)default_transition;
	d["default_ease"] = (int)default_ease;
	d["bound_node"] = (int64_t)(uint64_t)bound_node;
	d["is_bound"] = is_bound;
	d["total_time"] = total_time;
	d["current_step"] = current_step;
	d["loops"] = loops;
	d["loops_done"] = loops_done;
	d["speed_scale"] = speed_scale;
	d["ignore_time_scale"] = ignore_time_scale;
	d["started"] = started;
	d["running"] = running;
	d["dead"] = dead;
	d["valid"] = valid;
	d["default_parallel"] = default_parallel;
	d["parallel_enabled"] = parallel_enabled;

	Array steps;
	for (uint32_t s = 0; s < tweeners.size(); s++) {
		Array step_list;
		for (const Ref<Tweener> &tw : tweeners[s]) {
			Dictionary td = tw->_det_capture();
			td["type"] = tw->_det_type();
			td["iid"] = (int64_t)(uint64_t)tw->get_instance_id();
			td["elapsed"] = tw->elapsed_time;
			td["finished"] = tw->finished;
			step_list.push_back(td);
		}
		steps.push_back(step_list);
	}
	d["tweeners"] = steps;

	Array subs;
	for (const Ref<Tween> &st : subtweens) {
		subs.push_back(st->_det_capture());
	}
	d["subtweens"] = subs;
	return d;
}

Error Tween::_det_restore(const Dictionary &d) {
	process_mode = (TweenProcessMode)(int)d["process_mode"];
	pause_mode = (TweenPauseMode)(int)d["pause_mode"];
	default_transition = (TransitionType)(int)d["default_transition"];
	default_ease = (EaseType)(int)d["default_ease"];
	bound_node = ObjectID((uint64_t)(int64_t)d["bound_node"]);
	is_bound = d["is_bound"];
	total_time = d["total_time"];
	current_step = d["current_step"];
	loops = d["loops"];
	loops_done = d["loops_done"];
	speed_scale = d["speed_scale"];
	ignore_time_scale = d["ignore_time_scale"];
	started = d["started"];
	running = d["running"];
	dead = d["dead"];
	valid = d["valid"];
	default_parallel = d["default_parallel"];
	parallel_enabled = d["parallel_enabled"];
	in_step = false;

	tweeners.clear();
	Array steps = d["tweeners"];
	tweeners.resize(steps.size());
	for (int s = 0; s < steps.size(); s++) {
		Array step_list = steps[s];
		for (int i = 0; i < step_list.size(); i++) {
			Dictionary td = step_list[i];
			String type = td["type"];
			Ref<Tweener> tw;
			Tweener::det_restore_in_progress = true;
			if (type == "PropertyTweener") {
				Ref<PropertyTweener> t;
				t.instantiate();
				tw = t;
			} else if (type == "IntervalTweener") {
				Ref<IntervalTweener> t;
				t.instantiate();
				tw = t;
			} else if (type == "CallbackTweener") {
				Ref<CallbackTweener> t;
				t.instantiate();
				tw = t;
			} else if (type == "MethodTweener") {
				Ref<MethodTweener> t;
				t.instantiate();
				tw = t;
			} else if (type == "SubtweenTweener") {
				Ref<SubtweenTweener> t;
				t.instantiate();
				tw = t;
			} else if (type == "AwaitTweener") {
				Ref<AwaitTweener> t;
				t.instantiate();
				tw = t;
			} else {
				Tweener::det_restore_in_progress = false;
				ERR_FAIL_V_MSG(ERR_INVALID_DATA, "det-snapshot: unknown tweener type " + type);
			}
			Tweener::det_restore_in_progress = false;
			// Reclaim the original ObjectID before any further allocation.
			ObjectDB::claim_instance_id(tw.ptr(), ObjectID((uint64_t)(int64_t)td["iid"]));
			tw->set_tween(this);
			tw->elapsed_time = td["elapsed"];
			tw->finished = td["finished"];
			tw->_det_restore(td);
			tweeners[s].push_back(tw);
		}
	}

	subtweens.clear();
	Array subs = d["subtweens"];
	for (int i = 0; i < subs.size(); i++) {
		Dictionary sd = subs[i];
		Ref<Tween> st = memnew(Tween(parent_tree));
		ObjectDB::claim_instance_id(st.ptr(), ObjectID((uint64_t)(int64_t)sd["iid"]));
		st->_det_restore(sd);
		subtweens.push_back(st);
	}
	return OK;
}

Dictionary PropertyTweener::_det_capture() const {
	Dictionary d;
	d["target"] = (int64_t)(uint64_t)target;
	Array prop;
	for (const StringName &p : property) {
		prop.push_back(p);
	}
	d["property"] = prop;
	d["initial_val"] = initial_val;
	d["base_final_val"] = base_final_val;
	d["final_val"] = final_val;
	d["delta_val"] = delta_val;
	d["duration"] = duration;
	d["trans_type"] = (int)trans_type;
	d["ease_type"] = (int)ease_type;
	d["custom_method"] = _det_serialize_callable(custom_method);
	d["delay"] = delay;
	d["do_continue"] = do_continue;
	d["do_continue_delayed"] = do_continue_delayed;
	d["relative"] = relative;
	return d;
}

void PropertyTweener::_det_restore(const Dictionary &d) {
	target = ObjectID((uint64_t)(int64_t)d["target"]);
	Array prop = d["property"];
	property.resize(prop.size());
	for (int i = 0; i < prop.size(); i++) {
		property.write[i] = prop[i];
	}
	initial_val = d["initial_val"];
	base_final_val = d["base_final_val"];
	final_val = d["final_val"];
	delta_val = d["delta_val"];
	duration = d["duration"];
	trans_type = (Tween::TransitionType)(int)d["trans_type"];
	ease_type = (Tween::EaseType)(int)d["ease_type"];
	custom_method = _det_deserialize_callable(d["custom_method"]);
	delay = d["delay"];
	do_continue = d["do_continue"];
	do_continue_delayed = d["do_continue_delayed"];
	relative = d["relative"];
	Object *t = ObjectDB::get_instance(target);
	if (t && t->is_ref_counted()) {
		ref_copy = t;
	}
}

Dictionary IntervalTweener::_det_capture() const {
	Dictionary d;
	d["duration"] = duration;
	return d;
}

void IntervalTweener::_det_restore(const Dictionary &d) {
	duration = d["duration"];
}

Dictionary CallbackTweener::_det_capture() const {
	Dictionary d;
	d["callback"] = _det_serialize_callable(callback);
	d["delay"] = delay;
	return d;
}

void CallbackTweener::_det_restore(const Dictionary &d) {
	callback = _det_deserialize_callable(d["callback"]);
	delay = d["delay"];
	Object *t = ObjectDB::get_instance(callback.get_object_id());
	if (t && t->is_ref_counted()) {
		ref_copy = t;
	}
}

Dictionary MethodTweener::_det_capture() const {
	Dictionary d;
	d["duration"] = duration;
	d["delay"] = delay;
	d["trans_type"] = (int)trans_type;
	d["ease_type"] = (int)ease_type;
	d["initial_val"] = initial_val;
	d["delta_val"] = delta_val;
	d["final_val"] = final_val;
	d["callback"] = _det_serialize_callable(callback);
	return d;
}

void MethodTweener::_det_restore(const Dictionary &d) {
	duration = d["duration"];
	delay = d["delay"];
	trans_type = (Tween::TransitionType)(int)d["trans_type"];
	ease_type = (Tween::EaseType)(int)d["ease_type"];
	initial_val = d["initial_val"];
	delta_val = d["delta_val"];
	final_val = d["final_val"];
	callback = _det_deserialize_callable(d["callback"]);
	Object *t = ObjectDB::get_instance(callback.get_object_id());
	if (t && t->is_ref_counted()) {
		ref_copy = t;
	}
}

Dictionary SubtweenTweener::_det_capture() const {
	Dictionary d;
	d["delay"] = delay;
	if (subtween.is_valid()) {
		d["subtween"] = subtween->_det_capture();
	}
	return d;
}

void SubtweenTweener::_det_restore(const Dictionary &d) {
	delay = d["delay"];
	if (d.has("subtween")) {
		Dictionary sd = d["subtween"];
		subtween = Ref<Tween>(memnew(Tween));
		ObjectDB::claim_instance_id(subtween.ptr(), ObjectID((uint64_t)(int64_t)sd["iid"]));
		subtween->_det_restore(sd);
	}
}

Dictionary AwaitTweener::_det_capture() const {
	Dictionary d;
	d["signal_name"] = signal.get_name();
	Object *so = signal.get_object();
	d["signal_oid"] = (int64_t)(uint64_t)(so ? so->get_instance_id() : ObjectID());
	d["target_callable"] = _det_serialize_callable(target_callable);
	d["received"] = received;
	d["timeout"] = timeout;
	return d;
}

void AwaitTweener::_det_restore(const Dictionary &d) {
	Object *so = ObjectDB::get_instance(ObjectID((uint64_t)(int64_t)d["signal_oid"]));
	if (so) {
		signal = Signal(so, d["signal_name"]);
	}
	target_callable = Callable(this, SNAME("_signal_callback"));
	if (signal.is_null() == false) {
		signal.connect(target_callable);
	}
	received = d["received"];
	timeout = d["timeout"];
	if (so && so->is_ref_counted()) {
		ref_copy = so;
	}
}
