/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mc_custom_control.hpp"

#include <px4_platform_common/getopt.h>

#include <cmath>
#include <inttypes.h>

MulticopterCustomControl::MulticopterCustomControl() :
	px4::WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

MulticopterCustomControl::~MulticopterCustomControl()
{
	perf_free(_loop_perf);
}

bool MulticopterCustomControl::init()
{
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int MulticopterCustomControl::task_spawn(int argc, char *argv[])
{
	MulticopterCustomControl *instance = new MulticopterCustomControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void MulticopterCustomControl::ComputeOutputs(float dt, float servo_out[3], float motor_out[3])
{
	(void)dt;

	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool manual_valid = (_manual_control_setpoint.timestamp != 0)
				  && (hrt_elapsed_time(&_manual_control_setpoint.timestamp) < 500_ms);

	// 默认输出用于接口验证：servo0 = 0.5，motor0 = 20%
	for (int i = 0; i < 3; i++) {
		servo_out[i] = 0.0f;
		motor_out[i] = 0.0f;
	}

	servo_out[0] = 0.5f;
	motor_out[0] = 0.2f;

	if (!armed || !manual_valid) {
		return;
	}

	const float rate_damping = 0.1f;

	float roll_sp = PX4_ISFINITE(_manual_control_setpoint.roll) ? _manual_control_setpoint.roll : 0.0f;
	float pitch_sp = PX4_ISFINITE(_manual_control_setpoint.pitch) ? _manual_control_setpoint.pitch : 0.0f;
	float yaw_sp = PX4_ISFINITE(_manual_control_setpoint.yaw) ? _manual_control_setpoint.yaw : 0.0f;
	float throttle_sp = PX4_ISFINITE(_manual_control_setpoint.throttle) ? _manual_control_setpoint.throttle : -1.0f;

	roll_sp = math::constrain(roll_sp - rate_damping * _angular_velocity.xyz[0], -1.0f, 1.0f);
	pitch_sp = math::constrain(pitch_sp - rate_damping * _angular_velocity.xyz[1], -1.0f, 1.0f);
	yaw_sp = math::constrain(yaw_sp - rate_damping * _angular_velocity.xyz[2], -1.0f, 1.0f);

	servo_out[0] = roll_sp;
	servo_out[1] = pitch_sp;
	servo_out[2] = yaw_sp;

	const float thrust = math::constrain((throttle_sp + 1.0f) * 0.5f, 0.0f, 1.0f);

	for (int i = 0; i < 3; i++) {
		motor_out[i] = thrust;
	}
}

void MulticopterCustomControl::PublishServos(const float servo_out[3], hrt_abstime now)
{
	const bool publish_servo = (_last_servo_publish == 0) || ((now - _last_servo_publish) >= 100_ms);

	if (!publish_servo) {
		return;
	}

	actuator_servos_s actuator_servos{};
	actuator_servos.timestamp = now;
	actuator_servos.timestamp_sample = _angular_velocity.timestamp_sample;

	for (int i = 0; i < 3; i++) {
		float servo_value = PX4_ISFINITE(servo_out[i]) ? servo_out[i] : NAN;
		_last_servo_values[i] = servo_value;
		actuator_servos.control[i] = servo_value;
	}

	for (int i = 3; i < actuator_servos_s::NUM_CONTROLS; i++) {
		_last_servo_values[i] = NAN;
		actuator_servos.control[i] = NAN;
	}

	_actuator_servos_pub.publish(actuator_servos);
	_last_servo_publish = now;
}

void MulticopterCustomControl::PublishMotors(const float motor_out[3], hrt_abstime now)
{
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = now;
	actuator_motors.timestamp_sample = _angular_velocity.timestamp_sample;

	for (int i = 0; i < 3; i++) {
		float motor_value = PX4_ISFINITE(motor_out[i]) ? motor_out[i] : NAN;
		motor_value = PX4_ISFINITE(motor_value) ? math::constrain(motor_value, 0.0f, 1.0f) : NAN;
		actuator_motors.control[i] = motor_value;
	}

	for (int i = 3; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	actuator_motors.reversible_flags = 0;
	_actuator_motors_pub.publish(actuator_motors);
}

void MulticopterCustomControl::Run()
{
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (!_angular_velocity_sub.update(&_angular_velocity)) {
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
	}

	if (_attitude_sub.updated()) {
		_attitude_sub.copy(&_attitude);
	}

	if (_position_sub.updated()) {
		_position_sub.copy(&_position);
	}

	if (_manual_control_setpoint_sub.updated()) {
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
	}

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	perf_begin(_loop_perf);

	if (_last_run == 0) {
		_last_run = _angular_velocity.timestamp_sample;
		perf_end(_loop_perf);
		return;
	}

	const float dt = math::constrain(((_angular_velocity.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
	_last_run = _angular_velocity.timestamp_sample;

	float servo_out[3] {};
	float motor_out[3] {};

	ComputeOutputs(dt, servo_out, motor_out);

	const hrt_abstime now = hrt_absolute_time();
	PublishServos(servo_out, now);
	PublishMotors(motor_out, now);

	perf_end(_loop_perf);
}

int MulticopterCustomControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterCustomControl::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Last servo publish: %" PRIu64, _last_servo_publish);
	return 0;
}

int MulticopterCustomControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Custom multicopter control module that reads core state (attitude, rates, position, manual setpoints)
and produces raw actuator outputs (servos + motors) inside a single WorkItem. Intended as a starting
point for experimenting with unified control strategies without touching the existing mc_* controllers.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_custom_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_custom_control_main(int argc, char *argv[])
{
	return MulticopterCustomControl::main(argc, argv);
}
