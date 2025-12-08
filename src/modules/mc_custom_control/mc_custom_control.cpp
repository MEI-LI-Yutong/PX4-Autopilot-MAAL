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

using matrix::Vector3f;

MulticopterCustomControl::MulticopterCustomControl() :
	ModuleParams(nullptr),
	px4::WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_trajectory_setpoint = PositionControl::empty_trajectory_setpoint;
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

	updateParams();
	ParametersUpdated();

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

void MulticopterCustomControl::ParametersUpdated()
{
	UpdatePositionControlGains();
	UpdateAttitudeControlGains();
	UpdateRateControlGains();
}

void MulticopterCustomControl::UpdatePositionControlGains()
{
	_position_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));

	const Vector3f vel_p(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get());
	const Vector3f vel_i(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get());
	const Vector3f vel_d(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get());
	_position_control.setVelocityGains(vel_p, vel_i, vel_d);

	_position_control.setVelocityLimits(_param_mpc_xy_vel_max.get(),
					    _param_mpc_z_vel_max_up.get(),
					    _param_mpc_z_vel_max_dn.get());

	_position_control.setThrustLimits(_param_mpc_thr_min.get(), _param_mpc_thr_max.get());
	_position_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
	_position_control.setTiltLimit(math::radians(_param_mpc_tiltmax_air.get()));

	_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
	_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
}

void MulticopterCustomControl::UpdateAttitudeControlGains()
{
	const Vector3f att_p(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get());
	_attitude_control.setProportionalGain(att_p, _param_mc_yaw_weight.get());
	_attitude_control.setRateLimit(Vector3f(
					     math::radians(_param_mc_rollrate_max.get()),
					     math::radians(_param_mc_pitchrate_max.get()),
					     math::radians(_param_mc_yawrate_max.get())));
}

void MulticopterCustomControl::UpdateRateControlGains()
{
	// 使用 MC_*RATE_K 作为简单对角 LQR 增益，便于快速试飞
	_lqr_gains = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());
}

matrix::Vector3f MulticopterCustomControl::ComputeLqrTorque(const matrix::Vector3f &rates,
		const matrix::Vector3f &rates_sp)
{
	// u = -K * (rate - rate_sp)
	const Vector3f rate_error = rates - rates_sp;
	return -_lqr_gains.emult(rate_error);
}

bool MulticopterCustomControl::RunPositionControl(float dt, vehicle_attitude_setpoint_s &att_sp,
		matrix::Vector3f &thrust_sp)
{
	if (!_vehicle_control_mode.flag_control_position_enabled || !_vehicle_control_mode.flag_control_velocity_enabled) {
		_position_control.resetIntegral();
		return false;
	}

	PositionControlStates states{};
	states.position = Vector3f(_position.x, _position.y, _position.z);
	states.velocity = Vector3f(_position.vx, _position.vy, _position.vz);
	states.acceleration = Vector3f(_position.ax, _position.ay, _position.az);
	states.yaw = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();

	_position_control.setState(states);

	trajectory_setpoint_s current_setpoint = PositionControl::empty_trajectory_setpoint;

	if (_trajectory_setpoint_sub.updated()) {
		_trajectory_setpoint_sub.copy(&_trajectory_setpoint);
	}

	current_setpoint = _trajectory_setpoint;
	_position_control.setInputSetpoint(current_setpoint);

	const bool hover_valid = (_hover_thrust_estimate.timestamp != 0)
				 && (hrt_elapsed_time(&_hover_thrust_estimate.timestamp) < 1_s)
				 && PX4_ISFINITE(_hover_thrust_estimate.hover_thrust);
	const float hover_thrust = hover_valid ? _hover_thrust_estimate.hover_thrust : _param_mpc_thr_hover.get();
	_position_control.updateHoverThrust(hover_thrust);
	_position_control.setHoverThrust(hover_thrust);

	// update constraints & takeoff handling
	_vehicle_constraints_sub.update(&_vehicle_constraints);

	if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
		_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	}

	// 起飞判定：armed 且 setpoint/约束要求上升
	const bool want_takeoff = _vehicle_control_mode.flag_armed &&
				  (_vehicle_constraints.want_takeoff
				   || (PX4_ISFINITE(current_setpoint.position[2]) && (current_setpoint.position[2] < states.position(2)))
				   || (PX4_ISFINITE(current_setpoint.velocity[2]) && (current_setpoint.velocity[2] < 0.f))
				   || (PX4_ISFINITE(current_setpoint.acceleration[2]) && (current_setpoint.acceleration[2] < 0.f)));

	_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _land_detected.landed, want_takeoff,
				    _vehicle_constraints.speed_up, false, hrt_absolute_time());

	const bool not_taken_off = (_takeoff.getTakeoffState() < TakeoffState::rampup);

	// during takeoff ramp, freeze accel feed-forward on vertical axis
	if (_takeoff.getTakeoffState() == TakeoffState::rampup) {
		current_setpoint.acceleration[2] = NAN;
	}

	if (not_taken_off) {
		current_setpoint = PositionControl::empty_trajectory_setpoint;
		current_setpoint.timestamp = hrt_absolute_time();
		Vector3f(0.f, 0.f, 100.f).copyTo(current_setpoint.acceleration);
		_position_control.resetIntegral();
	}

	const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
				     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
	_position_control.setTiltLimit(math::radians(tilt_limit_deg));

	const float speed_up = _takeoff.updateRamp(dt, PX4_ISFINITE(_vehicle_constraints.speed_up)
				  ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
	const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
				 _param_mpc_z_vel_max_dn.get();

	const float minimum_thrust = not_taken_off ? 0.f : _param_mpc_thr_min.get();
	_position_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

	float max_speed_xy = _param_mpc_xy_vel_max.get();

	if (PX4_ISFINITE(_position.vxy_max)) {
		max_speed_xy = math::min(max_speed_xy, _position.vxy_max);
	}

	_position_control.setVelocityLimits(
		max_speed_xy,
		math::min(speed_up, _param_mpc_z_vel_max_up.get()),
		math::max(speed_down, 0.f));

	_position_control.setInputSetpoint(current_setpoint);

	const bool pos_ok = _position_control.update(dt);

	if (!pos_ok) {
		return false;
	}

	vehicle_local_position_setpoint_s local_sp{};
	_position_control.getLocalPositionSetpoint(local_sp);
	local_sp.timestamp = hrt_absolute_time();
	_local_pos_sp_pub.publish(local_sp);

	_position_control.getAttitudeSetpoint(att_sp);
	att_sp.timestamp = hrt_absolute_time();
	_att_sp_pub.publish(att_sp);

	thrust_sp = Vector3f(att_sp.thrust_body[0], att_sp.thrust_body[1], att_sp.thrust_body[2]);
	return true;
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
		updateParams();
		ParametersUpdated();
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_attitude_sub.update(&_attitude);
	_position_sub.update(&_position);
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);
	_vehicle_land_detected_sub.update(&_land_detected);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_hover_thrust_estimate_sub.update(&_hover_thrust_estimate);
	_vehicle_constraints_sub.update(&_vehicle_constraints);

	perf_begin(_loop_perf);

	if (_last_run == 0) {
		_last_run = _angular_velocity.timestamp_sample;
		perf_end(_loop_perf);
		return;
	}

	const float dt = math::constrain(((_angular_velocity.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
	_last_run = _angular_velocity.timestamp_sample;

	vehicle_attitude_setpoint_s att_sp{};

	if (RunPositionControl(dt, att_sp, _thrust_sp)) {
		matrix::Quatf qd(att_sp.q_d);
		_attitude_control.setAttitudeSetpoint(qd, att_sp.yaw_sp_move_rate);
		_rates_sp = _attitude_control.update(matrix::Quatf(_attitude.q));
	} else {
		_rates_sp.zero();
		_thrust_sp.zero();
	}

	const Vector3f torque_sp = ComputeLqrTorque(Vector3f(_angular_velocity.xyz), _rates_sp);

	vehicle_rates_setpoint_s rates_setpoint{};
	rates_setpoint.timestamp = hrt_absolute_time();
	rates_setpoint.roll = _rates_sp(0);
	rates_setpoint.pitch = _rates_sp(1);
	rates_setpoint.yaw = _rates_sp(2);
	rates_setpoint.thrust_body[0] = _thrust_sp(0);
	rates_setpoint.thrust_body[1] = _thrust_sp(1);
	rates_setpoint.thrust_body[2] = _thrust_sp(2);
	_vehicle_rates_setpoint_pub.publish(rates_setpoint);

	vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
	vehicle_thrust_setpoint.timestamp_sample = _angular_velocity.timestamp_sample;
	vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
	_thrust_sp.copyTo(vehicle_thrust_setpoint.xyz);
	_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

	vehicle_torque_setpoint_s vehicle_torque_setpoint{};
	vehicle_torque_setpoint.timestamp_sample = _angular_velocity.timestamp_sample;
	vehicle_torque_setpoint.timestamp = hrt_absolute_time();
	vehicle_torque_setpoint.xyz[0] = torque_sp(0);
	vehicle_torque_setpoint.xyz[1] = torque_sp(1);
	vehicle_torque_setpoint.xyz[2] = torque_sp(2);
	_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

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
