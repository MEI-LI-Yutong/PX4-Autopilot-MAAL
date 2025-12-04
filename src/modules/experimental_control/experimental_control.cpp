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

#include "experimental_control.hpp"

#include <cstring>
#include <cmath>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

namespace
{

class ZeroBackend : public ControllerBackend
{
public:
	const char *name() const override { return "zero_backend"; }

	bool init() override { return true; }

	bool update(const float input[15], float output[6], uint32_t &duration_us) override
	{
		const hrt_abstime start = hrt_absolute_time();

		for (int i = 0; i < 6; i++) {
			output[i] = 0.f;
		}

		duration_us = static_cast<uint32_t>(hrt_absolute_time() - start);

		(void)input;
		return true;
	}
};

class ConstantBackend : public ControllerBackend
{
public:
	const char *name() const override { return "constant_backend"; }

	bool init() override { return true; }

	bool update(const float input[15], float output[6], uint32_t &duration_us) override
	{
		const hrt_abstime start = hrt_absolute_time();

		(void)input;

		output[0] = 0.5f; // fixed non-zero output on channel 0 for verification
		output[1] = 0.0f;
		output[2] = 0.0f;
		output[3] = 0.0f;
		output[4] = 0.0f;
		output[5] = 0.0f;

		duration_us = static_cast<uint32_t>(hrt_absolute_time() - start);
		return true;
	}
};

} // namespace

ExperimentalControl::ExperimentalControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_trajectory_setpoint.position[0] = NAN;
	_trajectory_setpoint.position[1] = NAN;
	_trajectory_setpoint.position[2] = NAN;
}

ExperimentalControl::~ExperimentalControl()
{
	perf_free(_loop_perf);
}

bool ExperimentalControl::init()
{
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_controller_ready = setup_controller_backend();
	return _controller_ready;
}

bool ExperimentalControl::setup_controller_backend()
{
	const int backend_type = _param_backend_type.get();

	if (backend_type == 0) {
		_controller = std::make_unique<ZeroBackend>();
		_force_zero_motors = false;

	} else if (backend_type == 1) {
		_controller = std::make_unique<ConstantBackend>();
		_force_zero_motors = true;

	} else {
		PX4_WARN("unknown backend %d, fallback to zero", backend_type);
		_controller = std::make_unique<ZeroBackend>();
		_force_zero_motors = false;
	}

	if (!_controller) {
		PX4_ERR("backend alloc failed");
		return false;
	}

	if (!_controller->init()) {
		PX4_ERR("backend init failed");
		return false;
	}

	PX4_INFO("backend: %s", _controller->name());
	return true;
}

void ExperimentalControl::Run()
{
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();

		if (_sent_mode_registration) {
			unregister_flight_mode(_arming_check_id, _mode_id);
		}

		exit_and_cleanup();
		return;
	}

	if (!_sent_mode_registration) {
		register_flight_mode();
		_sent_mode_registration = true;
		return;
	}

	if (_mode_id == -1 || _arming_check_id == -1) {
		check_mode_registration();
		return;
	}

	perf_begin(_loop_perf);

	if (_arming_check_request_sub.updated()) {
		arming_check_request_s arming_check_request;
		_arming_check_request_sub.copy(&arming_check_request);
		reply_to_arming_check(arming_check_request.request_id);
	}

	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&vehicle_status);
		const bool in_custom_mode = vehicle_status.nav_state == _mode_id;
		const bool in_mission = vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		const bool in_hold = vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		_use_controller = in_custom_mode || in_mission || in_hold;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		_controller_ready = setup_controller_backend();
	}

	if (!_use_controller || !_controller_ready) {
		perf_end(_loop_perf);
		return;
	}

	if (_angular_velocity_sub.update(&_angular_velocity)) {
		const float dt = math::constrain(((_angular_velocity.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = _angular_velocity.timestamp_sample;

		if (_attitude_sub.updated()) {
			_attitude_sub.copy(&_attitude);
		}

		if (_position_sub.updated()) {
			_position_sub.copy(&_position);

			if (!PX4_ISFINITE(_trajectory_setpoint.position[0])
			    && !PX4_ISFINITE(_trajectory_setpoint.position[1])
			    && !PX4_ISFINITE(_trajectory_setpoint.position[2])) {
				reset_trajectory_setpoint(_position);
			}
		}

		if (_param_manual_control.get()) {
			_manual_control_setpoint_sub.update(&_manual_control_setpoint);
			check_setpoint_validity(_position);
			generate_trajectory_setpoint(dt);

		} else {
			if (_trajectory_setpoint_sub.updated()) {
				trajectory_setpoint_s trajectory_setpoint_temp;
				_trajectory_setpoint_sub.copy(&trajectory_setpoint_temp);

				if (PX4_ISFINITE(trajectory_setpoint_temp.position[0])
				    && PX4_ISFINITE(trajectory_setpoint_temp.position[1])
				    && PX4_ISFINITE(trajectory_setpoint_temp.position[2])) {
					_trajectory_setpoint = trajectory_setpoint_temp;
				}
			}
		}

		populate_inputs();

		const uint32_t start_time_us = now_us();
		bool ok = _controller && _controller->update(_input_data, _output_data, _last_inference_time_us);
		const uint32_t controller_time_us = now_us() - start_time_us;

		if (!ok) {
			PX4_ERR("controller update failed");
			perf_end(_loop_perf);
			return;
		}

		rescale_actions(_output_data, _force_zero_motors);
		publish_output(_output_data);
		publish_debug(_last_inference_time_us, controller_time_us);
	}

	perf_end(_loop_perf);
}

void ExperimentalControl::generate_trajectory_setpoint(float dt)
{
	float vx_sp = 0.0f;

	if (fabsf(_manual_control_setpoint.pitch) > 0.1f) {
		vx_sp = _manual_control_setpoint.pitch * 0.5f;
	}

	float vy_sp = 0.0f;

	if (fabsf(_manual_control_setpoint.roll) > 0.1f) {
		vy_sp = _manual_control_setpoint.roll * 0.5f;
	}

	float vz_sp = 0.0f;

	if (fabsf(_manual_control_setpoint.throttle) > 0.1f) {
		vz_sp = -_manual_control_setpoint.throttle * 0.5f;
	}

	matrix::Vector3f velocity_setpoint(vx_sp, vy_sp, vz_sp);
	const float yaw = matrix::Eulerf(matrix::Quatf(_attitude.q)).psi();
	matrix::Eulerf euler(0.0f, 0.0f, yaw);
	const matrix::Quatf q_yaw = euler;
	const matrix::Vector3f rotated_velocity_setpoint = q_yaw.rotateVector(velocity_setpoint);

	_trajectory_setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint.position[0] = _trajectory_setpoint.position[0] + rotated_velocity_setpoint(0) * dt;
	_trajectory_setpoint.position[1] = _trajectory_setpoint.position[1] + rotated_velocity_setpoint(1) * dt;
	_trajectory_setpoint.position[2] = _trajectory_setpoint.position[2] + rotated_velocity_setpoint(2) * dt;
}

void ExperimentalControl::reset_trajectory_setpoint(vehicle_local_position_s &position)
{
	_trajectory_setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint.position[0] = position.x;
	_trajectory_setpoint.position[1] = position.y;
	_trajectory_setpoint.position[2] = position.z;
}

void ExperimentalControl::check_setpoint_validity(vehicle_local_position_s &position)
{
	const float setpoint_age = (hrt_absolute_time() - _trajectory_setpoint.timestamp) * 1e-6f;

	if (setpoint_age < 0.0f || setpoint_age > 1.0f) {
		reset_trajectory_setpoint(position);
		PX4_INFO("Age: %.2f s, resetting trajectory setpoint to current position", (double)setpoint_age);
	}
}

void ExperimentalControl::populate_inputs()
{
	matrix::Dcmf frame_transf;
	frame_transf(0, 0) = 1.0f;
	frame_transf(0, 1) = 0.0f;
	frame_transf(0, 2) = 0.0f;
	frame_transf(1, 0) = 0.0f;
	frame_transf(1, 1) = -1.0f;
	frame_transf(1, 2) = 0.0f;
	frame_transf(2, 0) = 0.0f;
	frame_transf(2, 1) = 0.0f;
	frame_transf(2, 2) = -1.0f;

	matrix::Dcmf frame_transf_2;
	frame_transf_2(0, 0) = 0.0f;
	frame_transf_2(0, 1) = 1.0f;
	frame_transf_2(0, 2) = 0.0f;
	frame_transf_2(1, 0) = -1.0f;
	frame_transf_2(1, 1) = 0.0f;
	frame_transf_2(1, 2) = 0.0f;
	frame_transf_2(2, 0) = 0.0f;
	frame_transf_2(2, 1) = 0.0f;
	frame_transf_2(2, 2) = 1.0f;

	_trajectory_setpoint.position[0] = PX4_ISFINITE(_trajectory_setpoint.position[0]) ? _trajectory_setpoint.position[0] : 0.0f;
	_trajectory_setpoint.position[1] = PX4_ISFINITE(_trajectory_setpoint.position[1]) ? _trajectory_setpoint.position[1] : 0.0f;
	_trajectory_setpoint.position[2] = PX4_ISFINITE(_trajectory_setpoint.position[2]) ? _trajectory_setpoint.position[2] : -1.0f;

	matrix::Vector3f position_local = matrix::Vector3f(_position.x, _position.y, _position.z);
	position_local = frame_transf * frame_transf_2 * position_local;

	matrix::Vector3f trajectory_setpoint_local = matrix::Vector3f(_trajectory_setpoint.position[0],
			_trajectory_setpoint.position[1], _trajectory_setpoint.position[2]);
	trajectory_setpoint_local = frame_transf * frame_transf_2 * trajectory_setpoint_local;

	matrix::Vector3f linear_velocity_local = matrix::Vector3f(_position.vx, _position.vy, _position.vz);
	linear_velocity_local = frame_transf * frame_transf_2 * linear_velocity_local;

	matrix::Quatf attitude = matrix::Quatf(_attitude.q);
	matrix::Dcmf attitude_local_mat = frame_transf * (frame_transf_2 * matrix::Dcmf(attitude)) * frame_transf.transpose();

	matrix::Vector3f angular_vel_local = matrix::Vector3f(_angular_velocity.xyz[0], _angular_velocity.xyz[1],
					     _angular_velocity.xyz[2]);
	angular_vel_local = frame_transf * angular_vel_local;

	_input_data[0] = trajectory_setpoint_local(0) - position_local(0);
	_input_data[1] = trajectory_setpoint_local(1) - position_local(1);
	_input_data[2] = trajectory_setpoint_local(2) - position_local(2);
	_input_data[3] = attitude_local_mat(0, 0);
	_input_data[4] = attitude_local_mat(0, 1);
	_input_data[5] = attitude_local_mat(0, 2);
	_input_data[6] = attitude_local_mat(1, 0);
	_input_data[7] = attitude_local_mat(1, 1);
	_input_data[8] = attitude_local_mat(1, 2);
	_input_data[9] = linear_velocity_local(0);
	_input_data[10] = linear_velocity_local(1);
	_input_data[11] = linear_velocity_local(2);
	_input_data[12] = angular_vel_local(0);
	_input_data[13] = angular_vel_local(1);
	_input_data[14] = angular_vel_local(2);
}

void ExperimentalControl::rescale_actions(float *actions, bool force_zero_motors)
{
	const float thrust_coeff = _param_thrust_coeff.get() / 100000.0f;
	const float min_rpm = _param_min_rpm.get();
	const float max_rpm = _param_max_rpm.get();
	const float a = 0.8f;
	const float b = (1.0f - 0.8f);
	const float tmp1 = b / (2.f * a);
	const float tmp2 = b * b / (4.f * a * a);

	for (int i = 0; i < 3; i++) {
		actions[i] = math::constrain(actions[i], -1.0f, 1.0f);
	}

	for (int i = 3; i < 6; i++) {
		if (force_zero_motors) {
			actions[i] = 0.0f;
			continue;
		}

		actions[i] = math::constrain(actions[i], -1.0f, 1.0f);

		actions[i] = actions[i] + 1.0f;
		float rps = actions[i] / thrust_coeff;
		rps = sqrtf(rps);
		const float rpm = rps * 60.0f;
		actions[i] = (rpm * 2.0f - max_rpm - min_rpm) / (max_rpm - min_rpm);
		actions[i] = a * (((actions[i] + 1.0f) / 2.0f + tmp1) * ((actions[i] + 1.0f) / 2.0f + tmp1) - tmp2);
	}
}

void ExperimentalControl::publish_output(float *command_actions)
{
	actuator_servos_s actuator_servos{};
	actuator_servos.timestamp = hrt_absolute_time();
	actuator_servos.timestamp_sample = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		actuator_servos.control[i] = PX4_ISFINITE(command_actions[i]) ? command_actions[i] : NAN;
	}

	for (int i = 3; i < actuator_servos_s::NUM_CONTROLS; i++) {
		actuator_servos.control[i] = NAN;
	}

	_actuator_servos_pub.publish(actuator_servos);

	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		float motor_value = PX4_ISFINITE(command_actions[i + 3]) ? command_actions[i + 3] : NAN;

		if (PX4_ISFINITE(motor_value) && motor_value < 0.0f) {
			motor_value = 0.0f;
		}

		actuator_motors.control[i] = motor_value;
	}

	for (int i = 3; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	actuator_motors.reversible_flags = 0;
	_actuator_motors_pub.publish(actuator_motors);
}

void ExperimentalControl::publish_debug(uint32_t inference_time_us, uint32_t controller_time_us)
{
	experimental_control_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.inference_time = inference_time_us;
	status.controller_time = controller_time_us;

	for (int i = 0; i < 15; i++) {
		status.observation[i] = _input_data[i];
	}

	for (int i = 0; i < 6; i++) {
		status.network_output[i] = _output_data[i];
	}

	_exp_ctrl_status_pub.publish(status);
}

void ExperimentalControl::register_flight_mode()
{
	register_ext_component_request_s register_ext_component_request{};
	register_ext_component_request.timestamp = hrt_absolute_time();
	strncpy(register_ext_component_request.name, "Experimental Control", sizeof(register_ext_component_request.name) - 1);
	register_ext_component_request.request_id = _mode_request_id;
	register_ext_component_request.px4_ros2_api_version = 1;
	register_ext_component_request.register_arming_check = true;
	register_ext_component_request.register_mode = true;
	_register_ext_component_request_pub.publish(register_ext_component_request);
}

void ExperimentalControl::unregister_flight_mode(int8_t arming_check_id, int8_t mode_id)
{
	unregister_ext_component_s unregister_ext_component{};
	unregister_ext_component.timestamp = hrt_absolute_time();
	strncpy(unregister_ext_component.name, "Experimental Control", sizeof(unregister_ext_component.name) - 1);
	unregister_ext_component.arming_check_id = arming_check_id;
	unregister_ext_component.mode_id = mode_id;
	_unregister_ext_component_pub.publish(unregister_ext_component);
}

void ExperimentalControl::configure_flight_mode(int8_t mode_id)
{
	vehicle_control_mode_s config_control_setpoints{};
	config_control_setpoints.timestamp = hrt_absolute_time();
	config_control_setpoints.source_id = mode_id;
	config_control_setpoints.flag_multicopter_position_control_enabled = false;
	config_control_setpoints.flag_control_manual_enabled = _param_manual_control.get();
	config_control_setpoints.flag_control_offboard_enabled = false;
	config_control_setpoints.flag_control_position_enabled = false;
	config_control_setpoints.flag_control_climb_rate_enabled = true;
	config_control_setpoints.flag_control_allocation_enabled = false;
	config_control_setpoints.flag_control_termination_enabled = true;
	_config_control_setpoints_pub.publish(config_control_setpoints);
}

void ExperimentalControl::reply_to_arming_check(int8_t request_id)
{
	arming_check_reply_s arming_check_reply{};
	arming_check_reply.timestamp = hrt_absolute_time();
	arming_check_reply.request_id = request_id;
	arming_check_reply.registration_id = _arming_check_id;
	arming_check_reply.health_component_index = arming_check_reply.HEALTH_COMPONENT_INDEX_NONE;
	arming_check_reply.num_events = 0;
	arming_check_reply.can_arm_and_run = true;
	arming_check_reply.mode_req_angular_velocity = true;
	arming_check_reply.mode_req_local_position = true;
	arming_check_reply.mode_req_attitude = true;
	arming_check_reply.mode_req_local_alt = true;
	arming_check_reply.mode_req_home_position = false;
	arming_check_reply.mode_req_mission = false;
	arming_check_reply.mode_req_global_position = false;
	arming_check_reply.mode_req_prevent_arming = false;
	arming_check_reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(arming_check_reply);
}

void ExperimentalControl::check_mode_registration()
{
	register_ext_component_reply_s register_ext_component_reply{};
	int tries = register_ext_component_reply.ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&register_ext_component_reply) && --tries >= 0) {
		if (register_ext_component_reply.request_id == _mode_request_id && register_ext_component_reply.success) {
			_arming_check_id = register_ext_component_reply.arming_check_id;
			_mode_id = register_ext_component_reply.mode_id;
			PX4_INFO("Experimental control mode registration ok, arming_check_id: %d, mode_id: %d", _arming_check_id,
				 _mode_id);
			configure_flight_mode(_mode_id);
			break;
		}
	}
}

uint32_t ExperimentalControl::now_us() const
{
	return static_cast<uint32_t>(hrt_absolute_time());
}

int ExperimentalControl::task_spawn(int argc, char *argv[])
{
	ExperimentalControl *instance = new ExperimentalControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;

		} else {
			PX4_ERR("init failed");
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ExperimentalControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ExperimentalControl::print_status()
{
	if (_mode_id == -1) {
		PX4_INFO("Experimental control flight mode: Mode registration pending");
		PX4_INFO("Request sent: %d", _sent_mode_registration);

	} else {
		PX4_INFO("Experimental control flight mode: Registered, mode id: %d, arming check id: %d", _mode_id,
			 _arming_check_id);
	}

	PX4_INFO("backend: %s", _controller ? _controller->name() : "none");
	return 0;
}

int ExperimentalControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Experimental end-to-end control module for multicopters. It demonstrates a plug-in style
controller backend (default is a zero-output stub) and publishes actuator outputs directly.
Inputs: [pos_err(3), att(6), vel(3), ang_vel(3)]
Outputs: [Servo controls(3), Motor controls(3)]
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("experimental_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int experimental_control_main(int argc, char *argv[])
{
	return ExperimentalControl::main(argc, argv);
}
