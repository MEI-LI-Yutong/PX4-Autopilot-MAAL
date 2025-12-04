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

/**
 * @file experimental_control.hpp
 * Experimental end-to-end control module to host pluggable controllers.
 *
 * The module subscribes to sensor data and setpoints, runs a controller backend
 * (placeholder backend by default), and publishes actuator outputs. It mirrors
 * the structure of the mc_nn_control prototype to make it easy to swap in
 * different controller implementations, including NN-based ones.
 */

#pragma once

#include <perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/experimental_control_status.h>

#include <cstdint>
#include <memory>

#define MODULE_NAME "experimental_control"

using namespace time_literals;

class ControllerBackend
{
public:
	virtual ~ControllerBackend() = default;
	virtual const char *name() const = 0;
	virtual bool init() = 0;
	virtual bool update(const float input[15], float output[6], uint32_t &duration_us) = 0;
};

class ExperimentalControl : public ModuleBase<ExperimentalControl>, public ModuleParams, public px4::WorkItem
{
public:
	ExperimentalControl();
	~ExperimentalControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:
	void Run() override;

	bool setup_controller_backend();
	void generate_trajectory_setpoint(float dt);
	void reset_trajectory_setpoint(vehicle_local_position_s &position);
	void check_setpoint_validity(vehicle_local_position_s &position);
	void populate_inputs();

	void rescale_actions(float *actions, bool force_zero_motors);
	void publish_output(float *command_actions);
	void publish_debug(uint32_t inference_time_us, uint32_t controller_time_us);

	void register_flight_mode();
	void unregister_flight_mode(int8_t arming_check_id, int8_t mode_id);
	void configure_flight_mode(int8_t mode_id);
	void reply_to_arming_check(int8_t request_id);
	void check_mode_registration();

	uint32_t now_us() const;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<experimental_control_status_s> _exp_ctrl_status_pub{ORB_ID(experimental_control_status)};
	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};

	bool _use_controller{false};
	bool _sent_mode_registration{false};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	hrt_abstime _last_run{0};
	uint8_t _mode_request_id{231};
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};

	float _input_data[15]{};
	float _output_data[6]{};
	bool _force_zero_motors{false};

	trajectory_setpoint_s _trajectory_setpoint{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_local_position_s _position{};
	vehicle_attitude_s _attitude{};
	manual_control_setpoint_s _manual_control_setpoint{};

	uint32_t _last_inference_time_us{0};

	std::unique_ptr<ControllerBackend> _controller;
	bool _controller_ready{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_NN_MAX_RPM>) _param_max_rpm,
		(ParamInt<px4::params::MC_NN_MIN_RPM>) _param_min_rpm,
		(ParamFloat<px4::params::MC_NN_THRST_COEF>) _param_thrust_coeff,
		(ParamInt<px4::params::MC_NN_MANL_CTRL>) _param_manual_control,
		(ParamFloat<px4::params::MC_NN_SV_MAX_ANG>) _param_servo_max_angle,
		(ParamFloat<px4::params::MC_NN_SV_MIN_ANG>) _param_servo_min_angle,
		(ParamInt<px4::params::EXP_CTRL_BACKEND>) _param_backend_type
	)
};
