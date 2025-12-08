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
 * @file mc_custom_control.hpp
 * Minimal unified multicopter control module that bundles sensor inputs and
 * raw actuator outputs into a single WorkItem.
 */

#pragma once

#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <lib/rate_control/rate_control.hpp>
#include <modules/mc_att_control/AttitudeControl/AttitudeControl.hpp>
#include <modules/mc_pos_control/PositionControl/PositionControl.hpp>
#include <modules/mc_pos_control/Takeoff/Takeoff.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

using namespace time_literals;

class MulticopterCustomControl : public ModuleBase<MulticopterCustomControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterCustomControl();
	~MulticopterCustomControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	void Run() override;

	void ComputeOutputs(float dt, float servo_out[3], float motor_out[3]);
	void PublishServos(const float servo_out[3], hrt_abstime now);
	void PublishMotors(const float motor_out[3], hrt_abstime now);
	void ParametersUpdated();
	void UpdatePositionControlGains();
	void UpdateAttitudeControlGains();
	void UpdateRateControlGains();
	bool RunPositionControl(float dt, vehicle_attitude_setpoint_s &att_sp, matrix::Vector3f &thrust_sp);

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	perf_counter_t _loop_perf{nullptr};
	hrt_abstime _last_run{0};
	hrt_abstime _last_servo_publish{0};
	float _last_servo_values[actuator_servos_s::NUM_CONTROLS]{};

	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_attitude_s _attitude{};
	vehicle_local_position_s _position{};
	trajectory_setpoint_s _trajectory_setpoint{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_constraints_s _vehicle_constraints{};
	hover_thrust_estimate_s _hover_thrust_estimate{};

	PositionControl _position_control;
	TakeoffHandling _takeoff;
	AttitudeControl _attitude_control;
	RateControl _rate_control;

	matrix::Vector3f _rates_sp{};
	matrix::Vector3f _thrust_sp{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>) _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>) _param_mc_yaw_weight,
		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _param_mc_yawrate_max,

		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,

		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>) _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>) _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>) _param_mpc_z_vel_d_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_THR_MIN>) _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) _param_mpc_tiltmax_lnd,
		(ParamFloat<px4::params::MPC_THR_XY_MARG>) _param_mpc_thr_xy_marg,
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _param_mpc_tko_ramp_t,
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time
	)
};
