/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/utrim.h>
#include <drivers/drv_hrt.h>

class ActuatorEffectivenessMCTilt : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessMCTilt(ModuleParams *parent);
	virtual ~ActuatorEffectivenessMCTilt() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "MC Tilt"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	// 动态更新标志（供 ControlAllocator 跳过 100ms 限制）
	bool dynamicUpdatePending() const override { return _dynamic_update_pending; }

	// 供 ControlAllocator 定期调用以检查 utrim 更新
	void checkForDynamicUpdates();

protected:
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;
	ActuatorVector _tilt_offsets;
	int _first_tilt_idx{0};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos{false};
		bool tilt_yaw_neg{false};
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};

	// Collective tilt control - rate limited state machine
	enum class RampState {
		NONE = 0,
		RAMP_TO_ZERO
	};

	uORB::Subscription _vt_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};

	// Current collective tilt state
	float _collective_tilt_angle{0.f};		// Current angle in radians
	float _collective_tilt_target{0.f};		// Target angle in radians
	bool _collective_tilt_valid{false};		// Whether incoming command is valid
	bool _collective_was_clipped{false};		// Whether collective was clipped this frame

	// State machine variables
	RampState _ramp_state{RampState::NONE};		// Current ramp state
	hrt_abstime _last_valid_command_time{0};	// Last time we received valid command
	hrt_abstime _last_update_time{0};		// Last time updateSetpoint was called

	// Rate limiting constants
	static constexpr float COLLECTIVE_TILT_RATE_MAX_RAD_S = 1.2f;	   // Normal rate limit: 1.2 rad/s
	static constexpr float COLLECTIVE_TILT_RATE_RETURN_RAD_S = 1.5f;   // Return-to-zero rate: 1.5 rad/s
	static constexpr hrt_abstime COLLECTIVE_TIMEOUT_US = 200000;	   // 200ms timeout

	// utrim & dynamic axis support for Fx thrust
	uORB::Subscription _utrim_sub{ORB_ID(utrim)};
	bool _dynamic_update_pending{false};
	hrt_abstime _last_dynamic_update{0};
	static constexpr hrt_abstime MIN_DYNAMIC_INTERVAL_US = 30000; // 30ms 节流
	float _last_collective_norm{NAN};
	bool _config_phase{true}; // 首次 CONFIGURATION_UPDATE 后保持一次竖直轴向

private:
	// 新增私有成员
	utrim_s _last_utrim{};   // 缓存最近一次 utrim
	bool    _have_utrim{false};

	/**
	 * Update collective tilt angle from vehicle thrust setpoint and handle state machine
	 */
	void updateCollectiveTiltAngle();

	/**
	 * Update collective tilt angle using rate limiting based on current state
	 * @param dt time step in seconds
	 */
	void updateCollectiveTiltWithRateLimit(float dt);

	/**
	 * Calculate yaw saturation flags based on yaw-only actuator values
	 * @param yaw_only_values actuator values before collective tilt is applied
	 * @param tilt_count actual number of tilts to process
	 * @param actuator_min minimum actuator limits
	 * @param actuator_max maximum actuator limits
	 */
	void calculateYawSaturationFlags(const float yaw_only_values[], int tilt_count,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max);

	/**
	 * Apply collective tilt control to forward-tilting servos with yaw priority
	 * @param actuator_sp actuator setpoints to modify
	 * @param yaw_only_values base actuator values before collective tilt
	 * @param tilt_count actual number of tilts to process
	 * @param actuator_min minimum actuator limits
	 * @param actuator_max maximum actuator limits
	 */
	void applyCollectiveTilt(ActuatorVector &actuator_sp, const float yaw_only_values[], int tilt_count,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max);

	/**
	 * Handle utrim message and update collective norm for dynamic axis updates
	 * @return true if utrim was processed and may trigger update
	 */
	bool handleUtrimMessage();

	/**
	 * Update rotor axis configuration from collective norm value
	 * @param collective_norm normalized collective tilt value [-1,1]
	 */
	void updateRotorAxisFromCollective(float collective_norm);

	/**
	 * Build actuator trim and control trim vectors from utrim data
	 * @param configuration effectiveness configuration to modify
	 */
	void buildActuatorTrimAndControlTrim(Configuration &configuration);
};
