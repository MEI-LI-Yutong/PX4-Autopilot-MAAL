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

protected:
	ActuatorVector _tilt_offsets;
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;
	int _first_tilt_idx{0};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos;
		bool tilt_yaw_neg;
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};

private:
	// ===== Post-allocation linear Fx layer (equal Δθ on forward-tilting servos) =====
	static constexpr float MAX_LINEAR_TILT_RAD = 0.30f; // ~17 deg
	static constexpr float EPS_F = 1e-6f;

	// Fx residual to inject into control_allocator_status
	float _fx_residual{0.f};

	// 用于log记录的详细变量
	struct TiltLogData {
		float fx_cmd{0.f};          // 输入的Fx指令
		float fz_cmd{0.f};          // 输入的Fz指令 (thrust_body[2])
		float f_total{0.f};         // 前倾舵机控制电机的总推力
		float delta_theta{0.f};     // 计算的角度增量 [rad]
		float theta_fl_old{0.f};    // 前左舵机原始角度 [rad]
		float theta_fr_old{0.f};    // 前右舵机原始角度 [rad]
		float theta_tail_old{0.f};  // 尾部舵机原始角度 [rad]
		float theta_fl_new{0.f};    // 前左舵机最终角度 [rad]
		float theta_fr_new{0.f};    // 前右舵机最终角度 [rad]
		float theta_tail_new{0.f};  // 尾部舵机最终角度 [rad]
		float servo_fl_old{0.f};    // 前左舵机原始归一化值 [-1,1]
		float servo_fr_old{0.f};    // 前右舵机原始归一化值 [-1,1]
		float servo_tail_old{0.f};  // 尾部舵机原始归一化值 [-1,1]
		float servo_fl_new{0.f};    // 前左舵机最终归一化值 [-1,1]
		float servo_fr_new{0.f};    // 前右舵机最终归一化值 [-1,1]
		float servo_tail_new{0.f};  // 尾部舵机最终归一化值 [-1,1]
		float fx_real{0.f};         // 实际产生的Fx
		int num_motors{0};          // 参与计算的电机数量
	} _tilt_log_data;

	// Helpers: angle [rad] <-> servo normalized [-1, 1]
	float angleToServoNormalized(int tilt_index, float theta) const;
	float servoNormalizedToAngle(int tilt_index, float normalized) const;
};
