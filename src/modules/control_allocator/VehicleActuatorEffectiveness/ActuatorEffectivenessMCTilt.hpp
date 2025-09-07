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

    // 新增：设置抗风滑移系数 k ∈ [0,1]
    void setAntiWindBlendK(float k) { _antiwind_blend_k = math::constrain(k, 0.f, 1.f); }

    bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

    void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
    {
        allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
    }

    void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
    {
        normalize[0] = true;
    }

    void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
                const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

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
    // ===== 后处理 Fx 层（等角Δθ）+ 抗风零空间推进（尾部 φ 固定负向，前排等量 ψ） =====
    static constexpr float MAX_LINEAR_TILT_RAD = 0.30f;       // 线性小角域 ~17°
    static constexpr float EPS_F = 1e-6f;
    static constexpr float TAIL_Y_EPS = 0.05f;                // |y|<阈值视为尾部
    static constexpr float ANTIWIND_PHI_RAD = -10.f * M_PI_F / 180.f; // φ 缺省为 -10°（负向、反 Fx）
    static constexpr bool  ENABLE_FX_LAYER = true;
    static constexpr bool  ENABLE_ANTIWIND = true;

    // Fx 残差（用于上报到 control_allocator_status）
    float _fx_residual{0.f};

    // 抗风滑移系数 k ∈ [0,1]（来自 TrimSelector）
    float _antiwind_blend_k{0.5f};

    // 角度 <-> 归一化映射
    float angleToServoNormalized(int tilt_index, float theta) const;
    float servoNormalizedToAngle(int tilt_index, float normalized) const;
};