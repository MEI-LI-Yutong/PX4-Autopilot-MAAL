/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/theta_trim.h>
#include <uORB/topics/utrim.h>
// 新增 uORB 话题
#include <uORB/topics/airspeed_wind.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
// 平滑起降所需话题
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>

using namespace time_literals;

class TrimSelector : public ModuleBase<TrimSelector>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TrimSelector();
	~TrimSelector() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_update(bool force = false);

	uORB::Publication<theta_trim_s> _theta_trim_pub{ORB_ID(theta_trim)};
	uORB::Publication<utrim_s> _utrim_pub{ORB_ID(utrim)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // === 新增订阅 ===
    uORB::Subscription _airspeed_wind_sub{ORB_ID(airspeed_wind)};
    uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

    // === 获取风速大小的辅助函数 ===
    float get_wind_magnitude();

    // === 平滑起降：内部状态 ===
    uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

    enum class RampPhase : uint8_t {
        Idle = 0,
        TakeoffRamp,
        Flight,
        LandingRamp
    };

    RampPhase _ramp_phase{RampPhase::Idle};
    float _utrim_alpha{0.f};               // 0..1，用于缩放 f1..f3
    hrt_abstime _ramp_start_time{0};

    // 默认斜坡时间（秒）。如需参数化，可后续加入参数系统
    static constexpr float _takeoff_ramp_time_s{1.5f};
    static constexpr float _landing_ramp_time_s{2.0f};

    // 油门阈值：用于判断起飞/降落意图
    static constexpr float _takeoff_throttle_threshold{0.6f};  // 起飞油门阈值
    static constexpr float _landing_throttle_threshold{0.3f};  // 降落油门阈值

    // 飞机质量参数
    static constexpr float _vehicle_mass{2.8f};  // 质量 [kg]

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TS_PITCH_GAIN>) _param_ts_pitch_gain   /**< 俯仰角增益参数 */
	)
};
