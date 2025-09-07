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
#include <lib/perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/theta_trim.h>
#include <uORB/topics/utrim.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/trim_selector_status.h>

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
	float calculate_polynomial(float x, int polynomial_index);

	// 起飞/降落斜坡更新
	void update_takeoff_land_ramp(float dt);

	// 更新阵风估计和抗风滑移系数
	void update_gust_estimation(float dt);

	// 计算名义配平（不含 ramp），返回是否有有效轨迹数据
	bool compute_nominal_trim(float &f1, float &f2, float &f3,
	                          float &theta1_deg, float &theta2_deg, float &theta3_deg,
	                          float &horizontal_velocity_magnitude, bool &data_valid);

	// Publications
	uORB::Publication<theta_trim_s> _theta_trim_pub{ORB_ID(theta_trim)};
	uORB::Publication<utrim_s> _utrim_pub{ORB_ID(utrim)};
	uORB::Publication<log_message_s> _log_message_pub{ORB_ID(log_message)};
	uORB::Publication<wind_s> _wind_pub{ORB_ID(wind)};
	uORB::Publication<trim_selector_status_s> _trim_selector_status_pub{ORB_ID(trim_selector_status)};

	// Subscriptions
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _manual_sp_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	perf_counter_t _loop_perf{nullptr};      ///< 循环性能计数器
	log_message_s _log_message{};            ///< 日志消息

	// Params
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TS_PITCH_GAIN>) _param_ts_pitch_gain,    // 原有：俯仰角增益
		(ParamFloat<px4::params::TS_RAMP_T_UP>)  _param_ts_ramp_t_up,     // 起飞斜坡时间（s）
		(ParamFloat<px4::params::TS_RAMP_T_DN>)  _param_ts_ramp_t_dn,     // 降落斜坡时间（s）
		(ParamFloat<px4::params::TS_THR_TKO>)    _param_ts_thr_tko,       // 手动模式起飞油门阈值 [0..1]
		(ParamFloat<px4::params::TS_S_LAND>)     _param_ts_s_land,        // 降落阶段目标 s
		(ParamFloat<px4::params::TS_MASS>)       _param_ts_mass,         // 机体质量 [kg]
		(ParamInt<px4::params::TS_RAMP_EN>)      _param_ts_ramp_en        // 斜坡开关（1 启用）
	)

	// Ramp state
	float _s{0.f};                // 当前 ramp 因子 [0..1]
	float _s_target{0.f};         // 目标 ramp 因子
	hrt_abstime _last_run{0};     // 上次运行时间

	// 阵风估计相关
	static constexpr float GUST_FILTER_TC = 0.318f;     // 0.5Hz 低通滤波时间常数
	static constexpr float GUST_K_MIN = 3.0f;        // k映射最小阈值 (m/s)
	static constexpr float GUST_K_MAX = 10.0f;       // k映射最大阈值 (m/s)
	float _gust_raw{0.0f};        // 原始阵风值
	float _gust_filt{0.0f};       // 滤波后阵风值
	float _antiwind_k{0.0f};      // 抗风滑移系数 [0,1]
};
