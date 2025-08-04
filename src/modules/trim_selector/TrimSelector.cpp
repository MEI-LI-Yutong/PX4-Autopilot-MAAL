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

#include "TrimSelector.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>

#include <mathlib/mathlib.h>
#include <geo/geo.h>

using namespace matrix;

TrimSelector::TrimSelector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化参数更新
	parameters_update(true);
}

TrimSelector::~TrimSelector()
{
	// 析构函数，清理资源（如果有需要）
}

bool TrimSelector::init()
{
	// 初始化定时器，定期执行Run()函数
	ScheduleOnInterval(10_ms); // 100Hz 更新率，与 position control 相同
	PX4_INFO("Trim Selector module initialized and started");
	return true;
}

void TrimSelector::parameters_update(bool force)
{
	// 检查参数更新
	if (_parameter_update_sub.updated() || force) {
		// 清除更新标志
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从存储中更新参数
		updateParams();
	}
}

void TrimSelector::Run()
{
	// 首先检查参数是否需要更新
	parameters_update();

	// 订阅轨迹设定点
	trajectory_setpoint_s trajectory_setpoint{};
	bool has_trajectory_data = _trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// 添加调试信息（每5秒打印一次状态）
	static uint64_t last_status_time = 0;
	uint64_t now = hrt_absolute_time();
	if (now - last_status_time > 5000000) { // 5秒
		if (!has_trajectory_data) {
			PX4_WARN("Trim Selector: No trajectory_setpoint data - using defaults");
		} else if (!PX4_ISFINITE(trajectory_setpoint.velocity[0]) || !PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
			PX4_WARN("Trim Selector: Invalid velocity data - using defaults");
		} else {
			PX4_INFO("Trim Selector: Valid trajectory data available");
		}
		last_status_time = now;
	}

	// 初始化默认值
	float horizontal_velocity_magnitude = 0.0f;
	float pitch_setpoint = 0.0f;
	bool data_valid = false;

	// 检查是否有有效的轨迹数据
	if (has_trajectory_data && PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
		// 计算水平速度大小
		horizontal_velocity_magnitude = sqrtf(matrix::Vector2f(trajectory_setpoint.velocity).norm_squared());
		// 使用参数作为增益
		pitch_setpoint = _param_ts_pitch_gain.get() * horizontal_velocity_magnitude;
		data_valid = true;
	}
	// 如果没有有效数据，使用默认值（已经初始化为0）

	// 创建并发布 utrim 消息（始终发布）
	utrim_s utrim{};
	utrim.timestamp = hrt_absolute_time();
	utrim.horizontal_velocity = horizontal_velocity_magnitude;
	utrim.valid = data_valid;

	if (data_valid) {
		// 有效数据：基于实际速度和俯仰角计算
		utrim.polynomial_values[0] = 5.0f + horizontal_velocity_magnitude * 2.0f;  // f1
		utrim.polynomial_values[1] = 5.0f + horizontal_velocity_magnitude * 2.0f;  // f2
		utrim.polynomial_values[2] = 8.0f + horizontal_velocity_magnitude * 1.5f;  // f3

		float pitch_degrees = pitch_setpoint * 180.0f / M_PI_F;
		utrim.polynomial_values[3] = pitch_degrees * 0.5f;  // theta1
		utrim.polynomial_values[4] = pitch_degrees * 0.5f;  // theta2
		utrim.polynomial_values[5] = pitch_degrees * 0.8f;  // theta3
	} else {
		// 默认值：基于重力和质量的安全参数
		// m = 2.8 kg, g = CONSTANTS_ONE_G (9.80665 m/s^2)
		// 每个电机承担 m*g/3 的推力
		const float mass = 2.8f;  // 质量 [kg]
		const float thrust_per_motor = mass * CONSTANTS_ONE_G / 3.0f;  // 每个电机的推力 [N]

		utrim.polynomial_values[0] = thrust_per_motor;  // f1 = m*g/3
		utrim.polynomial_values[1] = thrust_per_motor;  // f2 = m*g/3
		utrim.polynomial_values[2] = thrust_per_motor;  // f3 = m*g/3
		utrim.polynomial_values[3] = 0.0f;  // theta1 默认值
		utrim.polynomial_values[4] = 0.0f;  // theta2 默认值
		utrim.polynomial_values[5] = 0.0f;  // theta3 默认值

		// 在第一次使用默认值时打印计算信息
		static bool first_default_log = true;
		if (first_default_log) {
			PX4_INFO("Trim Selector: Using calculated defaults - mass=%.1f kg, g=%.3f m/s², thrust_per_motor=%.3f N",
				(double)mass, (double)CONSTANTS_ONE_G, (double)thrust_per_motor);
			first_default_log = false;
		}
	}

	// 发布 utrim（始终发布）
	_utrim_pub.publish(utrim);

	// 创建并发布 theta_trim 消息（始终发布）
	theta_trim_s theta_trim{};
	theta_trim.timestamp = hrt_absolute_time();

	if (data_valid) {
		// 有效数据：基于计算的俯仰角
		theta_trim.pitch_angle = pitch_setpoint * 180.0f / M_PI_F;
	} else {
		// 默认值：零俯仰角
		theta_trim.pitch_angle = 0.0f;
	}

	// 发布 theta_trim（始终发布）
	_theta_trim_pub.publish(theta_trim);

	// 添加调试信息（每秒只打印一次，避免日志刷屏）
	static uint64_t last_log_time = 0;
	if (now - last_log_time > 1000000) { // 1秒
		if (data_valid) {
			PX4_INFO("Trim Selector: vel_mag=%.3f, pitch=%.3f deg [VALID]",
				(double)horizontal_velocity_magnitude, (double)(pitch_setpoint * 180.0f / M_PI_F));
		} else {
			PX4_INFO("Trim Selector: Using DEFAULT values [INVALID DATA]");
		}

		// 记录 utrim 发布数据
		PX4_INFO("Published utrim: f1=%.3f, f2=%.3f, f3=%.3f, θ1=%.3f°, θ2=%.3f°, θ3=%.3f°, valid=%s",
			(double)utrim.polynomial_values[0], (double)utrim.polynomial_values[1], (double)utrim.polynomial_values[2],
			(double)utrim.polynomial_values[3], (double)utrim.polynomial_values[4], (double)utrim.polynomial_values[5],
			utrim.valid ? "true" : "false");

		// 记录 theta_trim 发布数据
		PX4_INFO("Published theta_trim: pitch=%.3f°",
			(double)theta_trim.pitch_angle);

		last_log_time = now;
	}
}

int TrimSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
Trim Selector模块接收轨迹设定点，计算姿态设定点和控制分配参数，并将其发布到uORB。
特别是，它将计算俯仰角设定点，等于速度的2倍（theta = 2 * v_x），
同时发布utrim消息用于自定义控制分配。

### 实现
该模块订阅轨迹设定点，计算所需的姿态，并发布姿态设定点和控制分配参数。
发布的消息包括：
- theta_trim: 姿态设定点（四元数、推力、俯仰角）
- utrim: 控制分配参数（6个多项式值：f1,f2,f3,θ1,θ2,θ3）
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trim_selector", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TrimSelector::task_spawn(int argc, char *argv[])
{
	PX4_INFO("Starting Trim Selector module...");
	TrimSelector *instance = new TrimSelector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			PX4_INFO("Trim Selector module started successfully");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	PX4_ERR("Trim Selector module start failed");
	return PX4_ERROR;
}

int TrimSelector::custom_command(int argc, char *argv[])
{
	return print_usage("不支持的命令");
}

extern "C" __EXPORT int trim_selector_main(int argc, char *argv[])
{
	return TrimSelector::main(argc, argv);
}
