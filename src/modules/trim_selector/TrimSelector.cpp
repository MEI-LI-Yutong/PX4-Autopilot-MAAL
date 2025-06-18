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
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// 计算俯仰角期望
	// 根据需求，theta = gain * sqrt(v_x^2 + v_y^2)
	if (PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
		// 计算水平速度大小
		float horizontal_velocity_magnitude = sqrtf(matrix::Vector2f(trajectory_setpoint.velocity).norm_squared());

		// 使用参数作为增益
		float pitch_setpoint = _param_ts_pitch_gain.get() * horizontal_velocity_magnitude;

		// 创建 theta_trim 消息
		theta_trim_s theta_trim{};
		theta_trim.timestamp = hrt_absolute_time();

		// 计算期望的四元数（基于俯仰角）
		// 注意：这里我们仅设置俯仰角，其他角度保持为零
		Eulerf euler_setpoint(0.0f, -pitch_setpoint, 0.0f); // 负号是因为俯仰角的定义（向前为负）
		Quatf q_sp = Quatf(euler_setpoint);
		q_sp.copyTo(theta_trim.q_d);

		// 设置推力（这里简单设置为向下的推力，实际应用中可能需要更复杂的计算）
		theta_trim.thrust_body[0] = 0.0f;
		theta_trim.thrust_body[1] = 0.0f;
		theta_trim.thrust_body[2] = -0.5f; // 垂直向下的推力，实际应用中需要调整

		// 发布 theta_trim
		_theta_trim_pub.publish(theta_trim);
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
Trim Selector模块接收轨迹设定点，计算姿态设定点，并将其发布到uORB。
特别是，它将计算俯仰角设定点，等于速度的2倍（theta = 2 * v_x）。

### 实现
该模块订阅轨迹设定点，计算所需的姿态，并发布姿态设定点。
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trim_selector", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TrimSelector::task_spawn(int argc, char *argv[])
{
	TrimSelector *instance = new TrimSelector();

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

int TrimSelector::custom_command(int argc, char *argv[])
{
	return print_usage("不支持的命令");
}

extern "C" __EXPORT int trim_selector_main(int argc, char *argv[])
{
	return TrimSelector::main(argc, argv);
}
