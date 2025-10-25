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

/**
 * @file trim_selector_params.c
 * Trim Selector 模块参数
 *
 * @author PX4 Development Team
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/param/param.h>

/**
 * Trim Selector 俯仰角增益
 *
 * @min 0.0
 * @max 10.0
 * @increment 0.1
 * @decimal 1
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_PITCH_GAIN, 2.0f);

/**
 * Trim Selector 起飞斜坡时间
 *
 * @min 0.1
 * @max 10.0
 * @increment 0.1
 * @decimal 1
 * @unit s
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_RAMP_T_UP, 1.0f);

/**
 * Trim Selector 降落斜坡时间
 *
 * @min 0.1
 * @max 10.0
 * @increment 0.1
 * @decimal 1
 * @unit s
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_RAMP_T_DN, 1.0f);

/**
 * Trim Selector 手动起飞油门阈值
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 2
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_THR_TKO, 0.15f);

/**
 * Trim Selector 降落阶段配平因子
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 2
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_S_LAND, 0.15f);

/**
 * Trim Selector 斜坡功能使能
 *
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Enabled
 * @group Trim Selector
 */
PARAM_DEFINE_INT32(TS_RAMP_EN, 1);

/**
 * Trim Selector 机体质量
 *
 * @min 0.1
 * @max 50.0
 * @increment 0.1
 * @decimal 1
 * @unit kg
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_MASS, 2.8f);