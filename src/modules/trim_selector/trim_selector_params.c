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
 * Trim Selector俯仰角增益
 *
 * 该参数控制了从速度到俯仰角的转换增益。
 * 设置为2意味着俯仰角设定点等于2倍的前向速度（theta = 2 * v_x）
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_PITCH_GAIN, 2.0f);

/**
 * Takeoff ramp time
 *
 * Time for ramping utrim from 0 to 1 during takeoff.
 *
 * @unit s
 * @min 0.05
 * @max 5.0
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_RAMP_T_UP, 1.0f);

/**
 * Landing ramp time
 *
 * Time for ramping utrim down towards TS_S_LAND during landing.
 *
 * @unit s
 * @min 0.05
 * @max 5.0
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_RAMP_T_DN, 1.0f);

/**
 * Manual takeoff throttle threshold
 *
 * When landed and not in auto, ramp-up starts when throttle exceeds this threshold.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_THR_TKO, 0.15f);

/**
 * Landing target ramp factor
 *
 * Target utrim scaling during landing phase (0..1).
 *
 * @min 0.0
 * @max 1.0
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_S_LAND, 0.15f);

/**
 * Enable utrim ramping
 *
 * 0: disabled (utrim is not ramped), 1: enabled
 *
 * @boolean
 * @group Trim Selector
 */
PARAM_DEFINE_INT32(TS_RAMP_EN, 1);

/**
 * Vehicle mass for nominal trim
 *
 * Mass used to compute nominal thrust per motor.
 *
 * @unit kg
 * @min 0.1
 * @max 100
 * @decimal 2
 * @group Trim Selector
 */
PARAM_DEFINE_FLOAT(TS_MASS, 2.8f);
