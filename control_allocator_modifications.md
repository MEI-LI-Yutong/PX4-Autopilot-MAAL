# Control Allocator 修改总结

## 修改概述
基于用户提供的控制分配模型，对 Control Allocator 模块进行了修改，实现了自定义的控制分配算法，包括trim处理和完整的actuator输出。

## 修改内容

### 1. 头文件修改 (ControlAllocator.hpp)
- 添加了 `#include <uORB/topics/utrim.h>` 以支持 utrim 消息
- 添加了 `uORB::Subscription _utrim_sub{ORB_ID(utrim)};` 用于订阅 utrim 消息
- 添加了自定义分配相关的成员变量：
  ```cpp
  // Custom allocation results
  matrix::Vector<float, 6> _custom_allocation_result;
  bool _custom_allocation_valid{false};

  // 自定义trim向量
  matrix::Vector<float, NUM_ACTUATORS> _custom_trim_vec;
  ```
- 添加了函数声明：
  ```cpp
  void calculate_custom_allocation();
  bool apply_custom_allocation(matrix::Vector<float, NUM_ACTUATORS> &actuator_sp);
  ```

### 2. 实现文件修改 (ControlAllocator.cpp)

#### 2.1 控制分配流程修改
在 `Run()` 函数中修改了分配流程：
```cpp
// 首先尝试自定义分配
calculate_custom_allocation();

// 在分配循环中优先使用自定义分配
if (_custom_allocation_valid) {
    matrix::Vector<float, NUM_ACTUATORS> custom_actuator_sp;
    if (apply_custom_allocation(custom_actuator_sp)) {
        _control_allocation[i]->setActuatorSetpoint(custom_actuator_sp);
        PX4_INFO("使用自定义控制分配");
    } else {
        // 回退到标准分配
        _control_allocation[i]->allocate();
    }
} else {
    // 回退到标准分配
    _control_allocation[i]->allocate();
}
```

#### 2.2 自定义分配算法实现
实现了 `calculate_custom_allocation()` 函数，包含以下逻辑：

**控制分配模型**：
$$\begin{bmatrix}
f_x \\
f_z \\
\tau_x/L_3 \\
\tau_y/L_1 \\
\tau_z/L_3
\end{bmatrix} =
\begin{bmatrix}
\sin\theta_1 & \sin\theta_2 & \sin\theta_3 & \cos\theta_1 & \cos\theta_2 & \cos\theta_3 \\
-\cos\theta_1 & -\cos\theta_2 & -\cos\theta_3 & \sin\theta_1 & \sin\theta_2 & \sin\theta_3 \\
-\cos\theta_1 & \cos\theta_2 & 0 & \sin\theta_1 & -\sin\theta_2 & 0 \\
\cos\theta_1 & \cos\theta_2 & -\left(L_2/L_1\right)\cos\theta_3 & -\sin\theta_1 & -\sin\theta_2 & \left(L_2/L_1\right)\sin\theta_3 \\
-\sin\theta_1 & \sin\theta_2 & 0 & -\cos\theta_1 & \cos\theta_2 & 0
\end{bmatrix}
\begin{bmatrix}
df_1 \\
df_2 \\
df_3 \\
df_1d\theta_1 \\
df_2d\theta_2 \\
df_3d\theta_3
\end{bmatrix}$$

**实现细节**：
1. **输入获取**：
   - 从 `_thrust_sp` 和 `_torque_sp` 获取期望的力和力矩
   - 从 utrim 消息获取参数：
     - `f1, f2, f3 = utrim.polynomial_values[0,1,2]`（推力相关）
     - `theta1, theta2, theta3 = utrim.polynomial_values[3,4,5]`（角度，转换为弧度）

2. **常量定义**：
   - L1 = 1.0f, L2 = 1.0f, L3 = 0.41f

3. **矩阵求解**：
   - 构建 5x6 效率矩阵 A 和 5x1 向量 b
   - 使用伪逆求解：`du = A^T * (A*A^T)^{-1} * b`
   - 对后三个分量进行缩放：`du(3)/=f1, du(4)/=f2, du(5)/=f3`

#### 2.3 Trim处理实现
实现了完整的trim处理逻辑：

**电机trim处理（前三个量）**：
```cpp
// 推力值转换为电机信号 [0-1]
// 公式：f = 34.024x - 767.4，其中f是推力(N)，x是电机信号[0-100]
// 用户修改：考虑重力加速度和单位转换
float motor_signal = (thrust_value/9.81f*1000.0f + motor_offset) / motor_coeff;
motor_signal = constrain(motor_signal, 0.0f, 100.0f);
_custom_trim_vec(i) = motor_signal / 100.0f;  // 归一化到[0-1]
```

**舵机trim处理（后三个量）**：
```cpp
// 角度转换为[-1,1]范围，基于±45°标准范围
_custom_trim_vec(i) = angle_deg / 45.0f;
_custom_trim_vec(i) = constrain(_custom_trim_vec(i), -1.0f, 1.0f);
```

#### 2.4 执行器输出实现
实现了 `apply_custom_allocation()` 函数：
```cpp
// actuator_sp = du + trim
for (int i = 0; i < NUM_ACTUATORS && i < 6; ++i) {
    actuator_sp(i) = _custom_allocation_result(i) + _custom_trim_vec(i);
}
```

## 数据流程
1. **输入**：`vehicle_torque_setpoint` 和 `vehicle_thrust_setpoint`
2. **参数**：`utrim` 消息提供 f1,f2,f3,θ1,θ2,θ3
3. **处理**：
   - 自定义分配算法计算 du
   - Trim处理转换为标准化值
   - 输出：`actuator_sp = du + trim`
4. **输出**：`actuator_motors` 和 `actuator_servos` 消息

## PWM映射关系
- **Control Allocator 输出**：`actuator_sp` 范围 [-1,1]
- **MixingOutput 处理**：`PWM = PWM_min + (actuator_sp + 1)/2 * (PWM_max - PWM_min)`
- **QGC 参数**：`PWM_MAIN_MIN` 和 `PWM_MAIN_MAX` 定义实际PWM范围

## 功能特点
- ✅ 完整的自定义控制分配算法
- ✅ Trim支持（电机推力 + 舵机角度）
- ✅ 单位转换和归一化处理
- ✅ 回退机制（自定义分配失败时使用标准分配）
- ✅ 详细的调试日志输出
- ✅ 参数验证和错误处理

## 验证
代码已经实现并可以通过 `make px4_sitl_default` 命令进行编译验证。所有修改都保持了与原有PX4架构的兼容性。
