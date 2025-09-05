# Linear Fx + Yaw Tilt Control for MCTilt Vehicles

## 概述 (Overview)

本文档描述了为PX4倾转多旋翼 (MCTilt) 飞机实施的线性小扰动前向力(Fx) + 偏航(Yaw)倾转控制算法，包括外部尾舵倾转控制接口。该实现在不修改效果矩阵的情况下，通过倾转舵机提供轻量级的前向力控制能力。

## 算法原理 (Algorithm Principles)

### 1. 线性小扰动模型

基于小角度假设 (|θ| ≤ 17°)，倾转角度θ与水平力的关系可以线性化为：

```
Fx ≈ Σ f_i * θ_i       (前向力)
Mz ≈ -Σ y_i * f_i * θ_i (偏航力矩)
```

其中：
- `f_i ≈ ct_i * u_i` (第i个电机的推力，ct为推力系数，u为归一化输出)
- `y_i` 为第i个电机在机体Y轴上的位置
- `θ_i` 为对应倾转舵机的物理角度

### 2. 角度分解策略

倾转角度被分解为三个分量：

```
θ_fl = Δ + δ           (左前舵机)
θ_fr = Δ - δ           (右前舵机)  
θ_tail = θ_external    (尾舵：外部控制)
```

其中：
- **Δ** (delta_common): 共同前向增量，用于产生前向力
- **δ** (delta_yaw): 偏航差动，用于产生偏航力矩
- **θ_external**: 来自外部uORB消息的尾舵角度

### 3. 控制分配算法

#### Step 1: 尾舵角度分配
```cpp
// 优先使用外部控制
if (readExternalTailTilt(ext_tail, tilt_index_tail)) {
    theta_tail = ext_tail;
    tail_used = true;
}
// 兜底：内部分配 (TAIL_FX_SHARE = 0.0 默认禁用)
if (!tail_used && TAIL_FX_SHARE > 0.f) {
    theta_tail = TAIL_FX_SHARE * Fx_cmd / f_tail;
}
```

#### Step 2: 偏航差动计算
```cpp
// δ ≈ -Mz / (2 * y_mean * f_pair_equiv)
delta_yaw = -Mz_cmd / (2.f * y_mean_abs * f_pair_equiv);
```

#### Step 3: 前向共同增量
```cpp
// Δ = (Fx_front) / (f_fl + f_fr)
float Fx_front = Fx_cmd - f_tail * theta_tail;
delta_common = Fx_front / pair_sum_f;
```

### 4. 垂直推力补偿

由于倾转导致的垂直推力损失 `≈ 0.5 * f_i * θ^2`，通过缩放电机输出进行一阶补偿：

```cpp
float scale = 1.f + 0.5f * theta_g * theta_g;
motor_cmd = actuator_sp(i) * scale;
```

## 代码修改详情 (Code Modifications)

### 1. 新增文件

#### `msg/tail_tilt_setpoint.msg`
```
uint64 timestamp                    # 时间戳
float32 normalized_setpoint         # [0,1] 归一化尾舵倾转角度设定值
# TOPICS tail_tilt_setpoint
```

### 2. 修改的文件

#### `msg/CMakeLists.txt`
添加了新的uORB消息到构建系统：
```cmake
tail_tilt_setpoint.msg
```

#### `ActuatorEffectivenessMCTilt.hpp`

**新增配置常量：**
```cpp
// 功能开关
static constexpr bool ENABLE_LINEAR_FX_TILT = true;
static constexpr bool ENABLE_EXTERNAL_TAIL_TILT = true;

// 算法参数
static constexpr float MAX_LINEAR_TILT_RAD = 0.30f;  // ~17°
static constexpr float TAIL_Y_EPS = 0.05f;          // 尾舵识别阈值
static constexpr float TAIL_FX_SHARE = 0.0f;        // 内部分配比例(禁用)
static constexpr float EPS_F = 1e-5f;               // 数值稳定性
```

**新增成员变量：**
```cpp
float _fx_residual{0.f};                          // Fx残余反馈
uORB::Subscription _tail_tilt_setpoint_sub;       // 外部尾舵控制订阅
bool _tail_tilt_sub_initialized{false};           // 订阅初始化标志
float _last_tail_tilt_norm{NAN};                  // 最后一次尾舵设定值
hrt_abstime _last_tail_tilt_ts{0};                // 最后一次消息时间戳
```

**新增方法：**
```cpp
float angleToServoNormalized(int tilt_index, float theta) const;
void initTailTiltSubscription();
bool readExternalTailTilt(float &theta_tail_out, int tail_tilt_index);
```

#### `ActuatorEffectivenessMCTilt.cpp`

**主要修改在 `updateSetpoint()` 方法中：**

1. **电机分组逻辑：**
```cpp
// 根据Y位置将电机分为三组
if (fabsf(y) < TAIL_Y_EPS) {
    // 尾部电机组
    f_tail += fi;
    motor_group[i] = Group::Tail;
} else if (y > 0.f) {
    // 左前电机组
    f_fl += fi;
    motor_group[i] = Group::FrontLeft;
} else {
    // 右前电机组  
    f_fr += fi;
    motor_group[i] = Group::FrontRight;
}
```

2. **外部尾舵控制集成：**
```cpp
// 延迟初始化uORB订阅
if (ENABLE_EXTERNAL_TAIL_TILT && !_tail_tilt_sub_initialized) {
    _tail_tilt_setpoint_sub = uORB::Subscription{ORB_ID(tail_tilt_setpoint)};
    _tail_tilt_sub_initialized = true;
}

// 优先使用外部控制
if (readExternalTailTilt(ext_tail, tilt_index_tail)) {
    theta_tail = ext_tail;
    tail_used = true;
}
```

3. **角度到舵机映射：**
```cpp
auto setServo = [&](int tilt_index, float theta) {
    if (tilt_index < 0 || tilt_index >= _tilts.count()) return;
    float s_norm = angleToServoNormalized(tilt_index, theta);
    int servo_col = _first_tilt_idx + tilt_index;
    actuator_sp(servo_col) = s_norm;
};
```

4. **残余力反馈：**
```cpp
// 在 getUnallocatedControl() 中
if (ENABLE_LINEAR_FX_TILT && matrix_index == 0) {
    status.unallocated_thrust[0] += _fx_residual;
}
```

## 飞行器舵机行为分析 (Vehicle Servo Behavior)

### 三轴倾转配置下的Yaw控制

当控制分配器产生**右偏航请求**时：

| 舵机位置 | 角度计算 | 实际行为 | 作用机制 |
|----------|----------|----------|----------|
| 左前舵机 | `θ_fl = Δ + δ` | 相对后倾 | 减少左侧前向分量 |
| 右前舵机 | `θ_fr = Δ - δ` | 相对前倾 | 增加右侧前向分量 |
| 尾部舵机 | `θ_tail = external` | 独立控制 | 不受yaw扰动影响 |

**关键优势：**
- ✅ **解耦控制**: 前向力和yaw力矩相对独立分配
- ✅ **尾舵专用**: 尾部舵机专注前向推进，提高效率
- ✅ **外部灵活性**: 支持来自其他模块的精确尾舵控制

### 控制优先级

```
1. Yaw控制 (最高优先级)
   ├── 前舵机差动倾转
   └── 饱和检测与反馈
   
2. 外部尾舵控制 (中优先级)  
   ├── uORB消息接收
   └── 角度范围限制
   
3. 内部前向力分配 (兜底)
   ├── 仅在外部控制失效时启用
   └── TAIL_FX_SHARE配置控制
```

## 使用示例 (Usage Examples)

### 1. 外部模块控制尾舵

```cpp
#include <uORB/topics/tail_tilt_setpoint.h>

// 发布尾舵倾转角度
tail_tilt_setpoint_s msg{};
msg.timestamp = hrt_absolute_time();
msg.normalized_setpoint = 0.3f;  // 30%倾转角度

orb_advertise(ORB_ID(tail_tilt_setpoint), &msg);
```

### 2. 参数调整

```cpp
// 调整最大线性角度范围
static constexpr float MAX_LINEAR_TILT_RAD = 0.26f; // ~15°

// 启用内部尾舵分配兜底
static constexpr float TAIL_FX_SHARE = 0.3f; // 30%前向力分配给尾舵
```

### 3. 诊断与调试

通过 `control_allocator_status` 消息监控：
```cpp
// 检查前向力残余
float fx_residual = status.unallocated_thrust[0];

// 检查yaw饱和状态  
bool yaw_saturated = (status.unallocated_torque[2] != 0.f);
```

## 限制与注意事项 (Limitations & Notes)

### 算法限制
1. **小角度假设**: 倾转角度应保持在 ±17° 以内，超出范围线性近似误差增大
2. **几何假设**: 假定三组电机布局（左前、右前、尾部），其他布局需要调整分组逻辑
3. **效果矩阵**: Fx行仍为零，分配器本身不"知道"Fx自由度，依赖残余反馈

### 实施注意
1. **uORB消息**: 确保 `tail_tilt_setpoint.msg` 在构建系统中正确注册
2. **参数配置**: 根据具体飞行器几何调整 `TAIL_Y_EPS` 等参数
3. **安全限制**: 所有角度都有硬限制保护，防止超出舵机物理范围

## 扩展方向 (Future Extensions)

1. **动态效果矩阵**: 将Fx控制集成到效果矩阵中，实现真正的6DOF控制分配
2. **非线性补偿**: 实现更高精度的三角函数补偿算法
3. **自适应参数**: 根据飞行状态动态调整控制分配策略
4. **多种几何支持**: 扩展支持更复杂的倾转多旋翼布局

---

**版本**: PX4 v1.16 + Linear Fx/Yaw Tilt Extension  
**作者**: Claude Code Assistant  
**日期**: 2025年1月  
**状态**: 已测试编译通过