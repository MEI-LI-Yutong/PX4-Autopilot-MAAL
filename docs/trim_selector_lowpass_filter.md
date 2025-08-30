# TrimSelector 低通滤波功能

## 概述

TrimSelector 模块在 v1.16-trim-CA_tilt 分支中新增了一阶低通滤波功能，用于平滑发布到 uORB 的 `utrim` 消息数据。该功能旨在减少配平值的高频噪声，提供更加稳定的控制分配输入。

## 新增成员变量

### 滤波器状态变量

```cpp
class TrimSelector {
private:
    // Low-pass filter state for fc = 0.5 Hz
    bool _lp_inited{false};       // 滤波器初始化标志
    float _utrim_lp[6]{};         // polynomial_values 的 LPF 状态
    float _utrim_norm_lp[6]{};    // normalized_values 的 LPF 状态
};
```

#### `_lp_inited`
- **类型**: `bool`
- **初始值**: `false`
- **用途**: 滤波器初始化标志位
- **功能**: 
  - 冷启动时为 `false`，首次运行时直接将滤波状态置为当前计算值
  - 避免启动时产生瞬态尖峰响应
  - 初始化后置为 `true`，后续使用正常的滤波更新公式

#### `_utrim_lp[6]`
- **类型**: `float[6]`
- **初始值**: `{0,0,0,0,0,0}`
- **用途**: `utrim.polynomial_values` 的低通滤波状态存储
- **数据结构**:
  - `_utrim_lp[0]` → f1 推力值滤波状态 [N]
  - `_utrim_lp[1]` → f2 推力值滤波状态 [N] 
  - `_utrim_lp[2]` → f3 推力值滤波状态 [N]
  - `_utrim_lp[3]` → θ1 角度值滤波状态 [deg]
  - `_utrim_lp[4]` → θ2 角度值滤波状态 [deg]
  - `_utrim_lp[5]` → θ3 角度值滤波状态 [deg]

#### `_utrim_norm_lp[6]`
- **类型**: `float[6]`
- **初始值**: `{0,0,0,0,0,0}`
- **用途**: `utrim.normalized_values` 的低通滤波状态存储
- **数据结构**:
  - `_utrim_norm_lp[0-2]` → 电机相关归一化值滤波状态 [0,1]
  - `_utrim_norm_lp[3-5]` → 舵机相关归一化值滤波状态 [-1,1]

## 滤波器参数

### 基本参数
- **截止频率**: `fc = 0.5 Hz`
- **时间常数**: `τ = 1 / (2π × fc) ≈ 0.318 秒`
- **滤波器类型**: 一阶低通滤波器
- **更新公式**: `y += α × (x - y)`，其中 `α = dt / (τ + dt)`

### 自适应特性
- **dt 自适应**: α 系数根据实际时间步长 dt 动态计算
- **时间步长约束**: dt 被限制在 [2ms, 50ms] 范围内
- **频率鲁棒性**: 即使模块调度频率有微小抖动，滤波特性仍保持稳定

## 工作流程

### 1. 初始化阶段
```cpp
if (!_lp_inited) {
    // 冷启动：直接将滤波状态置为当前值
    for (int i = 0; i < 6; ++i) {
        _utrim_lp[i] = poly_raw[i];
        _utrim_norm_lp[i] = norm_raw[i];
    }
    _lp_inited = true;
}
```

### 2. 正常运行阶段
```cpp
// 一阶低通滤波：y += α (x - y)
const float alpha = dt / (tau + dt);
for (int i = 0; i < 6; ++i) {
    _utrim_lp[i] += alpha * (poly_raw[i] - _utrim_lp[i]);
    _utrim_norm_lp[i] += alpha * (norm_raw[i] - _utrim_norm_lp[i]);
}
```

### 3. 发布阶段
```cpp
// 使用滤波后的值发布
for (int i = 0; i < 6; ++i) {
    utrim.polynomial_values[i] = _utrim_lp[i];
    utrim.normalized_values[i] = _utrim_norm_lp[i];
}
```

## 性能特性

### 频率响应
- **-3dB 截止频率**: 0.5 Hz
- **衰减特性**: 频率每增加10倍，幅值衰减20dB
- **相位延迟**: 在截止频率处约为45°

### 时域特性
- **建立时间**: 约 2-3 个时间常数 (0.6-1.0 秒)
- **过冲**: 无过冲（一阶系统）
- **稳态误差**: 零稳态误差

## 运行配置

### 模块频率
- **调度频率**: 10 Hz (100ms 间隔)
- **发布频率**: 10 Hz (与调度频率同步)
- **滤波更新**: 每个调度周期更新一次

### 与现有功能的兼容性
- **Ramp 逻辑**: 完全保持不变，滤波作用在 ramp 应用之后
- **Smoothstep**: 保持不变，用于 ramp 因子的平滑处理
- **归一化计算**: 保持不变，滤波分别作用于原始值和归一化值
- **发布限频**: 从 20Hz 调整为 10Hz，与调度频率一致

## 使用场景

### 适用情况
- **高频噪声抑制**: 减少配平值的快速抖动
- **控制系统稳定性**: 为下游控制分配器提供平滑的输入
- **传感器噪声过滤**: 过滤来自轨迹计算的高频分量

### 注意事项
- **响应延迟**: 0.5Hz 截止频率会引入约 0.3 秒的响应延迟
- **瞬态抑制**: 快速的配平变化会被适当平滑
- **启动特性**: 冷启动时无瞬态响应，直接跟踪当前值

## 调试和监控

### 日志记录
滤波后的值会通过 uORB 消息发布，可在飞行日志中查看：
- `utrim.polynomial_values[0-5]` - 滤波后的配平值
- `utrim.normalized_values[0-5]` - 滤波后的归一化值

### 调试输出
如需查看详细的滤波效果，可以取消注释 `Run()` 函数中第323-334行的调试代码：
```cpp
PX4_DEBUG("TrimSelector: s=%.2f -> f=[%.2f %.2f %.2f]N, th=[%.1f %.1f %.1f]deg, vxy=%.2f, valid=%s",
          (double)s,
          (double)_utrim_lp[0], (double)_utrim_lp[1], (double)_utrim_lp[2],
          (double)_utrim_lp[3], (double)_utrim_lp[4], (double)_utrim_lp[5],
          (double)vxy_mag, data_valid ? "true":"false");
```

## 总结

TrimSelector 的低通滤波功能通过三个关键成员变量 (`_lp_inited`, `_utrim_lp[6]`, `_utrim_norm_lp[6]`) 实现了对配平数据的平滑处理。该功能在保持原有模块逻辑不变的基础上，为控制分配系统提供了更加稳定和可靠的配平输入，有助于提升整体飞行控制性能。