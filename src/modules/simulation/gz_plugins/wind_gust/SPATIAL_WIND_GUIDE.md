# 空间变化风场配置指南

## 概述

WindGustSystem 现在支持**时间 + 空间**双重变化的风场模型：

```
wind(x,y,z,t) = wind_mean + wind_temporal(t) + wind_spatial(x,y,z)
```

- **wind_mean**: 平均风（常值）
- **wind_temporal(t)**: 时间变化（原有功能）- sine, dryden, one_minus_cos 等
- **wind_spatial(x,y,z)**: 空间变化（新增功能）- 跟踪飞机位置

---

## 支持的空间风场模型

### 1. 线性风切变 (linear_shear)

**描述**: 风速沿任意方向线性变化，模拟短距离内的风切变

**物理公式**:
```
ΔV = gradient_x · Δx + gradient_y · Δy + gradient_z · Δz
```

**应用场景**:
- 垂直风切变（高度相关）
- 水平风切变（前沿、阵风锋）
- 微爆流（microburst）边界

**配置示例**:
```xml
<plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
  <!-- 基础参数 -->
  <mean>5 0 0</mean>  <!-- 5 m/s 东风 -->

  <!-- 空间模型配置 -->
  <spatial_model>linear_shear</spatial_model>
  <tracked_model>tiltrotor</tracked_model>

  <!-- 垂直风切变: 每升高10m，X方向风速增加2 m/s -->
  <shear_gradient_z>0.2 0 0</shear_gradient_z>
  <shear_ref_pos>0 0 0</shear_ref_pos>

  <!-- 或水平风切变: X方向每前进50m，风速增加5 m/s -->
  <!-- <shear_gradient_x>0.1 0 0</shear_gradient_x> -->
</plugin>
```

**高级示例 - 三维风切变**:
```xml
<!-- 复杂风切变: 模拟阵风锋 -->
<shear_gradient_x>0.05 0.02 0</shear_gradient_x>  <!-- X: 前进方向 -->
<shear_gradient_y>0 0.1 0</shear_gradient_y>      <!-- Y: 横向变化 -->
<shear_gradient_z>0.2 0 0.05</shear_gradient_z>   <!-- Z: 垂直变化 -->
<shear_ref_pos>100 0 0</shear_ref_pos>            <!-- 阵风锋中心 -->
```

---

### 2. 空间正弦波 (sine_wave)

**描述**: 正弦形式的空间风场，模拟大气重力波

**物理公式**:
```
V_spatial = A · sin(2π · k·r / λ + φ)
```
其中:
- A = sine_amplitude (幅值向量)
- k = sine_direction (波传播方向)
- λ = sine_wavelength (波长)
- φ = sine_phase (相位)

**应用场景**:
- 山地波动（lee waves）
- 大气重力波
- 周期性风场干扰

**配置示例**:
```xml
<plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
  <mean>10 0 0</mean>

  <spatial_model>sine_wave</spatial_model>
  <tracked_model>tiltrotor</tracked_model>

  <!-- 沿X方向传播的垂直振荡风 -->
  <sine_amplitude>0 0 2</sine_amplitude>      <!-- 2 m/s 垂直振幅 -->
  <sine_direction>1 0 0</sine_direction>      <!-- X方向传播 -->
  <sine_wavelength>100</sine_wavelength>       <!-- 100m 波长 -->
  <sine_phase>0</sine_phase>
</plugin>
```

**组合时空变化**:
```xml
<!-- 时间变化 + 空间变化 -->
<model>sine</model>
<frequency>0.1</frequency>
<amplitude>1 0 0</amplitude>

<spatial_model>sine_wave</spatial_model>
<sine_amplitude>0 1 0</sine_amplitude>
<sine_wavelength>50</sine_wavelength>
<!-- 结果: 风速随时间和空间双重振荡 -->
```

---

### 3. 涡旋风场 (vortex)

**描述**: 旋转风场，模拟龙卷风、尘卷风

**物理公式** (Rankine 涡模型):
```
核心区 (r < r_core): v_θ = Γ · r / r_core²
外围区 (r > r_core): v_θ = Γ / r
```

**应用场景**:
- 龙卷风/尘卷风
- 热气流上升涡旋
- 建筑物尾流涡旋

**配置示例**:
```xml
<plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
  <spatial_model>vortex</spatial_model>
  <tracked_model>tiltrotor</tracked_model>

  <!-- 涡旋中心位置 -->
  <vortex_center>50 50 0</vortex_center>

  <!-- 环量强度 (正值=逆时针) -->
  <vortex_strength>50</vortex_strength>

  <!-- 涡核半径 -->
  <vortex_core_radius>10</vortex_core_radius>
</plugin>
```

**危险场景 - 强龙卷风**:
```xml
<vortex_strength>200</vortex_strength>      <!-- 强环量 -->
<vortex_core_radius>5</vortex_core_radius>  <!-- 小涡核 -->
<!-- 核心区最大风速: 200/5 = 40 m/s -->
```

---

### 4. 边界层风廓线 (boundary_layer)

**描述**: 大气边界层的垂直风速分布

**物理公式**:
```
V(z) = V_ref · (z / z_ref)^α
```
其中 α 取决于大气稳定度:
- α = 0.10 (不稳定, 对流)
- α = 0.143 (中性)
- α = 0.20~0.40 (稳定)

**应用场景**:
- 真实大气风廓线
- 低空飞行风场
- 起降阶段风环境

**配置示例**:
```xml
<plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
  <spatial_model>boundary_layer</spatial_model>
  <tracked_model>tiltrotor</tracked_model>

  <!-- 参考高度10m处风速 -->
  <bl_ref_wind>15 0 0</bl_ref_wind>
  <bl_ref_height>10.0</bl_ref_height>

  <!-- 中性大气 -->
  <bl_exponent>0.143</bl_exponent>
</plugin>
```

**高度风速对比**:
```
z = 1m:   V = 15 × (1/10)^0.143  = 11.0 m/s
z = 5m:   V = 15 × (5/10)^0.143  = 13.5 m/s
z = 10m:  V = 15 × (10/10)^0.143 = 15.0 m/s  (参考高度)
z = 50m:  V = 15 × (50/10)^0.143 = 18.4 m/s
```

---

## 完整配置示例

### 示例 1: 垂直风切变 + 时间阵风

```xml
<world name="wind_shear_test">
  <wind>
    <linear_velocity>0 0 0</linear_velocity>
  </wind>

  <plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
    <!-- 时间变化: 1-cos 阵风 -->
    <model>one_minus_cos_simp</model>
    <mean>10 0 0</mean>
    <direction>1 0 0</direction>
    <A0>5.0</A0>      <!-- 5 m/s 阵风幅值 -->
    <T>10.0</T>       <!-- 10秒周期 -->

    <!-- 空间变化: 垂直风切变 -->
    <spatial_model>linear_shear</spatial_model>
    <tracked_model>tiltrotor</tracked_model>
    <shear_gradient_z>0.15 0 0</shear_gradient_z>
    <shear_ref_pos>0 0 10</shear_ref_pos>
  </plugin>
</world>
```

### 示例 2: 空间正弦波 + Dryden 湍流

```xml
<plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
  <!-- 时间变化: Dryden 湍流 -->
  <model>dryden</model>
  <mean>12 0 0</mean>
  <dryden_sigma>2.0 1.5 1.0</dryden_sigma>
  <dryden_length>200 200 50</dryden_length>
  <seed>12345</seed>

  <!-- 空间变化: 大气波动 -->
  <spatial_model>sine_wave</spatial_model>
  <tracked_model>tiltrotor</tracked_model>
  <sine_amplitude>0 2 1</sine_amplitude>
  <sine_direction>1 0 0</sine_direction>
  <sine_wavelength>150</sine_wavelength>
</plugin>
```

### 示例 3: 涡旋风场（龙卷风测试）

```xml
<plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
  <mean>0 0 0</mean>

  <spatial_model>vortex</spatial_model>
  <tracked_model>tiltrotor</tracked_model>

  <!-- 中等强度龙卷风 -->
  <vortex_center>100 0 0</vortex_center>
  <vortex_strength>100</vortex_strength>
  <vortex_core_radius>8</vortex_core_radius>
</plugin>
```

---

## 调试和验证

### 1. 检查风速话题
```bash
# 实时查看风速
gz topic -e -t /world/<world_name>/wind_gust

# 记录数据
gz topic -e -t /world/<world_name>/wind_gust > wind_data.txt
```

### 2. 可视化飞机位置
```bash
# 查看飞机位置
gz topic -e -t /world/<world_name>/pose/info | grep tiltrotor
```

### 3. Python 脚本验证
```python
import numpy as np
import matplotlib.pyplot as plt

# 线性风切变验证
z = np.linspace(0, 50, 100)
gradient_z = 0.15  # m/s per m
ref_pos_z = 10
wind = 10 + gradient_z * (z - ref_pos_z)

plt.plot(wind, z)
plt.xlabel('Wind Speed (m/s)')
plt.ylabel('Altitude (m)')
plt.title('Vertical Wind Shear')
plt.grid(True)
plt.show()
```

---

## 注意事项

1. **向后兼容**: 不设置 `spatial_model` 时，行为与原版完全相同
2. **性能**: 空间计算每帧执行一次，对性能影响极小
3. **组合使用**: 可以同时启用时间和空间变化
4. **单机限制**: 目前只能跟踪一个模型，多机场景需要额外开发

---

## 参考文献

1. MIL-F-8785C: Military Specification - Flying Qualities of Piloted Airplanes
2. MIL-STD-1797A: Flying Qualities of Piloted Aircraft
3. Etkin, B., "Dynamics of Atmospheric Flight"
4. Rankine vortex model: https://en.wikipedia.org/wiki/Rankine_vortex
