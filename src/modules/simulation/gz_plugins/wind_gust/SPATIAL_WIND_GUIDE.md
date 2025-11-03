# 空间变化风场配置指南

## 概述

WindGustSystem 支持“时间 + 空间”的风场叠加：

```
wind(x,y,z,t) = wind_mean + wind_temporal(t) + wind_spatial(x,y,z)
```

- wind_mean: 平均风（常值）
- wind_temporal(t): 时间变化（原有功能）- sine, dryden, one_minus_cos 等
- wind_spatial(x,y,z): 空间变化（仅保留 boundary_layer 模型）

---

## 支持的空间风场模型（仅保留）

### 边界层风廓线 (boundary_layer)

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

### 示例：边界层 + 时间阵风

```xml
<world name="boundary_layer_test">
  <wind>
    <linear_velocity>0 0 0</linear_velocity>
  </wind>

  <plugin filename="gz-sim-wind-gust-system" name="custom::WindGustSystem">
    <!-- 时间变化: 1-cos 阵风 -->
    <model>one_minus_cos_simp</model>
    <mean>10 0 0</mean>
    <direction>1 0 0</direction>
    <A0>5.0</A0>
    <T>10.0</T>

    <!-- 空间变化: 边界层风廓线 -->
    <spatial_model>boundary_layer</spatial_model>
    <tracked_model>tiltrotor</tracked_model>
    <bl_ref_wind>15 0 0</bl_ref_wind>
    <bl_ref_height>10.0</bl_ref_height>
    <bl_exponent>0.143</bl_exponent>
  </plugin>
</world>
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

1. 向后兼容: 未设置或设置为 `none` 时，不启用空间风场
2. 模型范围: 仅支持 `boundary_layer`，配置其他值将被忽略并打印告警
3. 性能: 空间计算每帧一次，对性能影响极小
4. 组合: 可与时间模型（sine/dryden/one_minus_cos 等）叠加

---

## 参考文献

1. MIL-F-8785C: Military Specification - Flying Qualities of Piloted Airplanes
2. MIL-STD-1797A: Flying Qualities of Piloted Aircraft
3. Etkin, B., "Dynamics of Atmospheric Flight"
