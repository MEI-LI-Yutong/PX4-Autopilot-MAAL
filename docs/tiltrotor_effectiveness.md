# PX4 倾转多旋翼 / 让舵机倾转直接参与 Fx, Fz 与姿态力矩分配的改造说明

作者：
日期：

---

## 0. 目标概述

当前 PX4 `control_allocator` 中，**倾转舵机** 主要通过改变电机推力方向（刷新 rotor axis）或作为附加 yaw/pitch 贡献进入混合，尚未把 “舵机角增量” 当作与 “推力幅值增量” 并列的独立自由度。
本改造目标：在效能矩阵 (Effectiveness Matrix) 中显式加入 **推力幅值增量** 与 **倾转角增量** 两类列，使分配器可以同时利用 “推力大小” 与 “推力方向” 调节来生成
$$
(F_x, F_y, F_z, M_x, M_y, M_z)
$$
并保持与 Method A（baseline trim + incremental control）兼容。

---

## 1. 现有源码逻辑速览

### 1.1 控制向量
控制分配器的目标向量：
$$
v =
\begin{bmatrix}
M_x \\ M_y \\ M_z \\ F_x \\ F_y \\ F_z
\end{bmatrix}
$$

### 1.2 现有效能矩阵含义
当前使用：
$$
\Delta v = E \, \Delta u
$$
其中 $ \Delta u $ 是“执行器归一化输出”小幅度变化（电机推力归一化、舵机归一化等）。
多旋翼电机列（简化）：
$$
\text{thrust}_i = c_{t,i} \, a_i,\quad
\text{moment}_i = c_{t,i} \big( r_i \times a_i - k_{m,i} a_i \big)
$$
放入矩阵行：
$$
E(:, i) =
\begin{bmatrix}
\text{moment}_i \\ \text{thrust}_i
\end{bmatrix}
$$

### 1.3 CT / KM 的作用
- $c_{t,i}$ (CT)：列尺度（线性化推力增益）。
- $k_{m,i}$ (KM)：反扭矩比系数，提供 yaw 权限。
- 即使上层 setpoint 可能“标幺化”，列仍需真实相对权重以决定分配比例与去饱和行为。

---

## 2. 新的增广思路：加入倾转角自由度

### 2.1 新的增量控制变量

为每个可倾转电机添加角度自由度，线性化变量扩展为：
$$
\Delta u_{\text{aug}} =
\begin{bmatrix}
\Delta f_1 & \dots & \Delta f_n & \Delta \theta_1 & \dots & \Delta \theta_n
\end{bmatrix}^T
$$
最终：
$$
\Delta v = E_{\text{aug}} \, \Delta u_{\text{aug}}
$$

（实施层可再通过链式法则映射到电机归一化增量 $\Delta u_i$ 与舵机归一化增量 $\Delta s_i$。）

### 2.2 动机
- 使分配器可选择 “调角度” 而不必靠 “推力差” 来生成 $F_x$ 或滚转 / 俯仰力矩；
- 减少对高度 ( $F_z$ ) 的耦合扰动；
- 提升 yaw 协同（角度列亦含 -$k_{m,i}$ 项）。

---

## 3. 数学线性化推导

### 3.1 定义
第 $i$ 个（可倾转）电机：

| 符号 | 含义 |
|------|------|
| $r_i$ | 电机位置向量（机体系） |
| $a_i(\theta_i)$ | 当前推力方向单位向量（由倾转角 $\theta_i$ 决定） |
| $f_i$ | 推力标量（工作点） |
| $k_{m,i}$ | 反扭矩系数 |
| $h_i$ | 舵机旋转轴单位向量（例如绕机体 $Y$ 轴则 $h_i=[0,1,0]^T$） |

推力与力矩：
$$
F_i = f_i a_i(\theta_i), \qquad
\tau_i = r_i \times (f_i a_i) - k_{m,i} f_i a_i
$$

### 3.2 对推力幅值 $f_i$ 的偏导
$$
\frac{\partial F_i}{\partial f_i} = a_i
$$
$$
\frac{\partial \tau_i}{\partial f_i} = r_i \times a_i - k_{m,i} a_i
$$

### 3.3 对倾转角 $\theta_i$ 的偏导

旋转微分：
$$
\frac{\partial a_i}{\partial \theta_i} = h_i \times a_i
$$

力：
$$
\frac{\partial F_i}{\partial \theta_i} = f_i (h_i \times a_i)
$$

力矩：
$$
\frac{\partial \tau_i}{\partial \theta_i}
= f_i \Big( r_i \times (h_i \times a_i) - k_{m,i} (h_i \times a_i) \Big)
$$

### 3.4 列的构造

将 $df_i$ 与 $d\theta_i$ 列分别写入（按行顺序 $M_x,M_y,M_z,F_x,F_y,F_z$）：

**第 $i$ 列（对应 $df_i$**）：
$$
E_{\text{aug}}(:, i) =
\begin{bmatrix}
(r_i \times a_i - k_{m,i} a_i) \\[4pt]
a_i
\end{bmatrix}
$$

**第 $n+i$ 列（对应 $d\theta_i$**）：
$$
E_{\text{aug}}(:, n+i) =
\begin{bmatrix}
f_i \big( r_i \times (h_i \times a_i) - k_{m,i} (h_i \times a_i) \big) \\[4pt]
f_i (h_i \times a_i)
\end{bmatrix}
$$

> 说明：若实现层使用的是归一化舵机增量 $\Delta s_i$，且 $\theta_i = \theta_{0,i} + K_{\text{servo},i} s_i$，则列再乘以 $K_{\text{servo},i}$。

---

## 4. 与实际电机归一化变量的链式关系

若推力模型使用（源码注释）：
$$
f_i = C_{T,i} u_i^2
$$
在工作点 $u_{i0}$ 线性化：
$$
df_i \approx 2 C_{T,i} u_{i0} \, du_i
$$

因此若直接构建对电机归一化增量 $du_i$ 的列，可将推力幅值列整体乘以 $2C_{T,i}u_{i0}$。
当前主线实现以常数 $c_{t,i}$ 代替该动态项；增强版可改为：
$$
c_{t,i}^{\text{eff}} = 2 C_{T,i} u_{i0}
$$

---

## 5. Method A：基线与增量

设工作点（配平）执行器向量（含电机与舵机）：
$$
u_0 =
\begin{bmatrix}
u_{1,0} & \dots & u_{n,0} & s_{1,0} & \dots & s_{n,0}
\end{bmatrix}^T
$$

构造：
$$
\text{actuator\_trim} = u_0
$$

用增广矩阵：
$$
\text{control\_trim} = E_{\text{aug}} \, \text{actuator\_trim}
$$

分配目标增量：
$$
\Delta v_{\text{target}} = v_{\text{sp}} - \text{control\_trim}
$$

求增量解：
$$
\Delta u_{\text{aug}} \approx E_{\text{aug}}^{+} \, \Delta v_{\text{target}}
$$

输出命令：
$$
u = \text{actuator\_trim} + \Delta u_{\text{aug}}
$$

---

## 6. 动态更新策略

触发条件（任一满足）：
- $|u_{i0}^{\text{new}} - u_{i0}^{\text{old}}| > \epsilon_u$（例如 0.02）
- $|\theta_{i}^{\text{new}} - \theta_{i}^{\text{old}}| > \epsilon_\theta$（例如 0.5°）
- utrim 刷新（起飞/巡航配平变化）
- 飞行阶段变化（tilt 统合与姿态模式切换）

步骤：
1. 读取新配平（utrim） → 更新 $u_0, \theta_0$；
2. 计算 $a_i(\theta_0)$、$h_i$、$f_i = c_{t,i}^{\text{eff}}$ 或 $c_{t,i} u_{i0}^2$；
3. 构建增广矩阵 $E_{\text{aug}}$；
4. 计算 `control_trim = E_aug * actuator_trim`；
5. 传入分配器（绕过 100 ms 节流：`dynamicUpdatePending()`）。

---

## 7. 常见问题与规避

| 问题 | 现象 | 原因 | 规避 |
|------|------|------|------|
| Yaw 不可控 | `unallocated_torque[2]` 大 | 忽略 $k_{m,i}$ 或符号错 | 正确设置 KM 正负 |
| 前向力引起高度扰动 | Fz 振荡 | 仅用推力差生成 Fx | 引入角度列，限制推力列变化 |
| 角度列过度使用 | 舵机打满 ±1 | 列尺度过大 | 对角度列加权或正则 |
| Rank 降低 | 伪逆条件数高 | 所有 tilt 角相同导致列相关 | 设最小 $f_i$，必要时冻结某些列 |
| 跳变尖峰 | 推力过冲 | 更新 trim 后未同时更新 control_trim | 保证同步重建 |

---

## 8. 验证 Checklist

1. **打印矩阵**：新增角度列是否出现非零力行与力矩行。
2. **Hover**：仅 $F_z$ 指令时，角度列基本不被使用。
3. **小 $F_x$ 步进**：主要通过角度列产生 $F_x$，而非大幅推力差。
4. **Yaw 步进**：电机差速与角度列共同工作，`unallocated_torque[2]` 接近 0。
5. **大前倾**：矩阵条件数保持可接受（< 500）。
6. **动力变化**：utrim 更新 → E 与 control_trim 快速刷新，无一次性冲击。
7. **单电机失效**：角度列帮助维持部分 $F_x/M$ 能力。

---

## 9. 公式速览（核心）

**推力与力矩：**
$$
F_i = f_i a_i,\qquad
\tau_i = r_i \times (f_i a_i) - k_{m,i} f_i a_i
$$

**偏导：**
$$
\frac{\partial F_i}{\partial f_i} = a_i,\quad
\frac{\partial \tau_i}{\partial f_i} = r_i \times a_i - k_{m,i} a_i
$$

$$
\frac{\partial a_i}{\partial \theta_i} = h_i \times a_i
$$

$$
\frac{\partial F_i}{\partial \theta_i} = f_i (h_i \times a_i),\quad
\frac{\partial \tau_i}{\partial \theta_i} = f_i \big(r_i \times (h_i \times a_i) - k_{m,i} (h_i \times a_i)\big)
$$

**列组合：**
$$
E_{\text{aug}}(:, i) =
\begin{bmatrix}
r_i \times a_i - k_{m,i} a_i \\ a_i
\end{bmatrix},\quad
E_{\text{aug}}(:, n+i) =
\begin{bmatrix}
f_i \big(r_i \times (h_i \times a_i) - k_{m,i} (h_i \times a_i)\big) \\ f_i (h_i \times a_i)
\end{bmatrix}
$$

---

## 10. 渐进实施路线

| 阶段 | 内容 |
|------|------|
| 0 | Method A：引入 `actuator_trim` / `control_trim` 基线 |
| 1 | 增加角度列（不加 km）验证 Fx 生成 |
| 2 | 加入 km 项，验证 yaw 行为 |
| 3 | 动态线性化 `ct_eff = 2 C_T u_0` |
| 4 | 列缩放 / 正则（必要时） |
| 5 | 自适应或在线估计 CT/KM（高级） |

---

## 11. 总结

通过在效能矩阵中显式加入推力幅值与倾转角增量两类列，并使用基线 + 增量 (Method A) 框架，能够使倾转多旋翼在前向力、姿态力矩分配与 yaw 协同上获得更高效率与更好解耦。
关键在于正确的线性化偏导、同步更新的 `control_trim`、以及合适的动态刷新与列尺度管理。

---

（完）
