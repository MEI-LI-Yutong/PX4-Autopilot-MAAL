# 倾转多旋翼前向推力实现的两种算法
（算法 A(3) “纯投影” vs 算法 B “小信号线性化增量分配” – 修正版）

---

## 1. 目标与背景

在保持机体俯仰角 (pitch ≈ 0) 的同时产生前向推力 $F_x$，并维持 $F_z$ 与姿态力矩 $M_x,M_y,M_z$ 控制。
现有 PX4 `ActuatorEffectivenessMCTilt` 为静态矩阵，未真实利用倾角产生 $F_x$。本文比较两种实现：

| 名称 | 核心思想 | CA 中是否直接分配 $F_x$ | 是否线性化 | 额外自由度 |
|------|----------|-------------------------|------------|------------|
| A(3) 纯投影 | 先几何反解 $(F_x,F_z)\to(T,\theta)$，CA 只分配 1D 推力+力矩 | 否（或 Fx 行被置 0） | 否 | 电机推力幅值 |
| B 小信号线性化 | 工作点 $(f^*,\theta^*)$ 线性化 $y\approx y_{\text{trim}}+E\,\delta u$ | 是 | 是 | 推力增量 + 倾角增量 |

---

## 2. 坐标与符号

- 机体系：$x$ 前，$y$ 右，$z$ 下（PX4 多旋翼惯例）。
- 单倾转电机轴向（绕机体 $y$ 轴前倾角 $\theta$）
  $$
  a(\theta)=
  \begin{bmatrix}
  \sin\theta\\
  0\\
  -\cos\theta
  \end{bmatrix}
  $$

- 电机 $i$ 推力矢量：$\mathbf{f}_i = c_{t,i} f_i\, a(\theta_i)$（此处假设线性：推力 $= c_{t,i} f_i$；若平方律只需替换导数系数）
- 总力与力矩：
  $$
  \mathbf{F}=\sum_i \mathbf{f}_i,\qquad
  \mathbf{M}=\sum_i \left(\mathbf{r}_i \times \mathbf{f}_i\right) - k_{m,i}\mathbf{f}_i
  $$

---

## 3. 算法 A(3)：纯投影（几何反解）

### 3.1 反解公式

给定期望 $(F_{x,\text{des}}, F_{z,\text{des}})$（机体系）：
$$
\theta = \operatorname{atan2}(F_{x,\text{des}}, -F_{z,\text{des}}),\qquad
T_{\text{tot}} = \sqrt{F_{x,\text{des}}^{2}+F_{z,\text{des}}^{2}}
$$

裁剪 $\theta\in[\theta_{\min},\theta_{\max}]$ 后的实际：
$$
F_{x,\text{ach}} = T_{\text{tot}}\sin\theta,\qquad
F_{z,\text{ach}} = -T_{\text{tot}}\cos\theta
$$

若使用 PX4 “1D thrust” 模式（关闭 3D 推力），则 CA 仅看到 $F_z$ 行的 $T_{\text{tot}}$，$F_x$ 由轴向倾斜投影自然获得。

### 3.2 流程

1. 外环或调度层输出 $F_{x,\text{des}},F_{z,\text{des}}$
2. 几何反解 $(\theta, T_{\text{tot}})$ → 滤波 + 速率限制
3. 设定 tilt 伺服归一化指令（映射到 [-1,1]）
4. CA 输入：$F_z = T_{\text{tot}}$，$F_x = 0$（1D 模式）
5. CA 分配各电机推力幅值；执行器输出后几何形成 $F_x$

### 3.3 性质

- 不需要线性化；大范围 $\theta$ 行为一致
- $F_x/F_z = -\tan\theta$（统一倾角时为刚性比值）
- Pitch 解耦能力依赖你对 $\theta$ 的调度（无需机体整体俯仰即可产生前向力）

### 3.4 优缺点

| 优点 | 说明 |
|------|------|
| 简单鲁棒 | 仅用三角投影，对参数误差不敏感 |
| 低计算量 | 无偏导矩阵重构 |
| 易调参 | 只管理 $\theta$ 限幅/速率/滤波 |
| 平滑行为 | 大范围机动不需重建矩阵 |

| 缺点 | 说明 |
|------|------|
| 比值受限 | 任一时刻 $F_x/F_z$ 被单一 $\theta$ 固定 |
| 不能内部最优组合 | 不会“同时改推力与角度”满足多目标 |
| 扩展性一般 | 多独立倾角/能耗优化需改架构 |

---

## 4. 算法 B：小信号线性化增量分配

### 4.1 仿射近似模型

输出：
$$
y = \begin{bmatrix} M_x & M_y & M_z & F_x & F_y & F_z \end{bmatrix}^T
$$

工作点 $u_{\text{trim}}=\{f_i^*,\theta_i^*\}$ 线性化：
$$
y \approx y_{\text{trim}} + E\,\delta u,\qquad
\delta u = \begin{bmatrix} \delta f_1&\dots&\delta f_N&\delta\theta_1&\dots&\delta\theta_K \end{bmatrix}^T
$$

### 4.2 方向与偏导

轴向：
$$
a_i =
\begin{bmatrix}
\sin\theta_i\\ 0\\ -\cos\theta_i
\end{bmatrix}, \qquad
\frac{\partial a_i}{\partial \theta_i} =
\begin{bmatrix}
\cos\theta_i\\ 0\\ \sin\theta_i
\end{bmatrix}
$$

推力假设 $T_i = c_{t,i} f_i$。
线性化展开：$f_i=f_i^*+\delta f_i,\; \theta_i=\theta_i^*+\delta\theta_i$，保留一阶：
$$
\delta \mathbf{F} \approx \sum_i c_{t,i}\Big( a_i^*\,\delta f_i + f_i^* \frac{\partial a_i}{\partial \theta_i}\Big|_{\theta_i^*} \delta\theta_i \Big)
$$

得到（在工作点评估）：
$$
\frac{\partial \mathbf{F}}{\partial f_i} = c_{t,i} a_i^*,\qquad
\frac{\partial \mathbf{F}}{\partial \theta_i} = c_{t,i} f_i^* \frac{\partial a_i}{\partial \theta_i}\Big|_{\theta_i^*}
$$

力矩：
$$
\mathbf{M} = \sum_i c_{t,i} f_i \Big(\mathbf{r}_i \times a_i - k_{m,i} a_i\Big)
$$

倾角偏导：
$$
\frac{\partial \mathbf{M}}{\partial \theta_i} =
c_{t,i} f_i^* \Big(\mathbf{r}_i \times \frac{\partial a_i}{\partial \theta_i} - k_{m,i} \frac{\partial a_i}{\partial \theta_i}\Big)\Big|_{\theta_i^*}
$$

### 4.3 归一化伺服输入

伺服归一化 $u_{\theta,i}\in[-1,1]$：
$$
\theta_i = \theta_{i,\min} + \frac{u_{\theta,i}+1}{2}(\theta_{i,\max}-\theta_{i,\min}),\qquad
\frac{\partial \theta_i}{\partial u_{\theta,i}} = \frac{\theta_{i,\max}-\theta_{i,\min}}{2}
$$

链式法则列向量（对归一化伺服输入）：
$$
\frac{\partial y}{\partial u_{\theta,i}} = \frac{\partial y}{\partial \theta_i}\cdot \frac{\theta_{i,\max}-\theta_{i,\min}}{2}
$$

具体力与力矩部分：
$$
\frac{\partial \mathbf{F}}{\partial u_{\theta,i}} =
c_{t,i} f_i^* \frac{\partial a_i}{\partial \theta_i}\Big|_{\theta_i^*}
\frac{\theta_{i,\max}-\theta_{i,\min}}{2}
$$
$$
\frac{\partial \mathbf{M}}{\partial u_{\theta,i}} =
c_{t,i} f_i^*
\Big(\mathbf{r}_i \times \frac{\partial a_i}{\partial \theta_i} - k_{m,i} \frac{\partial a_i}{\partial \theta_i} \Big)\Big|_{\theta_i^*}
\frac{\theta_{i,\max}-\theta_{i,\min}}{2}
$$

> 如果推力真实为 $T_i=c_{t,i} f_i^2$，只需用 $T_i^*=c_{t,i} (f_i^*)^2$ 且 $\partial T_i/\partial f_i = 2 c_{t,i} f_i^*$ 替换。

### 4.4 运行流程

1. 计算 $\delta y = y_{\text{des}} - y_{\text{trim}}$
2. CA 求解 $\delta y \approx E\,\delta u$（顺序去饱和或伪逆）
3. 输出 $u = u_{\text{trim}} + \delta u$
4. 触发重线性化：$|\Delta\theta| > \epsilon_\theta$、$|\Delta f|/f^* > p$ 或超时

### 4.5 优缺点

| 优点 | 说明 |
|------|------|
| 更强即时解耦 | 可通过 $\delta\theta$ 增 $F_x$ 同时用 $\delta f$ 维持 $F_z$ |
| 可扩展优化 | 能耗、冗余、故障重构 |
| 灵活 | 允许不同电机不同倾角贡献 |

| 缺点 | 说明 |
|------|------|
| 模型敏感 | CT、几何、反扭矩误差影响偏导 |
| 复杂度高 | 需维护 $y_{\text{trim}},E$、阈值、列尺度 |
| 噪声/抖动风险 | 数值差分或参数跳变放大噪声 |
| 调参成本大 | 多个门限与滤波参数 |

---

## 5. 核心差异（聚焦 A(3) vs B）

| 维度 | A(3) 纯投影 | B 线性化增量 |
|------|-------------|--------------|
| $F_x$ 生成 | 几何投影 $(T,\theta)$ | 在线性方程里与 $F_z$ 联合求解 |
| $F_x/F_z$ 自由度 | 由统一 $\theta$ 决定 | 由 $E$ 中多列组合 |
| 模型依赖 | 低 | 中~高 |
| 大角度一致性 | 好 | 需频繁重线性化 |
| 噪声敏感度 | 低 | 可能高 |
| 实现/维护 | 低 | 高 |
| 扩展最优/容错 | 需重构 | 直接加层 |
| 初期风险 | 小 | 大 |

---

## 6. 选型建议

1. 初期 / 工程量产优先：先用 A(3)。
2. 采集日志：pitch 偏差、$F_x$ 跟踪误差、$F_z$ 饱和比、$\theta$ 利用率。
3. 若 pitch 摆动仍大或能耗冗余大，再试验 B（从小范围 $\theta$ 起）。

---

## 7. A(3) 伪代码

```c
// 输入: Fx_des, Fz_des
theta_cmd = atan2(Fx_des, -Fz_des);
theta_cmd = constrain(theta_cmd, theta_min, theta_max);
T_tot = sqrtf(Fx_des*Fx_des + Fz_des*Fz_des);

// 滤波 + 速率限制
theta_cmd_f = lpf(theta_cmd);
theta_cmd_f = rate_limit(theta_cmd_f);

// 映射伺服 [-1,1]
u_tilt = 2*(theta_cmd_f - theta_min)/(theta_max - theta_min) - 1;

// 1D thrust 模式
control_sp[FZ_AXIS] = T_tot;
control_sp[FX_AXIS] = 0.0f;

// 若轴向需更新:
if (fabs(theta_curr - theta_cmd_f) > update_threshold) { updateAxis(); dirty = true; }
```

---

## 8. B 伪代码

```c
// 已缓存 y_trim, E, u_trim
delta_y = y_des - y_trim;

// 控制分配
setControlSetpoint(delta_y);
allocate(E);              // 得到 delta_u

// 组合输出
for each motor i:
  f_i = f_i_trim + delta_f_i;
for each tilt j:
  theta_j = theta_j_trim + delta_theta_j;

// 重线性化条件
if (abs(theta_j - theta_j_trim) > dtheta_thresh ||
    abs(delta_f_i)/max(f_i_trim, eps) > df_rel_thresh ||
    time_since_linearize > timeout) {
  recompute_y_trim_and_E();
}
```

---

## 9. 多方向倾转扩展

若存在绕 $z$ 的侧向方向 $\phi_i$：
$$
a_i(\theta_i,\phi_i)=R_z(\phi_i)R_y(-\theta_i)\begin{bmatrix}0\\0\\-1\end{bmatrix}
$$
数值差分：
$$
\frac{\partial a_i}{\partial \theta_i}\approx
\frac{a_i(\theta_i+\varepsilon,\phi_i)-a_i(\theta_i-\varepsilon,\phi_i)}{2\varepsilon},\qquad
\frac{\partial a_i}{\partial \phi_i}\approx
\frac{a_i(\theta_i,\phi_i+\varepsilon)-a_i(\theta_i,\phi_i-\varepsilon)}{2\varepsilon}
$$

---

## 10. A(3) 稳定性指标（示例）

| 指标 | 定义 | 目标值（示例） |
|------|------|---------------|
| pitch_rms | 加速段俯仰均方根 | < 2° |
| Fx_err_ratio | $\|F_{x,\text{des}}-F_x\|/\|F_{x,\text{des}}\|$ | < 10% |
| Fz_sat_ratio | 推力指令 >0.95 占比 | < 15% |
| theta_rate_peak | 倾角速率峰值 | < 0.8 $\theta_{\text{rate\_limit}}$ |
| unallocated_norm | 未分配向量范数比 | < 5% |

---

## 11. 何时升级 B

- 要求大 $F_x$ 下保持高度扰动极低（严格 $F_z$）
- 需多执行器冗余/故障重构
- 计划做能耗或力矩最优分配研究
- 已完成精确推力/几何标定

---

## 12. “倾角偏导（评估于 $f_i^*$）” 说明

推力项 $c_{t,i} f_i a(\theta_i)$ 对 $\theta_i$ 的导数：$f_i$ 视为常量 → 在工作点取值 $f_i^*$。若 $f_i^*=0$，该电机的 $\partial \mathbf{F}/\partial \theta_i$ 一阶近似为 0，符合“无推力即使旋转轴向也不产生力增量”的物理直觉。

---

## 13. 决策流程（简图）

```
实现 A(3)
  ├─ 记录指标
  ├─ 满足性能 → 保持
  └─ 不满足
       ├─ 优化 θ 调度与外环
       └─ 仍不足 → 引入 B (窄范围) → 扩大范围
```

---

## 14. 压缩对照表

| 维度 | A(3) | B |
|------|------|---|
| 复杂度 | 低 | 高 |
| 解耦 | 中 | 高 |
| 模型依赖 | 低 | 中~高 |
| 大角度一致性 | 优 | 需刷新 |
| 抖动风险 | 低 | 中/高 |
| 扩展最优/容错 | 需改 | 原生 |
| 调参工作量 | 小 | 大 |

---

## 15. 总结

- **A(3)**：$(F_x,F_z)\to(T,\theta)$ 几何投影，简单、鲁棒、快速实现“平飞 + 前向推力”。
- **B**：基于偏导矩阵的增量协同分配，提供更高解耦与优化潜力，但增加模型、调参和噪声风险。
建议：先稳定落地 A(3)，用数据判断是否需要 B 的增益。

---
