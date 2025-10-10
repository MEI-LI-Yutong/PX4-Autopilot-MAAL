# 矢量推力：姿态期望与机体系 X/Z 推力期望生成逻辑

本文档描述：当飞行器具有“前后单轴可倾转”的矢量推力能力时，如何由外环（位置 / 速度控制）给出的世界系期望净力生成
1. 姿态期望 $ (\phi_{\text{sp}}, \theta_{\text{sp}}, \psi_{\text{sp}})$ （roll, pitch, yaw）
2. 机体系力期望 $\mathbf{F}_b^{\text{des}} = (F_{b,x}, F_{b,z})$（$F_{b,y}\approx 0$）
并把该机体系力交由 **控制分配 (Control Allocation)** 统一求解（而非直接转为固定推力+倾角）。

---

## 1. 坐标与符号

| 名称 | 定义 |
|------|------|
| 世界系 | NED：$+X$ 北，$+Y$ 东，$+Z$ 向下 |
| 机体系 | FRD：$+x$ 前，$+y$ 右，$+z$ 向下 |
| $\mathbf{F}_w \in \mathbb{R}^3$ | 世界系期望净力（含重力补偿），悬停时 $\mathbf{F}_w \approx [0,0,-mg]^\top$（NED） |
| $R_{\text{sp}}\in SO(3)$ | 最终姿态矩阵（world → body） |
| $\psi_{\text{sp}}, \theta_{\text{sp}}$ | 任务/规划提供的航向与俯仰 |
| $\phi_{\text{sp}}$ | 通过力几何自动解算的滚转 |
| $\mathbf{F}_b=[F_{b,x},F_{b,y},F_{b,z}]^\top$ | 机体系净力（$F_{b,z}>0$ 向下，向上为负） |
| 输出 | 姿态期望 $R_{\text{sp}}$（或四元数），力期望 $\mathbf{F}_b^{\text{des}}$ |

---

## 2. 输入与输出

输入：
- 世界系净力 $\mathbf{F}_w$
- 航向设定 $\psi_{\text{sp}}$
- 任务俯仰 $\theta_{\text{sp}}$
- 参数：滚转限幅 $\phi_{\max}$，数值阈值 $\varepsilon_y$

输出：
- 姿态期望（四元数或欧拉角）
- 机体系力 $\mathbf{F}_b^{\text{des}}$（主用 $F_{b,x}, F_{b,z}$）

---

## 3. 总体流程

1. 从外环得到 $\mathbf{F}_w$
2. 构造无 roll 的中间旋转
   $$R_{y\!p} = R_z(\psi_{\text{sp}}) R_y(\theta_{\text{sp}})$$
3. 变换力到“未滚转”机体系：
   $$\mathbf{F}_{b'} = R_{y\!p}^\top \mathbf{F}_w = \begin{bmatrix}F'_x\\F'_y\\F'_z\end{bmatrix}$$
4. 解算滚转 $\phi_{\text{sp}}$ 使最终 $F_{b,y}\approx 0$：
   $$\phi_{\text{sp}} = \operatorname{atan2}(-F'_y,\; F'_z)$$
   $$\phi_{\text{sp}} \leftarrow \operatorname{constrain}(\phi_{\text{sp}}, -\phi_{\max}, \phi_{\max})$$
5. 最终姿态：
   $$R_{\text{sp}} = R_z(\psi_{\text{sp}}) R_y(\theta_{\text{sp}}) R_x(\phi_{\text{sp}})$$
6. 机体系力：
   $$\mathbf{F}_b = R_{\text{sp}}^\top \mathbf{F}_w$$
7. 若 $\lvert F_{b,y}\rvert < \varepsilon_y$ 置 $F_{b,y}=0$
8. 发布姿态与 $\mathbf{F}_b^{\text{des}}$

---

## 4. 关键公式推导

### 4.1 中间力
$$
\mathbf{F}_{b'} = R_{y\!p}^\top \mathbf{F}_w,\quad
\mathbf{F}_{b'} = [F'_x, F'_y, F'_z]^\top
$$

### 4.2 滚转解耦 $F_{b,y}$

引入滚转：
$$
\mathbf{F}_b = R_x(\phi)^\top \mathbf{F}_{b'}
$$

其中
$$
R_x(\phi)^\top =
\begin{bmatrix}
1 & 0 & 0\\
0 & \cos\phi & \sin\phi\\
0 & -\sin\phi & \cos\phi
\end{bmatrix}
$$

第二分量：
$$
F_{b,y} = \cos\phi\, F'_y + \sin\phi\, F'_z
$$

令 $F_{b,y}=0$：
$$
\tan\phi = -\frac{F'_y}{F'_z}
\quad\Rightarrow\quad
\phi_{\text{sp}}=\operatorname{atan2}(-F'_y, F'_z)
$$

限幅：
$$
\phi_{\text{sp}} =
\begin{cases}
-\phi_{\max}, & \phi_{\text{sp}} < -\phi_{\max}\\
\phi_{\text{sp}}, & |\phi_{\text{sp}}|\le \phi_{\max}\\
\phi_{\max}, & \phi_{\text{sp}} > \phi_{\max}
\end{cases}
$$

### 4.3 最终力计算
$$
\mathbf{F}_b = R_{\text{sp}}^\top \mathbf{F}_w
$$

数值清理：
$$
|F_{b,y}| < \varepsilon_y \;\Rightarrow\; F_{b,y}=0
$$

---

## 5. 伪代码

```c++
void computeVectoredThrustSetpoints(const Vector3f &F_w,
                                    float yaw_sp,
                                    float pitch_sp,
                                    vehicle_attitude_setpoint_s &att_sp,
                                    Vector3f &F_b_des)
{
    Dcmf Rz(Eulerf(0.f, 0.f, yaw_sp));
    Dcmf Ry(Eulerf(0.f, pitch_sp, 0.f));
    Dcmf R_yawpitch = Rz * Ry;

    Vector3f F_bp = R_yawpitch.transpose() * F_w; // F' = [F'_x, F'_y, F'_z]

    float roll_sp = atan2(-F_bp(1), F_bp(2));
    roll_sp = math::constrain(roll_sp, -_roll_lim, _roll_lim);

    Dcmf R_sp(Eulerf(roll_sp, pitch_sp, yaw_sp));
    Vector3f F_b = R_sp.transpose() * F_w;

    if (fabsf(F_b(1)) < 1e-4f) F_b(1) = 0.f;

    Quatf q_sp(R_sp);
    q_sp.copyTo(att_sp.q_d);
    att_sp.roll_body  = roll_sp;
    att_sp.pitch_body = pitch_sp;
    att_sp.yaw_body   = yaw_sp;

    att_sp.thrust_body[0] = NAN;
    att_sp.thrust_body[1] = NAN;
    att_sp.thrust_body[2] = NAN;

    F_b_des = F_b;
}
```

对应数学：
$$
\mathbf{F}_{b'} = R_{y\!p}^\top \mathbf{F}_w,\quad
\phi_{\text{sp}} = \operatorname{atan2}(-F'_y, F'_z),\quad
\mathbf{F}_b = R_{\text{sp}}^\top \mathbf{F}_w
$$

---

## 6. 控制分配接口建议

发布消息（示例）：
```
fx = F_{b,x},\quad fy = F_{b,y},\quad fz = F_{b,z}
```

姿态控制器输出期望力矩 $\mathbf{M}_{\text{des}}=[M_x,M_y,M_z]^\top$。
合并：
$$
\mathbf{w}_{\text{des}} =
\begin{bmatrix}
F_{b,x} \\ F_{b,y} \\ F_{b,z} \\ M_x \\ M_y \\ M_z
\end{bmatrix}
$$

控制分配优化：
$$
\min_{\mathbf{u}}\ \| W(\mathbf{w}_{\text{des}} - \mathbf{w}(\mathbf{u})) \|_2^2
\quad \text{s.t.}\quad \mathbf{u}_{\min} \le \mathbf{u} \le \mathbf{u}_{\max}
$$

---

## 7. 与传统多旋翼的差异

| 项目 | 传统 | 矢量推力方案 |
|------|------|--------------|
| 推力方向 | 固定 $-\mathbf{z}_{\text{body}}$ | 可在 $x$–$z$ 平面偏转 |
| 姿态求解 | 推力方向 + yaw | yaw/pitch 任务 + roll 解耦 |
| 推力输出 | 标量 thrust | 向量 $\mathbf{F}_b$ |
| 分配 | 简单混控 | Control Allocation |
| 冗余利用 | 弱 | 可保持姿态同时施力 |

---

## 8. 限制与边界

1. $F'_z \approx 0$ 时 $\phi_{\text{sp}}$ 逼近 $\pm 90^\circ$，需限幅与平滑。
2. 低推力：倾转与滚转可辨识度下降，必要时冻结。
3. 超出可行锥或幅值限制：投影
   $$\mathbf{F}_b^{\text{proj}} = \arg\min_{\mathbf{F}\in\mathcal{C}}\|\mathbf{F} - \mathbf{F}_b^{\text{des}}\|_2$$
4. 残差回馈防积分：
   $$\mathbf{r}_F = \mathbf{F}_b^{\text{des}} - \mathbf{F}_b^{\text{real}}$$

---

## 9. 残差与抗饱和

若发生饱和或 $|F_{b,y}|>\varepsilon_y$：
$$
\mathbf{r}_F =
\begin{bmatrix}
0 \\ F_{b,y} \\ 0
\end{bmatrix}
\quad\text{或}\quad
\mathbf{r}_F = \mathbf{F}_b^{\text{des}} - \mathbf{F}_b^{\text{proj}}
$$

积分器：
$$
\dot{\mathbf{I}} = K_i(\mathbf{e}_v - K_{\text{aw}}\mathbf{r}_F)
$$

---

## 10. 示例

设 $m=2\text{ kg},\ g=9.81\text{ m/s}^2$
悬停：$\mathbf{F}_w=[0,0,-19.62]^\top$
给定：$\psi_{\text{sp}}=0,\ \theta_{\text{sp}}=5^\circ = 0.0873\text{ rad}$

\[
\mathbf{F}_{b'} =
\begin{bmatrix}
-19.62\sin(0.0873) \\ 0 \\ -19.62\cos(0.0873)
\end{bmatrix}
\approx
\begin{bmatrix}
-1.71 \\ 0 \\ -19.47
\end{bmatrix}
\]

\[
\phi_{\text{sp}}=\operatorname{atan2}(-0,-19.47)\approx 0
\]

\[
\mathbf{F}_b \approx [-1.71,\ 0,\ -19.47]^\top
\]

---

## 11. 双轴矢量扩展（概念）

若云台可绕 $x,y$ 双轴（角 $\alpha,\beta$）：
1. 任务姿态 $R_{\text{task}}$ 与力解耦
2. $\mathbf{F}_b = R_{\text{task}}^\top \mathbf{F}_w$
3. 方向：$\mathbf{u}_t = \mathbf{F}_b / \|\mathbf{F}_b\|$
4. 幅值：$T=\|\mathbf{F}_b\|$
5. 超出锥角 $\theta_{\max}$ 时投影

---

## 12. 实施建议

- 日志：$\mathbf{F}_w,\mathbf{F}_{b'},\phi_{\text{sp}},\mathbf{F}_b, F_{b,y}$
- 模式参数：`VT_EN`
- 单元测试：随机 $\mathbf{F}_w$ 验证 $F_{b,y}\to 0$
- HIL：加侧风，验证饱和与残差反馈

---

## 13. 退化策略

| 情况 | 处理 |
|------|------|
| $F'_z \to 0$ 且 $|F'_y|$ 大 | $\phi_{\text{sp}}=\text{sign}(-F'_y)\phi_{\max}$ 并标记 |
| 推力很小 | 冻结 $\phi_{\text{sp}}$ |
| 分配失败 | 回退：仅垂向 $\mathbf{F}_b^{\text{des}}=[0,0,F_{b,z}]^\top$ |

---

## 14. 接口改动清单（参考）

- 新增 uORB：`vehicle_force_setpoint`
- mc_pos_control：添加矢量推力分支
- mc_att_control：若 thrust 为 NaN → 仅输出力矩
- Control Allocation：新增矢量推力效能类
- Mixer：矢量模式禁用旧混控

---

## 15. Checklist

| 项 | 状态 |
|----|------|
| 公式实现 |  |
| 滚转限幅/滤波 |  |
| 残差反馈 |  |
| Force SP 发布 |  |
| Allocation 集成 |  |
| 饱和投影 |  |
| 日志/测试 |  |
| 模式切换/回退 |  |

---

## 16. 核心要点

- 固定任务 $(\psi,\theta)$，滚转 $\phi$ 由几何解耦获得
- 机体系力：$\mathbf{F}_b = R_{\text{sp}}^\top \mathbf{F}_w$
- 单轴倾转：目标 $F_{b,y}\approx 0$，力分量交由控制分配
- 充分处理饱和、低推力、水平极端场景与残差反馈
- 为双轴/更高自由度扩展预留路径

---

需要更进一步（如效能矩阵推导、双轴扩展细节）可继续提出。
