# Control Allocator 修改总结

## 修改概述
基于用户提供的控制分配模型，对 Control Allocator 模块进行了修改，实现了自定义的控制分配算法。

## 修改内容

### 1. 头文件修改 (ControlAllocator.hpp)
- 添加了 `#include <uORB/topics/utrim.h>` 以支持 utrim 消息
- 添加了 `uORB::Subscription _utrim_sub{ORB_ID(utrim)};` 用于订阅 utrim 消息
- 添加了 `void calculate_custom_allocation();` 函数声明

### 2. 实现文件修改 (ControlAllocator.cpp)
- 在 `Run()` 函数中添加了对 `calculate_custom_allocation()` 的调用
- 实现了 `calculate_custom_allocation()` 函数，包含以下逻辑：

#### 控制分配模型
实现了用户提供的数学模型：

$$\begin{bmatrix} df_x \\ df_z \\ d\tau_x/L_3 \\ d\tau_y/L_1 \\ d\tau_z/L_3 \end{bmatrix}= \begin{bmatrix} \sin\theta_1 & \sin\theta_2 & \sin\theta_3 & \cos\theta_1 & \cos\theta_2 & \cos\theta_3 \\ -\cos\theta_1 & -\cos\theta_2 & -\cos\theta_3 & \sin\theta_1 & \sin\theta_2 & \sin\theta_3 \\ -\cos\theta_1 & \cos\theta_2 & 0 & \sin\theta_1 & -\sin\theta_2 & 0 \\ \cos\theta_1 & \cos\theta_2 & -\left(L_2/L_1\right)\cos\theta_3 & -\sin\theta_1 & -\sin\theta_2 & \left(L_2/L_1\right)\sin\theta_3 \\ -\sin\theta_1 & \sin\theta_2 & 0 & -\cos\theta_1 & \cos\theta_2 & 0 \end{bmatrix} \begin{bmatrix} df_1 \\ df_2 \\ df_3 \\ df_1d\theta_1 \\ f_2d\theta_2 \\ f_3d\theta_3 \end{bmatrix}$$

#### 实现细节
1. **输入获取**：
   - 从 `_thrust_sp(0)` 和 `_thrust_sp(2)` 获取 fx 和 fz
   - 从 utrim 消息的 `polynomial_values[0-2]` 获取 θ1、θ2、θ3

2. **常量定义**：
   - L1 = 1.0f
   - L2 = 1.0f  
   - L3 = 1.0f

3. **矩阵构建**：
   - 构建 5x6 的效率矩阵 A
   - 构建 5x1 的左侧向量 b (fx, fz, 0, 0, 0)

4. **求解算法**：
   - 使用伪逆方法：x = A^T * (A * A^T)^{-1} * b
   - 计算右侧向量 [df1, df2, df3, df1_dtheta1, df2_dtheta2, df3_dtheta3]

5. **输出日志**：
   - 记录输入参数 (fx, fz, θ1, θ2, θ3)
   - 记录计算结果向量的所有6个分量

## 功能说明
- 每个控制循环都会调用自定义分配算法
- 通过 `PX4_INFO` 输出详细的计算结果
- 采用MVP原则，仅实现核心功能和日志输出
- 使用伪逆方法求解超定方程组

## 验证
代码已经实现并可以通过 `make px4_sitl_default` 命令进行编译验证。