# Docker 容器跨机器运行故障排除指南

## 问题描述

当 Docker 镜像在不同机器上运行时，可能会遇到飞控无法 arm 的问题，典型错误包括：

```
Preflight Fail: system power unavailable
Preflight Fail: ekf2 missing data
heartbeats timed out
```

## 根本原因

这些错误**不是由显示相关参数引起的**，而是因为：

1. **渲染引擎兼容性**：不同机器的 GPU 驱动和 OpenGL 版本不同
2. **Gazebo 仿真器启动失败**：即使在 headless 模式下，Gazebo 仍需要渲染引擎
3. **传感器桥接失败**：PX4 与 Gazebo 之间的数据通信中断

## 解决方案

### 方法 1：使用更新后的脚本（推荐）

我们已经更新了启动脚本，添加了更好的跨平台兼容性：

```bash
# 使用默认配置（推荐）
./Tools/px4_gust_eval/run_gust_eval_container.sh

# 或手动指定渲染引擎
PX4_GZ_SIM_RENDER_ENGINE=ogre ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

**新增的关键配置：**
- `--privileged`：提供完整的设备访问权限
- `--network host`：使用主机网络栈
- `-v /dev:/dev`：挂载设备目录
- `PX4_GZ_SIM_RENDER_ENGINE=ogre`：使用兼容性更好的 OGRE 1.x 渲染引擎
- `LIBGL_ALWAYS_SOFTWARE=1`：强制使用软件渲染（不依赖 GPU）

### 方法 2：手动 Docker 命令

如果需要更多控制，可以直接使用 docker run：

```bash
sudo docker run --rm -it \
  --privileged \
  --network host \
  --ipc=host \
  -e HEADLESS=1 \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  -e QT_QPA_PLATFORM=offscreen \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /dev:/dev \
  px4-gust-eval
```

### 方法 3：在容器内调试

进入容器手动检查问题：

```bash
# 启动交互式容器
sudo docker run --rm -it \
  --privileged \
  --network host \
  -e HEADLESS=1 \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  px4-gust-eval /bin/bash

# 在容器内执行
export HEADLESS=1
export GZ_SIM_HEADLESS=1
export QT_QPA_PLATFORM=offscreen
export PX4_GZ_SIM_RENDER_ENGINE=ogre
export LIBGL_ALWAYS_SOFTWARE=1

# 检查 Gazebo 版本
gz sim --versions

# 测试 Gazebo 启动
gz sim -v 4 -s -r /path/to/world.sdf

# 手动启动 PX4 SITL
cd /PX4-Autopilot
make px4_sitl gz_tiltrotor_windy
```

## 环境变量说明

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `HEADLESS` | `1` | 是否以 headless 模式运行（1=是，0=否） |
| `PX4_GZ_SIM_RENDER_ENGINE` | `ogre` | Gazebo 渲染引擎（ogre/ogre2） |
| `QT_QPA_PLATFORM` | `offscreen` | Qt 平台插件（headless 时使用 offscreen） |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | 强制软件渲染（1=是，0=否） |
| `GZ_SIM_HEADLESS` | `1` | Gazebo 原生 headless 标志 |

## 常见问题排查

### 1. 检查 Gazebo 是否正常启动

```bash
# 查看容器内进程
docker exec -it <container_id> ps aux | grep gz

# 查看 Gazebo 日志
docker logs <container_id> 2>&1 | grep -i "gazebo\|gz"
```

### 2. 检查 PX4 日志

```bash
# 在容器内查看 PX4 日志
tail -f /PX4-Autopilot/log/*/ulg_*.log
```

### 3. 测试渲染引擎

```bash
# 在容器内测试不同渲染引擎
PX4_GZ_SIM_RENDER_ENGINE=ogre gz sim --version
PX4_GZ_SIM_RENDER_ENGINE=ogre2 gz sim --version
```

### 4. 检查 OpenGL 支持

```bash
# 在容器内检查
glxinfo | grep "OpenGL"
# 或
LIBGL_DEBUG=verbose glxinfo 2>&1 | head -20
```

## 不同场景的推荐配置

### 场景 1：带显示器的开发机器（GUI 模式）

```bash
sudo docker run --rm -it \
  --network host \
  --ipc=host \
  -e HEADLESS=0 \
  -e DISPLAY=${DISPLAY} \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  px4-gust-eval

# 运行前授权 X11 访问
xhost +local:docker
```

### 场景 2：无显示器的服务器（Headless 模式）

```bash
sudo docker run --rm -it \
  --privileged \
  --network host \
  -e HEADLESS=1 \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  px4-gust-eval
```

### 场景 3：虚拟机环境

```bash
# 虚拟机通常需要软件渲染
sudo docker run --rm -it \
  --privileged \
  --network host \
  -e HEADLESS=1 \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  px4-gust-eval
```

## 验证修复

成功启动后，您应该看到类似以下的日志：

```
INFO  [init] Gazebo simulator
INFO  [init] Starting gazebo with world: /path/to/world.sdf
INFO  [init] Gazebo world is ready
INFO  [init] Spawning model
INFO Health: armable=True gps_ok=True home_ok=True
```

## 更新镜像

如果修改了脚本，需要重新构建镜像：

```bash
# 构建新镜像
docker build -t px4-gust-eval .

# 或使用您的远程镜像名称
docker build -t raiots/maal_px4_simulation:latest .
docker push raiots/maal_px4_simulation:latest
```

## 参考资料

- [PX4 Gazebo 仿真文档](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)
- [Docker 最佳实践](https://docs.docker.com/develop/dev-best-practices/)

