# 🐳 Omni-LIVO 全环境 Docker 使用说明

本说明文档适用于你已经构建好的 **omni-livo:ros-noetic** 镜像，集成了：

- ROS Noetic 全套  
- Omni-LIVO + FAST-LIVO2 环境  
- Seeker 相机依赖与驱动  
- Livox MID360 依赖（Livox-SDK2 + livox_ros_driver2 ROS1 模式）  
- RViz 支持（可选，需宿主机图形环境）  

---

## 1️⃣ 前置准备

### 1.1 设置 Docker 代理（国内建议）
确保代理软件允许「局域网连接」。

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d

cat <<'EOF' | sudo tee /etc/systemd/system/docker.service.d/proxy.conf
[Service]
Environment="HTTP_PROXY=http://192.168.31.210:7890"
Environment="HTTPS_PROXY=http://192.168.31.210:7890"
Environment="NO_PROXY=localhost,127.0.0.1,::1"
EOF

sudo systemctl daemon-reload
sudo systemctl restart docker

# 验证代理是否生效
systemctl show --property=Environment docker
```

⚠️ 将 `192.168.31.210:7890` 替换为你的本地代理地址。

---

## 2️⃣ 构建镜像

```bash
docker build --no-cache --network=host   --build-arg GIT_HTTP_PROXY=http://192.168.31.210:7890   --build-arg GIT_HTTPS_PROXY=http://192.168.31.210:7890   -t omni-livo:ros-noetic .
```

参数说明：

- `--no-cache`：避免构建中使用缓存  
- `--network=host`：防止因 DNS/网络问题拉取失败  
- `--build-arg`：传递构建时的 Git 代理（可选）  

---

## 3️⃣ 首次使用（创建容器）

### 3.1 创建持久容器（命令行版本）
```bash
docker run -it   --net=host   --privileged   -v /dev:/dev   --name omni-livo-dev   omni-livo:ros-noetic
```

### 3.2 创建持久容器（支持 RViz 图形界面）
宿主机先允许 X11：
```bash
xhost +local:root
```

然后运行：
```bash
docker run -it   --net=host   --privileged   --runtime nvidia   -e NVIDIA_VISIBLE_DEVICES=all   -e NVIDIA_DRIVER_CAPABILITIES=all   -e DISPLAY=$DISPLAY   -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $HOME/.Xauthority:/root/.Xauthority:ro   -e XAUTHORITY=/root/.Xauthority   -v /dev:/dev   --name omni-livo-dev   omni-livo:ros-noetic
```

(Jetson 如果是Ubuntu22.04)
```bash
FFI8_REAL=$(readlink -f /usr/lib/aarch64-linux-gnu/libffi.so.8)
echo "$FFI8_REAL"   # 例如：/usr/lib/aarch64-linux-gnu/libffi.so.8.1.0


docker run -it --net=host --privileged --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -e XAUTHORITY=/root/.Xauthority \
  -v /dev:/dev \
  -v /usr/lib/aarch64-linux-gnu/libffi.so.8:/usr/lib/aarch64-linux-gnu/libffi.so.8:ro \
  -v ${FFI8_REAL}:/usr/lib/aarch64-linux-gnu/$(basename ${FFI8_REAL}):ro \
  --name omni-livo-dev \
  omni-livo:ros-noetic

```

### 3.3 容器内初始化
```bash
# 如果需要代理（可选）
export http_proxy=http://192.168.31.210:7890
export https_proxy=http://192.168.31.210:7890

# 初始化 rosdep（只需一次）
rosdep init
rosdep update

# 安装依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 加载环境（Dockerfile 已写入 .bashrc）
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 安装 Seeker 解畸变包
dpkg -i ~/catkin_ws/src/seeker1/deb/ros-noetic-image-undistort_0.0.0-0focal_arm64.deb
```

### 3.4 Seeker 相机额外配置
在宿主机配置 udev 规则：
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0000", MODE="0666"' | sudo tee /etc/udev/rules.d/99-seeker.rules
sudo udevadm control --reload
sudo udevadm trigger
```
然后 **拔插一次相机**。

### 3.5 相机标定配置
生成标定文件：
```bash
python3 ~/catkin_ws/src/seeker1/script/1get_kalibr_info.py
```

输出：`/tmp/kalibr_cam_chain.yaml`

部署配置：
```bash
cp /tmp/kalibr_cam_chain.yaml ~/catkin_ws/src/seeker1/config/seeker_omni_depth/
```

---

## 4️⃣ 后续使用（复用同一个容器）

### 4.1 启动容器
容器已经创建过了，之后只需：
```bash
docker start -ai omni-livo-dev
```

### 4.2 多终端进入同一容器
```bash
docker exec -it omni-livo-dev bash
```

---

## 5️⃣ 常见工作流

### 启动 ROS 核心
```bash
roscore
```

### 运行 Livox MID360 驱动（ROS1 模式）
```bash
roslaunch livox_ros_driver2 msg_MID360.launch

```

### 运行 Seeker 
```bash
# 启动基础数据流（鱼眼图像 + 视差图 + IMU）
roslaunch seeker 1seeker_nodelet.launch  time_sync:=false

```

### 运行 Omni-LIVO
```bash
roslaunch fast_livo mapping_mid360.launch
```

### 播放数据集
```bash
rosbag play YOUR_DATASET.bag
```

---

## 6️⃣ 注意事项

- **USB 设备访问**  
  `--privileged -v /dev:/dev` 已确保容器能访问相机/雷达。  
  如果仍然遇到 `Resource busy`，需要解绑宿主机的内核驱动或使用 PyUSB detach。  

- **网络设置（MID360）**  
  默认静态 IP：`192.168.1.xxx`。请确保宿主机/容器在同一网段。  

- **环境加载**  
  Dockerfile 已写入 `.bashrc`：  
  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  ```
  保险起见，每个新终端可手动再执行一次。  

- **性能优化**  
  Jetson 平台推荐 `--runtime nvidia` 并挂载 GPU。  
  桌面平台需安装 **NVIDIA Container Toolkit** 以支持 GPU 加速。  
