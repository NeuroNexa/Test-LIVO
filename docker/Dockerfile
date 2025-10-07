FROM arm64v8/ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
LABEL maintainer="wenjialong"

# 构建期代理（可选）
ARG GIT_HTTP_PROXY=""
ARG GIT_HTTPS_PROXY=""

# 1) 换源 + ROS 源
RUN sed -i "s/ports.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/" /etc/apt/sources.list && \
    sed -i "/security/d" /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends ca-certificates gnupg dirmngr && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros1-latest.list

# 2) 系统与 ROS 依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales tzdata build-essential cmake git wget curl pkg-config \
      ros-noetic-desktop-full \
      ros-noetic-pcl-ros libpcl-dev libeigen3-dev libopencv-dev \
      ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-image-proc \
      ros-noetic-libg2o libceres-dev libgoogle-glog-dev libsuitesparse-dev \
      libpcap-dev libgeographic-dev ros-noetic-serial ros-noetic-gtsam \
      ros-noetic-catkin ros-noetic-jsk-rviz-plugins \
      ros-noetic-compressed-image-transport \
      python3-pip python3-rosdep libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# 3) Python 包
RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip install --no-cache-dir catkin-tools pyusb numpy ruamel.yaml

# 4) ROS 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 5) 工作区
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src

# 6) Sophus（固定提交 + aarch64/gcc9 修正）
RUN cd /opt && rm -rf Sophus && \
    git -c http.proxy=${GIT_HTTP_PROXY} -c https.proxy=${GIT_HTTPS_PROXY} \
        clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout a621ff2e || git checkout a621ff && \
    sed -i 's/unit_complex_\.real()[[:space:]]*=[[:space:]]*1\.[[:space:]]*;/unit_complex_.real(1.);/' sophus/so2.cpp && \
    sed -i 's/unit_complex_\.imag()[[:space:]]*=[[:space:]]*0\.[[:space:]]*;/unit_complex_.imag(0.);/' sophus/so2.cpp && \
    cmake -S . -B build && cmake --build build -j"$(nproc)" && cmake --install build && ldconfig

# 7) Livox SDK2
RUN cd /opt && rm -rf Livox-SDK2 && \
    git -c http.proxy=${GIT_HTTP_PROXY} -c https.proxy=${GIT_HTTPS_PROXY} \
        clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cmake -S /opt/Livox-SDK2 -B /opt/Livox-SDK2/build && \
    cmake --build /opt/Livox-SDK2/build -j"$(nproc)" && \
    cmake --install /opt/Livox-SDK2/build && ldconfig

# 8) 拉取各包（更新：livox_ros_driver2 和 seeker1 均来自 Omni-LIVO/src）
RUN cd ${CATKIN_WS}/src && \
    git -c http.proxy=${GIT_HTTP_PROXY} -c https.proxy=${GIT_HTTPS_PROXY} \
        clone https://github.com/NeuroNexa/Test-LIVO.git && \
    mv Test-LIVO/src/livox_ros_driver2 ./livox_ros_driver2 && \
    mv Test-LIVO/src/seeker1 ./seeker1 && \
    git -c http.proxy=${GIT_HTTP_PROXY} -c https.proxy=${GIT_HTTPS_PROXY} \
        clone https://github.com/skohlbr/disparity_image_proc.git

# 9) —— 关键：两阶段构建 —— 
# 9.1 先只构建 livox_ros_driver2（用官方脚本）
RUN set -e && cd ${CATKIN_WS}/src && \
    touch Test-LIVO/CATKIN_IGNORE && \
    touch seeker1/CATKIN_IGNORE && \
    touch disparity_image_proc/CATKIN_IGNORE && \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash && cd ${CATKIN_WS}/src/livox_ros_driver2 && ./build.sh ROS1"

# 9.2 解除屏蔽，再编译整个工作区（此时已存在 livox_ros_driver2 的消息与 cmake 配置）
RUN set -e && cd ${CATKIN_WS}/src && \
    rm -f Test-LIVO/CATKIN_IGNORE seeker1/CATKIN_IGNORE disparity_image_proc/CATKIN_IGNORE && \
    /bin/bash -lc "\
      source /opt/ros/noetic/setup.bash; \
      cd ${CATKIN_WS}; \
      catkin_make -DCMAKE_BUILD_TYPE=Release \
    "

# 10) 环境写入
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# 11) 清理
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]

