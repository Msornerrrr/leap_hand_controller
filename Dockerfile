# LEAP hand controller: ROS Noetic + Python 3.10 + MCC compliance
#
# Build (from compliance/ root):
#   docker build -f leap_hand_controller/Dockerfile -t leap-compliance-mcc:latest .

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.10.14

# ── System toolchain ─────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl wget git \
    build-essential cmake pkg-config \
    python3-pip python3-dev \
    zlib1g-dev libssl-dev libffi-dev libbz2-dev libreadline-dev \
    libsqlite3-dev libncursesw5-dev libgdbm-dev liblzma-dev tk-dev uuid-dev xz-utils \
    usbutils sudo gnupg lsb-release \
    && rm -rf /var/lib/apt/lists/*

# ── Python 3.10 from source ─────────────────────────────────────────────────
RUN cd /tmp \
    && wget -q "https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz" \
    && tar -xzf "Python-${PYTHON_VERSION}.tgz" \
    && cd "Python-${PYTHON_VERSION}" \
    && ./configure --enable-shared --with-ensurepip=install \
    && make -j"$(nproc)" \
    && make altinstall \
    && ldconfig \
    && rm -rf /tmp/Python-${PYTHON_VERSION}*

RUN curl -sS https://bootstrap.pypa.io/get-pip.py | /usr/local/bin/python3.10
RUN ln -sf /usr/local/bin/python3.10 /usr/local/bin/python3 \
    && ln -sf /usr/local/bin/pip3.10 /usr/local/bin/pip3 \
    && ln -sf /usr/local/bin/python3.10 /usr/local/bin/python \
    && ln -sf /usr/local/bin/pip3.10 /usr/local/bin/pip

# ── ROS Noetic ───────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu focal main" \
       > /etc/apt/sources.list.d/ros-latest.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-noetic-ros-base \
       ros-noetic-sensor-msgs \
       ros-noetic-std-msgs \
       ros-noetic-geometry-msgs \
       ros-noetic-tf2-ros \
       ros-noetic-robot-state-publisher \
       ros-noetic-rviz \
       python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# ROS Python 3.10 compatibility: install rospkg/catkin/empy for python3.10
RUN python3.10 -m pip install --no-cache-dir rospkg catkin_pkg netifaces defusedxml empy==3.3.4

# ── MCC build deps ───────────────────────────────────────────────────────────
RUN python3.10 -m pip install --no-cache-dir --upgrade pip && \
    python3.10 -m pip install --no-cache-dir \
    scikit-build-core pybind11 cmake

# ── Runtime Python deps ──────────────────────────────────────────────────────
RUN python3.10 -m pip install --no-cache-dir \
    dynamixel-sdk joblib pyyaml tqdm

# ── MCC: install with C++ Dynamixel backend ──────────────────────────────────
COPY vendor/mcc /opt/mcc
WORKDIR /opt/mcc
RUN CMAKE_ARGS="-DBUILD_DYNAMIXEL=ON" python3.10 -m pip install --no-cache-dir -e .

# ── Catkin workspace with leap_hand + leap_description ──────────────────────
RUN mkdir -p /catkin_ws/src
COPY leap_hand /catkin_ws/src/leap_hand
COPY leap_description /catkin_ws/src/leap_description

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/local/bin/python3.10 -DCMAKE_POLICY_VERSION_MINIMUM=3.5"

# ── Copy entrypoint ─────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /opt/mcc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
