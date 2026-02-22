FROM ubuntu:24.04 as hakoniwa-mujoco-robots

ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=Asia/Tokyo \
    PIP_BREAK_SYSTEM_PACKAGES=1

ARG HAKO_CORE_PRO_REPO=https://github.com/hakoniwalab/hakoniwa-core-pro.git
ARG HAKO_CORE_PRO_REF=main

# Basic build/runtime packages for Ubuntu 24.04.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake make pkg-config \
    git curl wget unzip \
    ca-certificates \
    mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev \
    libglfw3-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    xvfb \
    jq \
    python3 python3-pip python3-dev \
 && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Clone, build and install official hakoniwa-core-pro (with submodules).
RUN git clone --depth 1 --branch "${HAKO_CORE_PRO_REF}" --recurse-submodules --shallow-submodules "${HAKO_CORE_PRO_REPO}" /tmp/hakoniwa-core-pro \
 && cmake -S /tmp/hakoniwa-core-pro \
      -B /tmp/hakoniwa-core-pro/cmake-build \
      -DCMAKE_INSTALL_PREFIX=/usr/local/hakoniwa \
      -DHAKO_ENABLE_GTEST=OFF \
      -DHAKO_DATA_MAX_ASSET_NUM=4 \
 && cmake --build /tmp/hakoniwa-core-pro/cmake-build -j"$(nproc)" \
 && cmake --install /tmp/hakoniwa-core-pro/cmake-build \
 && mkdir -p /var/lib/hakoniwa/mmap \
 && chmod 755 /var/lib/hakoniwa \
 && chmod 777 /var/lib/hakoniwa/mmap \
 && rm -rf /tmp/hakoniwa-core-pro

# Python dependencies used by auto-control scripts.
RUN pip3 install --no-cache-dir \
    numpy scipy \
    "aiohttp>=3.10.0" "aiohttp_cors>=0.7.0" "websockets==13.1" \
    hakoniwa-pdu

ENV PATH="/usr/local/hakoniwa/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/hakoniwa/lib"

WORKDIR /root/workspace

# Entrypoint for optional headless X (xvfb) execution.
COPY docker/entrypoint.sh /usr/local/bin/hako-entry
RUN chmod +x /usr/local/bin/hako-entry

ENTRYPOINT ["/usr/local/bin/hako-entry"]
CMD ["bash"]
