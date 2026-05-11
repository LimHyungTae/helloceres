ARG UBUNTU_VER=22.04
FROM ubuntu:${UBUNTU_VER}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    libceres-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .

RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

CMD ["bash", "scripts/run_tests.sh", "/workspace/build"]
