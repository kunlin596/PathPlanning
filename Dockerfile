FROM ubuntu:20.04

MAINTAINER Kun Lin "kun.lin.596@gmail.com"

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  build-essential\
  gdb\
  vim\
  cmake\
  git\
  python3-dev\
  pybind11-dev\
  libfmt-dev\
  libspdlog-dev\
  libboost-all-dev\
  libgtest-dev\
  libuv1-dev\
  libssl-dev\
  libz-dev

WORKDIR /opt

RUN mkdir -p build

# Build uWebSockets
RUN \
git clone https://github.com/uWebSockets/uWebSockets ./src/uWebSockets && \
cd ./src/uWebSockets && git checkout e94b6e1 && cd /opt && \
cmake -S ./src/uWebSockets/ -B ./build/uWebSockets && \
cmake -j16 --build ./build/uWebSockets/ && \
cd ./build/uWebSockets/ && make install -j16
RUN ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so

# Build planning server
COPY ./CMakeLists.txt ./src/pathplanningserver/
COPY ./cmakepatch.txt ./src/pathplanningserver/
COPY ./src/ ./src/pathplanningserver/src/

RUN \
cmake -DCMAKE_BUILD_TYPE=Release -S /opt/src/pathplanningserver/ -B /opt/build/pathplanningserver && \
cd /opt/build/pathplanningserver && make -j16

# Configs and data
COPY ./data/ ./data/
COPY ./default_conf.json ./default_conf.json

# Run
ENV PATH "${PATH}:/opt/build/pathplanningserver"
CMD ["pathplanningserver", "--conf", "/opt/default_conf.json", "--map", "/opt/data/highway_map.csv", "--loglevel", "info"]
