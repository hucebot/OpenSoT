FROM osrf/ros:jazzy-simulation

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

RUN apt-get update && apt-get upgrade -y && apt-get clean  && apt-get install -y --no-install-recommends terminator gedit locate cmake-curses-gui python3-pip python3-venv liburdfdom-dev ros-jazzy-moveit-core clang python3-dbg libxcb-cursor0 gdb \
 ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui libglpk-dev libopenblas-dev ros-jazzy-rosidl-generator-dds-idl && \
 rm -rf /var/lib/apt/lists/*

WORKDIR /home
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /home/forest_ws

ENV PYTHONUNBUFFERED=1

COPY templates/setup.bash /home/forest_ws/setup.bash
RUN echo "source $PWD/setup.bash" >> ~/.bashrc

# COAL
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/coal-library/coal.git && \
    cd /home/forest_ws/src/coal && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/coal && \
    cd /home/forest_ws/build/coal && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF -DBUILD_DOCUMENTATION=OFF ../../src/coal && \
    make -j$(nproc)  && \
    make install &&  rm -r /home/forest_ws/build/coal /home/forest_ws/src/coal

## PINOCCHIO
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/stack-of-tasks/pinocchio.git && \
    cd /home/forest_ws/src/pinocchio && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/pinocchio && \
    cd /home/forest_ws/build/pinocchio && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_TESTING=FALSE -DBUILD_PYTHON_INTERFACE=OFF ../../src/pinocchio && \
    make -j$(nproc)  && \
    make install &&  rm -r /home/forest_ws/build/pinocchio /home/forest_ws/src/pinocchio
    

# xbot2_interface
WORKDIR /home/forest_ws/src
RUN git clone -b devel https://github.com/hucebot/xbot2_interface.git && \   
    mkdir -p /home/forest_ws/build/xbot2_interface && \
    cd /home/forest_ws/build/xbot2_interface && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DXBOT2_IFC_BUILD_TESTS=OFF -DXBOT2_IFC_BUILD_ROS=OFF -DXBOT2_IFC_BUILD_ROS2=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/xbot2_interface && \
    make -j$(nproc)  && \
    make install &&  rm -r /home/forest_ws/build/xbot2_interface /home/forest_ws/src/xbot2_interface
    
# osqp
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/oxfordcontrol/osqp.git && \
    cd /home/forest_ws/src/osqp && \
    git checkout 0b34f2ef5c5eec314e7945762e1c8167e937afbd && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/osqp && \
    cd /home/forest_ws/build/osqp && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DDLONG=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/osqp && \
    make -j$(nproc)  && \
    make install &&  rm -r /home/forest_ws/build/osqp /home/forest_ws/src/osqp

# proxQP
RUN git clone https://github.com/Simple-Robotics/proxsuite.git && \
    cd /home/forest_ws/src/proxsuite && \
    git checkout 9f58ebd7bf42fb364de995d25679dc3f675dee7f && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/proxsuite && \
    cd /home/forest_ws/build/proxsuite && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/proxsuite && \
    make -j$(nproc)  && \
    make install &&  rm -r /home/forest_ws/build/proxsuite /home/forest_ws/src/proxsuite
    
# qpSWIFT
RUN git clone https://github.com/qpSWIFT/qpSWIFT.git && \
    mkdir -p /home/forest_ws/build/qpSWIFT && \
    cd /home/forest_ws/build/qpSWIFT && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/qpSWIFT && \
    make -j$(nproc)  && \
    make install &&  rm -r /home/forest_ws/build/qpSWIFT /home/forest_ws/src/qpSWIFT

# BLASFEO
RUN git clone https://github.com/giaf/blasfeo && \
    mkdir -p /home/forest_ws/build/blasfeo && \
    cd /home/forest_ws/build/blasfeo && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_SHARED_LIBS=ON -DBLASFEO_EXAMPLES=OFF ../../src/blasfeo && \
    make -j$(nproc)  && \
    make install && \
    mkdir /home/forest_ws/install/share/cmake/lib && \
    cp /home/forest_ws/install/lib/libblasfeo.so /home/forest_ws/install/share/cmake/lib && \
    rm -r /home/forest_ws/build/blasfeo /home/forest_ws/src/blasfeo

RUN echo "export PYTHONPATH=${PYTHONPATH}:/root/.local/lib/python3.12/site-packages:/usr/lib/python3/dist-packages/" >> ~/.bashrc

RUN cd /home/forest_ws/src && git clone https://github.com/ADVRHumanoids/MatLogger2.git && \
 cd /home/forest_ws/build && mkdir MatLogger2 && cd MatLogger2 && \
 cmake ../../src/MatLogger2/ -DCPACK_PACKAGING_INSTALL_PREFIX=/opt/xbot -DPYBIND11_PYTHON_VERSION=3 -DPYTHON_EXECUTABLE=$(readlink -f $(which python3)) -DPYTHON_VERSION=3 -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release && \
 make -j 4 && make install &&  rm -r /home/forest_ws/build/MatLogger2  /home/forest_ws/src/MatLogger2

WORKDIR /home/forest_ws
