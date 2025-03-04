FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Additional setup
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
  wget \
  git \
  libxext-dev \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  gnuplot \
  zlib1g-dev \
  libblas-dev \
  liblapack-dev \
  python3-pip \
  ros-humble-usb-cam \
  ros-humble-aruco-opencv

ENV QT_X11_NO_MITSHM=1

# Add a non-root user
ARG USERNAME=467-terp
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV USER=$USERNAME
ENV ROS_DISTRO=humble

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && printf "\nif [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Install dependencies for lab packages
USER $USERNAME
RUN git clone https://github.com/ENEE467/lab-workspace.git /home/$USERNAME/tmp_ws \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && (cd /home/$USERNAME/tmp_ws; rosdep update && rosdep install --from-paths src -y --ignore-src) \
  && rm -rf /home/$USERNAME/tmp_ws

# Install Matplot++
USER root
RUN git clone https://github.com/alandefreitas/matplotplusplus.git /home/$USERNAME/matplotplusplus \
  && (cd /home/$USERNAME/matplotplusplus; cmake --preset=system && cmake --build --preset=system && cmake --install build/system) \
  && rm -rf /home/$USERNAME/matplotplusplus

ENV DEBIAN_FRONTEND=
