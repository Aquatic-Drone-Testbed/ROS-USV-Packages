FROM ros:humble
ARG USERNAME=seamate-docker
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip gpsd gpsd-clients python3-gps
# Install pigpio Python library
RUN pip3 install pigpio
RUN pip3 install gps
# For IMU
RUN apt-get install -y python3-smbus 
RUN groupadd -f -r i2c && \
    usermod -a -G i2c seamate-docker
ENV SHELL /bin/bash

# Copy scripts and set permissions
COPY /home/ws/scripts/build-ekf.sh /home/$USERNAME/scripts/build-ekf.sh
COPY /home/ws/scripts/build-imu.sh /home/$USERNAME/scripts/build-imu.sh
RUN chmod +x /home/$USERNAME/scripts/build-ekf.sh /home/$USERNAME/scripts/build-imu.sh

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER $USERNAME
CMD ["/bin/bash"]
