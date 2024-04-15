FROM ros:humble
ARG USERNAME=seamate-docker
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG VIDEO_GID=44
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && if getent group video ; then echo "Group video already exists"; else groupadd --gid $VIDEO_GID video; fi \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -G video \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip gpsd gpsd-clients python3-gps
# Install pigpio Python library
RUN pip3 install pigpio
RUN pip3 install gps

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]