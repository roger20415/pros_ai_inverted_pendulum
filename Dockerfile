FROM registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_ai_gpu_image:latest
ENV ROS2_WS /workspaces
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO=humble
ARG THREADS=4
ARG TARGETPLATFORM

SHELL ["/bin/bash", "-c"]

##### Copy Source Code #####
COPY . /tmp

##### Environment Settings #####
WORKDIR /tmp

# System Upgrade and pip setup
RUN apt update && \
    apt upgrade -y && \
    apt autoremove -y && \
    apt autoclean -y && \
    pip3 install --no-cache-dir --upgrade pip

    ##### Install pip requirements #####
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt && \
    ##### Post-Settings #####
    # Clear tmp and cache
    rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS2_WS}
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash", "-l"]
