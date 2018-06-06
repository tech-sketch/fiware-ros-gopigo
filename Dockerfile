FROM ros:kinetic
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ARG additional="gopigo"

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh

RUN apt update && apt upgrade -y && \
    apt install build-essential libssl-dev libffi-dev python-dev python-pip python-setuptools -y

COPY . /opt/ws/src/fiware-ros-gopigo
RUN cd /opt && /bin/bash /opt/ws/src/fiware-ros-gopigo/update_tools_for_root.sh

WORKDIR /opt/ws

RUN source /opt/ros/kinetic/setup.bash && \
    /opt/ros/kinetic/bin/catkin_make && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ws/devel/setup.bash" >> /root/.bashrc && \
    cd /opt/ws/src/fiware-ros-gopigo && \
    pip install -r ./requirements/common.txt && \
    pip install -r ./requirements/${additional}.txt

RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

CMD ["/opt/ws/src/fiware-ros-gopigo/entrypoint.sh"]
