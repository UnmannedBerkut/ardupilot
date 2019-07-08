FROM ubuntu:18.04

ENV USER "root"

RUN \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get install -y sudo git apt-utils lsb-release tzdata && \
    ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime && \
    git clone https://github.com/UnmannedBerkut/ardupilot.git && \
    cd ardupilot && \
    git checkout Greywing && \
    cd /ardupilot/Tools/scripts/ && \
    bash install-prereqs-ubuntu.sh -y && \
    cd /ardupilot/ArduPlane/ && \
    ../Tools/autotest/sim_vehicle.py -wD < /dev/null

WORKDIR /ardupilot/ArduPlane

CMD ["/ardupilot/Tools/autotest/sim_vehicle.py", "--console", "--map", "-D", "--custom-location=32.70119,-117.25411,40,0", "--add-param-file=../Greywing/SimSettings.param"]


# run with:
#
# docker run \
#     --rm -it \
#     --net=host \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -e DISPLAY=$DISPLAY \
#     docker-mocu4.di2e.net/greywing
