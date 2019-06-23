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
CMD ["/ardupilot/Tools/autotest/sim_vehicle.py", "--console", "--map", "-D"]


# run with:
# docker run --rm -it --net=host -v /tmp/.X11-unix/X1:/tmp/.X11-unix/X1 -e DISPLAY=:1 greywing
#
# older versions of gnome would be X1 and :1
# see here for explanation:
# https://bugzilla.gnome.org/show_bug.cgi?id=747339#c20
