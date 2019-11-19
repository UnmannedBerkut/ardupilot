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
    ../Tools/autotest/sim_vehicle.py -wD < /dev/null && \
    pip install pygeodesy transforms3d

WORKDIR /ardupilot/ArduPlane

ENTRYPOINT ["/ardupilot/Tools/autotest/sim_vehicle.py", "-D", "--frame=plane-elevon-revthrust", "--add-param-file=../Greywing/SimSettings.param", "--out=172.17.0.1:14550"]
CMD ["--custom-location=32.70119,-117.25411,40,0"]
# run with:
# docker run --rm -it docker-mocu4.di2e.net/greywing
