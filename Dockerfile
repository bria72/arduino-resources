FROM ubuntu:18.04

RUN apt-get update && apt-get -y upgrade && \
    apt-get install -y locales && \
    apt-get install -y tzdata && \
    apt-get install -y wget && \
    apt-get clean
    
RUN locale-gen de_DE.UTF-8
ENV LANG de_DE.UTF-8 
ENV LANGUAGE de_DE:de 
ENV LC_ALL de_DE.UTF-8

RUN ln -fs /usr/share/zoneinfo/Europe/Berlin /etc/localtime; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get install -y tzdata; \
    dpkg-reconfigure --frontend noninteractive tzdata

WORKDIR /tmp
RUN wget http://de.archive.ubuntu.com/ubuntu/pool/universe/n/newlib/libnewlib-dev_3.0.0.20180802-2_all.deb && \
    wget http://de.archive.ubuntu.com/ubuntu/pool/universe/n/newlib/libnewlib-arm-none-eabi_3.0.0.20180802-2_all.deb && \
    dpkg -i libnewlib-dev_3.0.0.20180802-2_all.deb libnewlib-arm-none-eabi_3.0.0.20180802-2_all.deb && \
    rm libnewlib-dev_3.0.0.20180802-2_all.deb libnewlib-arm-none-eabi_3.0.0.20180802-2_all.deb

RUN wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino5-x86_64-pc-linux-gnu.tar.bz2 && \
    wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
    
WORKDIR /opt/compiler
RUN tar -xf /tmp/avr-gcc-7.3.0-atmel3.6.1-arduino5-x86_64-pc-linux-gnu.tar.bz2 && \
    tar -xf /tmp/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    rm -rf /tmp/*
ENV PATH="${PATH}:/opt/compiler/avr/bin:/opt/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin"

WORKDIR /opt/ora-cc-rsc/
COPY ./ ./
ENV arduino_resources_dir=/opt/arduino-resources/

ENTRYPOINT ["/opt/ora-cc-rsc/compile_all.sh"]
