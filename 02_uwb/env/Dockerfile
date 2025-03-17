# x86 Debian so we're doing 86-linux -> ARM linux 
FROM amd64/debian:10
 
# non-interactive mode, because I see this in many Dockerfiles
ENV DEBIAN_FRONTEND=noninteractive

# since debian 8.7 is no longer supported, we need to change the sources.list
# RUN rm /etc/apt/sources.list
# RUN echo "deb [trusted=yes] http://archive.debian.org/debian-security jessie/updates main" >> /etc/apt/sources.list.d/jessie.list
# RUN echo "deb [trusted=yes] http://archive.debian.org/debian jessie main" >> /etc/apt/sources.list.d/jessie.list

# cross-compiling tools for ARM and other needed packages
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    gcc-arm-linux-gnueabihf \   
    binutils-arm-linux-gnueabihf \  
    g++-arm-linux-gnueabihf \    
    git \
    sudo \
    make \
    nano \
    gcc \
    && apt-get clean
WORKDIR /root
CMD ["bash"]

# Clone the repository
RUN git clone https://github.com/jlab-sensing/wadar.git /root/wadar

