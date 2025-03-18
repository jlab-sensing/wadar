# lab computer's environment for consistency's sake
FROM ubuntu:22.04 

# essentials
RUN apt-get update && apt-get install -y \
  build-essential \
  curl \
  wget \
  git \
  vim \
  nano \
  sudo \
  && rm -rf /var/lib/apt/lists/*

# Install cross-compiler dependencies
RUN apt-get update && apt-get install -y \
  gcc-multilib \
  g++-multilib \
  lib32z1 \
  lib32stdc++6 \
  && rm -rf /var/lib/apt/lists/*


# Clone the repository
RUN git clone https://github.com/jlab-sensing/wadar.git /home/wadar

WORKDIR /home/wadar

# Default command
CMD ["bash"]