# How to build docker image:
# Windows CMD: docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
# Windows PS: Get-Content Dockerfile | docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
# Linux: docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
#
# How to publish docker image:
# docker push aruw/mcb-2020-gitlab-ci:2020-02-17.1

FROM python:3.8.0-slim-buster
# We do all installations in a single layer and clean up residual files in one
# pass to minimize Docker image size (and thus, CI build time).

# Install misc basic dependencies
# Installing gnupg2 for "apt-key add"
RUN apt-get update \
    && apt-get install -y --no-install-recommends wget bzip2 git gcc g++ gnupg2 \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Now that we have wget, install clang-format
# clang-format-10 depends on the python apt package, but Python is installed
# from source, not apt, so apt doesn't see it. Hack around it to avoid the dep.
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key| apt-key add -
RUN echo "deb http://apt.llvm.org/buster/ llvm-toolchain-buster-10 main" >> /etc/apt/sources.list
RUN apt-get update \
    && apt-get install -y --no-install-recommends libllvm10 libclang-cpp10 \
    && apt-get autoremove -y \
    && apt-get download clang-format-10 \
    && dpkg --force-all -i clang-format-10*.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 -O gcc-arm.tar.bz2 \
    && tar -xjf /gcc-arm.tar.bz2 \
    && rm gcc-arm.tar.bz2
ENV PATH="/gcc-arm-none-eabi-9-2019-q4-major/bin:$PATH"

RUN pip install modm scons
RUN wget -O /run-clang-format.py https://raw.githubusercontent.com/Sarcasm/run-clang-format/50a5250a2666218595ee21d68c71f50ac39866d2/run-clang-format.py
