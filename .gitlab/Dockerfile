# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of Taproot.
#
# Taproot is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Taproot is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Taproot.  If not, see <https://www.gnu.org/licenses/>.

# Temporary stages ===========
FROM debian:buster as gtest-build

ARG GTEST_VERSION=1.10.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

ADD https://github.com/google/googletest/archive/release-${GTEST_VERSION}.tar.gz /
RUN tar -xzf /release-*.tar.gz

RUN cd /googletest-release-${GTEST_VERSION}/ \
    && mkdir build
WORKDIR /googletest-release-${GTEST_VERSION}/build/

RUN cmake ../
RUN make -j

# Main image =================
FROM python:3.8.0-slim-buster
# We do all installations in a single layer and clean up residual files in one
# pass to minimize Docker image size (and thus, CI build time).

# Install misc basic dependencies
# Installing gnupg2 for "apt-key add"
RUN apt-get update \
    && apt-get install -y --no-install-recommends wget bzip2 git gcc g++ gnupg2 valgrind \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# lcov
RUN apt-get update \
    && apt-get install -y lcov gcovr \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

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

RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 -O gcc-arm.tar.bz2 \
    && tar -xjf /gcc-arm.tar.bz2 \
    && rm gcc-arm.tar.bz2
ENV PATH="/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

ARG GTEST_VERSION=1.10.0
# "install" gtest and gmock
COPY --from=gtest-build /googletest-release-${GTEST_VERSION}/googletest/include/ /usr/local/include/
COPY --from=gtest-build /googletest-release-${GTEST_VERSION}/googlemock/include/ /usr/local/include/
COPY --from=gtest-build /googletest-release-${GTEST_VERSION}/build/lib/libg*.a /usr/local/lib/

RUN pip install modm scons
RUN wget -O /run-clang-format.py https://raw.githubusercontent.com/Sarcasm/run-clang-format/50a5250a2666218595ee21d68c71f50ac39866d2/run-clang-format.py
