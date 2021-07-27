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
FROM ubuntu:20.04 as gtest-build

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
FROM ubuntu:20.04
# We do all installations in a single layer and clean up residual files in one
# pass to minimize Docker image size (and thus, CI build time).

ENV DEBIAN_FRONTEND=noninteractive

# Install misc basic dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends wget git valgrind clang-format-10 python3 python3-pip scons gcc-10 g++-10 locales locales-all lcov gcovr \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 90 --slave /usr/bin/g++ g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10

# "install" the gcc ARM cross-compiler
RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -O gcc-arm.tar.bz2 \
    && tar -xjf /gcc-arm.tar.bz2 \
    && rm gcc-arm.tar.bz2
ENV PATH="/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"

ARG GTEST_VERSION=1.10.0
# "install" gtest and gmock
COPY --from=gtest-build /googletest-release-${GTEST_VERSION}/googletest/include/ /usr/local/include/
COPY --from=gtest-build /googletest-release-${GTEST_VERSION}/googlemock/include/ /usr/local/include/
COPY --from=gtest-build /googletest-release-${GTEST_VERSION}/build/lib/libg*.a /usr/local/lib/

RUN pip3 install lbuild modm
RUN wget -O /run-clang-format.py https://raw.githubusercontent.com/Sarcasm/run-clang-format/50a5250a2666218595ee21d68c71f50ac39866d2/run-clang-format.py
