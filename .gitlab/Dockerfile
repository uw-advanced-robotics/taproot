# How to build docker image:
# Windows CMD: docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
# Windows PS: Get-Content Dockerfile | docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
# Linux: docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
#
# How to publish docker image:
# docker push aruw/mcb-2020-gitlab-ci:2020-02-17.1

FROM python:3.8.0-slim-buster
RUN apt-get update \
    && apt-get install -y --no-install-recommends wget bzip2 scons git cppcheck \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 -O gcc-arm.tar.bz2 \
    && tar -xjf /gcc-arm.tar.bz2 \
    && rm gcc-arm.tar.bz2
ENV PATH="/gcc-arm-none-eabi-9-2019-q4-major/bin:$PATH"

RUN pip install modm cpplint
