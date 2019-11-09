# How to build docker image:
# Windows CMD: docker build -t aruw/mcb-2020-gitlab-ci:<tag> < Dockerfile
# Windows PS: Get-Content Dockerfile | docker build -t aruw/mcb-2020-gitlab-ci:<tag> -
# Linux: docker build -t aruw/mcb-2020-gitlab-ci:<tag> - < Dockerfile
#
# How to publish docker image:
# docker push aruw/mcb-2020-gitlab-ci:<tag>

FROM python:3.8.0

RUN apt-get update

RUN apt-get install -y scons git

RUN pip install modm cpplint

RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 -O gcc-arm.tar.bz2
RUN tar -xjf gcc-arm.tar.bz2
ENV PATH="/gcc-arm-none-eabi-9-2019-q4-major/bin:$PATH"