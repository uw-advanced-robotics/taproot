# The .gitlab folder

## What is this folder?
This folder contains everything used solely by GitLab, and doesn't need to be placed alongside the project itself.

## What is in here?
Currently in this folder are the following files:
- `Dockerfile`: The Dockerfile for our CI docker image. Pushed as aruw/mcb-2020-gitlab-ci on Docker Hub.
- `linter.sh`: This is the shell script run by GitLab CI to use various C++ linters on our code.
- `rules.xml`: Custom rules for the cppcheck linter.

## Locally Linting
Note: You will have to push code to GitLab before you can build it using Docker.

The easy way (Docker Image for Windows/Mac):
1) Install Docker Desktop for [Windows](https://download.docker.com/win/stable/Docker%20Desktop%20Installer.exe) or [Mac](https://download.docker.com/mac/stable/Docker.dmg) or [Linux](https://docs.docker.com/install/)
2) Download/Run the image using the command `docker run -i --tty aruw/mcb-2020-gitlab-ci bash` (Note: when you exit the container, the container will be deleted.)
3) Clone the gitlab repo: `git clone https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020.git`
4) `cd mcb-2019-2020`
5) Checkout the branch/commit to lint: `git checkout <branch name or commit sha>`
6) `./.gitlab/linter.sh`

To update linter, run `docker pull aruw/mcb-2020-gitlab-ci`

The other way (Linux only):
1) Install cpplint and cppcheck: `apt-get install cpplint cppcheck`
2) Clone the gitlab repo: `git clone https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020.git`
3) `cd mcb-2019-2020`
4) Checkout the branch/commit to lint: `git checkout <branch name or commit sha>`
5) `./.gitlab/linter.sh`

To update linter, run `apt-get update` then `apt-get upgrade`

## Locally Building w/ Docker Image
Note: You will have to push code to GitLab before you can build it using Docker.

1) Install Docker Desktop for [Windows](https://download.docker.com/win/stable/Docker%20Desktop%20Installer.exe) or [Mac](https://download.docker.com/mac/stable/Docker.dmg) or [Linux](https://docs.docker.com/install/)
2) Download/Run the image using the command `docker run -i --tty aruw/mcb-2020-gitlab-ci bash` (Note: when you exit the container, the container will be deleted.)
3) Clone the gitlab repo: `git clone https://gitlab.com/aruw/code-2019-2020/mcb-2019-2020.git`
4) `cd mcb-2019-2020`
5) Checkout the branch/commit to lint: `git checkout <branch name or commit sha>`
6) `git submodule update --init --recursive`
7) `cd mcb-2019-2020-project`
8) `rm -r modm` (Optional, but recommended)
9) `lbuild build`
10) `/usr/bin/env python3 $(which scons) build`

Optional extras to improve experience in the container:
- Install a text editor like nano: `apt-get install nano` Usage: `nano file.txt` (Ctrl + X to exit)

## Configuring Linters
Right now we have two C++ linters in use. Here's how to configure them:
1) **cpplint**: There is a configuration file, `CPPLINT.cfg`, in the root of the repo. Info on what can go in there can be found here: https://github.com/cpplint/cpplint/blob/master/cpplint.py#L222-L263
2) **cppcheck**: You can define custom rules in the `rules.xml` file. See the file for more information on writing rules. Also currently all built-in checks are run. You can configure which are run by modifying command line arguments in `linter.sh`. (There's also other things that can be configured using command line arguments, check the output of `cppcheck` for more info)