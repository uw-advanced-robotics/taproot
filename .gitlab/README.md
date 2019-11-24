# The .gitlab folder

## What is this folder?
This folder contains everything used solely by GitLab, and doesn't need to be placed alongside the project itself.

## What is in here?
Currently in this folder are the following files:
- `Dockerfile`: The Dockerfile for our CI docker image. Pushed as aruw/mcb-2020-gitlab-ci on Docker Hub.
- `linter.sh`: This is the shell script run by GitLab CI to use various C++ linters on our code.
- `rules.xml`: Custom rules for the cppcheck linter.

## Configuring Linters
Right now we have two C++ linters in use. Here's how to configure them:
1) **cpplint**: There is a configuration file, `CPPLINT.cfg`, in the root of the repo. Info on what can go in there can be found here: https://github.com/cpplint/cpplint/blob/master/cpplint.py#L222-L263
2) **cppcheck**: You can define custom rules in the `rules.xml` file. See the file for more information on writing rules. Also currently all built-in checks are run. You can configure which are run by modifying command line arguments in `linter.sh`. (There's also other things that can be configured using command line arguments, check the output of `cppcheck` for more info)