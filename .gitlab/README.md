# MCB CI Docker image

## Build and publish instructions

With Makefile:
```
make build
make publish
```

Manually (make sure to change version):
```
docker build -t aruw/mcb-2020-gitlab-ci:2020-02-17.1 .
docker push aruw/mcb-2020-gitlab-ci:2020-02-17.1
```