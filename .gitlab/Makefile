IMAGE_VERSION := "$(shell ./get-new-version.sh)"

build:
	time docker build . --tag aruw/mcb-2020-gitlab-ci

publish:
	docker tag aruw/mcb-2020-gitlab-ci aruw/mcb-2020-gitlab-ci:$(IMAGE_VERSION)
	docker push aruw/mcb-2020-gitlab-ci:$(IMAGE_VERSION)
