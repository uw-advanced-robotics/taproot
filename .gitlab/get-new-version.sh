#! /bin/sh

TAGS_API_URL="https://registry.hub.docker.com/v1/repositories/aruw/mcb-2020-gitlab-ci/tags"

ver_base=$(TZ='America/Los_Angeles' date +%F)

# requires jq: sudo apt install jq
curl --silent "$TAGS_API_URL" \
    | jq -r "
        [
            .[]
            | .name
            | select(startswith(\"$ver_base\"))
            | ltrimstr(\"$ver_base.\")
            | tonumber
        ] | max // 0
        | .+1
        | tostring
        | \"$ver_base.\" + .
    "
