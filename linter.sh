#!/bin/bash
pip install cpplint
listTests=(
  "cpplint --recursive --linelength=100 --filter='-legal/copyright','-whitespace/parens','-build/header_guard','-whitespace/braces','-build/namespaces' ./mcb-2019-2020-project/src"
  "cpplint --linelength=100 --filter='-legal/copyright','-whitespace/parens','-build/header_guard','-whitespace/braces','-build/namespaces' ./mcb-2019-2020-project/rm-dev-board-a/* ./mcb-2019-2020-project/*"
)
for test in "${listTests[@]}"
do
  eval $test
  if [ $? -ne 0 ]; then
    exit 1
  fi
done
echo LINT PASSED
