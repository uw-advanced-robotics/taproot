#!/bin/bash
pip install cpplint
listTests=(
  "cpplint --recursive ./mcb-2019-2020-project/src"
  "cpplint ./mcb-2019-2020-project/rm-dev-board-a/* ./mcb-2019-2020-project/*"
)
for test in "${listTests[@]}"
do
  eval $test
  if [ $? -ne 0 ]; then
    exit 1
  fi
done
echo LINT PASSED
