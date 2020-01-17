#!/bin/bash
listTests=(
  "echo cpplint checks:  ================================================================"
  "cpplint --recursive ./mcb-2019-2020-project/src"
  "cpplint ./mcb-2019-2020-project/rm-dev-board-a/* ./mcb-2019-2020-project/*"
  "echo"
  "echo cppcheck checks:  ================================================================"
  "cppcheck ./mcb-2019-2020-project/src ./mcb-2019-2020-project/robot-type ./mcb-2019-2020-project/rm-dev-board-a --suppress=unusedFunction:./mcb-2019-2020-project/* --suppress=invalidPointerCast:./mcb-2019-2020-project/src/aruwlib/algorithms/mahony_ahrs.cpp --suppress=useStlAlgorithm:./mcb-2019-2020-project/src/aruwlib/errors/error_controller.cpp ./mcb-2019-2020-project/src/errors/ --rule-file=./.gitlab/rules.xml --inline-suppr --enable=all --error-exitcode=1 --template='[{file}:{line}] ({severity}) ({id}) {message}'"
)
for test in "${listTests[@]}"
do
  eval $test
  if [ $? -ne 0 ]; then
    echo LINT FAILED  ================================================================
    exit 1
  fi
done
echo LINT PASSED  ================================================================