#!/bin/bash -e

rm -rf {build,devel,install}
. ./tools/lint.sh
#catkin_make --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Werror" --make-args -j2 -l2
catkin_make --cmake-args -DCMAKE_CXX_FLAGS="-Wall" --make-args -j2 -l2
catkin_make run_tests
catkin_test_results --verbose ./build/test_results
