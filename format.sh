set -e

cd slam
clang-format -i *.cpp *.h
cd ../test
clang-format -i *.cpp
cd ../apps
clang-format -i *.cpp
cd ..
