#!/bin/sh
# Just a trivial example of how to run a particular gcc regression test
# Edit as needed
set -ex
TARGET=sh4-unknown-linux-gnu
TOOLCOMBO=gcc-3.3.1-glibc-2.3.2
PREFIX=/opt/crosstool/$TOOLCOMBO/$TARGET
TOP=`pwd`
cd build/$TARGET/$TOOLCOMBO/build-gcc/gcc/testsuite
DEJAGNU=$TOP/boards/master.exp PATH=$PREFIX/bin:$TOP/result/dejagnu/bin:$PATH runtest --tool=g++ --target=$TARGET -v -v -a compat.exp
