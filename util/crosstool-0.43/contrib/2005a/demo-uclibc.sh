#!/bin/sh
# Strange little script to demonstrate building a uclibc-based toolchain
# using an old version of crosstool

set -ex
TARBALLS_DIR=$HOME/downloads
mkdir -p $TARBALLS_DIR
wget -c -P $TARBALLS_DIR kegel.com/crosstool/crosstool-0.28-rc5.tar.gz
rm -rf crosstool-0.28-rc5
tar -xzf $TARBALLS_DIR/crosstool-0.28-rc5.tar.gz
cd crosstool-0.28-rc5
patch -p1 < ../crosstool-uclibc-0.28-rc5-ter.patch

sed 's/linux-gnu/linux-uclibc/' < mipsel.dat > mipsel-uclibc.dat

RESULT_TOP=/opt/crosstool
mkdir -p $RESULT_TOP
export TARBALLS_DIR RESULT_TOP
GCC_LANGUAGES="c,c++"
export GCC_LANGUAGES
eval `cat gcc-3.3.3-uclibc-0.9.23.dat mipsel-uclibc.dat` sh all.sh

# The above command fails the "hello, world" test when linking non-static c++ programs,
# with error
#  mipsel-unknown-linux-uclibc-g++ hello2.cc -o mipsel-unknown-linux-uclibc-hello2
#  mipsel-unknown-linux-uclibc/lib/libstdc++.so: undefined reference to `sqrtf
# but can link static ones ok. 
