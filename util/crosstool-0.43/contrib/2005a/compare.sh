#!/bin/sh
# Example of how to compare toolchains built with and without a single patch
# This was used to prove that gcc-3.3.2-arm-softfloat.patch did not cause
# ppc750 toolchains to generate different binaries 
set -ex
TARBALLS_DIR=$HOME/downloads
export TARBALLS_DIR 
GCC_LANGUAGES="c"
export GCC_LANGUAGES

RESULT_TOP=/opt/crosstool/xxvfp
export RESULT_TOP
mkdir -p $RESULT_TOP
cp contrib/gcc-3.3.2-arm-softfloat.patch patches/gcc-3.3.3
eval `cat powerpc-750.dat gcc-3.3.3-glibc-2.3.2.dat`  sh all.sh --notest

RESULT_TOP=/opt/crosstool/novfp
export RESULT_TOP
mkdir -p $RESULT_TOP
rm -f patches/gcc-3.3.3/gcc-3.3.2-arm-softfloat.patch
eval `cat powerpc-750.dat gcc-3.3.3-glibc-2.3.2.dat`  sh all.sh --notest

/opt/crosstool/xxvfp/powerpc-750-linux-gnu/gcc-3.3.3-glibc-2.3.2/bin/powerpc-750-linux-gnu-gcc -static struct-ret-1.c -o xxvfp.out
/opt/crosstool/novfp/powerpc-750-linux-gnu/gcc-3.3.3-glibc-2.3.2/bin/powerpc-750-linux-gnu-gcc -static struct-ret-1.c -o novfp.out
echo comparing binaries.  Expect only to see \'no\' replaced by \'xx\', no other changes.
cmp xxvfp.out novfp.out || echo try \'cmp -l xxvfp.out novfp.out\' to see where they differ
echo Finished

