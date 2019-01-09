#!/bin/sh
set -ex
TARBALLS_DIR=$HOME/downloads
RESULT_TOP=/opt/crosstool
export TARBALLS_DIR RESULT_TOP
GCC_LANGUAGES="c,c++"
export GCC_LANGUAGES

# Really, you should do the mkdir before running this,
# and chown /opt/crosstool to yourself so you don't need to run as root.
mkdir -p $RESULT_TOP

# Build the toolchain.  Takes a couple hours and a couple gigabytes.

# Any and all of the following line should fail with symptoms described in
#  http://gcc.gnu.org/PR17209
# I'll leave them all uncommented, but the first one to hit the bug will abort the script.

 eval `cat armv5b-softfloat.dat gcc-3.4.2-glibc-2.2.5.dat`    sh all.sh --notest
 eval `cat armv5b-softfloat.dat gcc-3.4.2-glibc-20040827.dat` sh all.sh --notest
 eval `cat arm-iwmmxt.dat       gcc-3.4.2-glibc-2.3.3.dat`    sh all.sh --notest
 eval `cat arm-iwmmxt.dat       gcc-3.4.2-glibc-20040827.dat` sh all.sh --notest
 eval `cat arm-xscale.dat       gcc-3.4.2-glibc-20040827.dat` sh all.sh --notest

echo Done.
