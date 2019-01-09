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

# Build the toolchain.
# eval `cat i686-cygwin.dat gcc-3.3.2-cygwin-1.5.10-3.dat`  sh all.sh --notest
eval `cat i686-cygwin.dat gcc-3.3.2-cygwin-1.5.15-1.dat`  sh all.sh --notest

echo Done.
