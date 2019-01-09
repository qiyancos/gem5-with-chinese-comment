#!/bin/sh
abort() {
    echo $@
    exec /bin/false
}
# Script to download sources for, build, and test a gnu/linux toolchain
# Copyright (c) 2003 by Dan Kegel, Ixia Communications.
# All rights reserved.  This script is provided under the terms of the GPL.
# For questions, comments or improvements see the crossgcc mailing
# list at http://sources.redhat.com/ml/crossgcc, but do your homework first.
# As Bill Gatliff says, "THINK!"
# 17-Sep-03: Modified for Newlib by Nicolas Moreau ngbmoreau@yahoo.com.au

#
# Meant to be invoked from another shell script.
# Usage: six environment variables must be set, namely:
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${TARGET_CFLAGS}"    && abort "Please set TARGET_CFLAGS to any compiler flags needed when building glibc (-O recommended)"
test -z "${BINUTILS_DIR}"     && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${GCC_DIR}"          && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${NEWLIB_DIR}"       && abort "Please set NEWLIB_DIR to the bare filename of the newlib tarball or directory"

# Three environment variables are optional, namely:
test -z "${GCC_EXTRA_CONFIG}" && echo  "GCC_EXTRA_CONFIG not set, so not passing any extra options to gcc's configure script"

set -ex

TOOLCOMBO=$GCC_DIR
BUILD_DIR=`pwd`/build/$TARGET/$TOOLCOMBO
TOP_DIR=`pwd`

# Arbitrary locations for the input and output of the build.
# Change or override these to your taste.
TARBALLS_DIR=${TARBALLS_DIR-$TOP_DIR/tarballs}
RESULT_TOP=${RESULT_TOP-$TOP_DIR/result}
PREFIX=${PREFIX-$RESULT_TOP/$TARGET/$TOOLCOMBO}

export TOOLCOMBO
export PREFIX
export BUILD_DIR
export TARBALLS_DIR
export TOP_DIR

# Download/unpack/patch tarballs, if desired
while [ $# -gt 0 ]; do
    case "$1" in
	--nounpack|-nounpack) 
	   opt_no_unpack=1
	   ;;
	--nobuild|-nobuild) 
	   opt_no_build=1
	   ;;
	--builduserland|-builduserland) 
	   opt_builduserland=1
	   ;;
	--notest|-notest) 
	   opt_no_test=1
	   ;;
	*)
	    abort "Usage: build.sh [--nounpack|--nobuild|--builduserland|--notest]"
    esac
    shift
done

if test "$opt_no_unpack" = ""; then
   # Download and patch
   rm -rf $BUILD_DIR; mkdir -p $BUILD_DIR
   sh getandpatch-newlib.sh
fi

if test "$opt_no_build" = ""; then
    # Build
    rm  -rf  $PREFIX
    mkdir -p $PREFIX
    mkdir -p $BUILD_DIR
    cd $BUILD_DIR
    sh $TOP_DIR/crosstool-newlib.sh
    cd $TOP_DIR

    # Cute little compile test
    sh testhello.sh
fi
