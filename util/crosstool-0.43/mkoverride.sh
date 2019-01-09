#!/bin/sh
#
# Copyright 2004 Google Inc.
# All Rights Reserved.  This script is provided under the terms of the GPL.
#
# Author: matthewbg@google.com (Matthew Beaumont-Gay)
#
# Creates the support library override directories and associated scripts

abort() {
  echo $@
  exec false
}

test -z "${TARGET}" && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${PREFIX}" && abort "Please set PREFIX to where you want the toolchain installed."

set -ex

# These should match the lib* subpackages in the crosstool-gcc specfile
# FIXME: Include more shared libraries, like java and fortran

for LIB in libgcc_s libstdc++ libssp libmudflap
do
    for DIR in $PREFIX/$TARGET/lib $PREFIX/$TARGET/lib64
    do
        # FIXME: what's right way to test whether a wildcard matches?
        if test -f $DIR/$LIB.so || test -f $DIR/$LIB.so.?; then
            rm -rf $DIR/$LIB.dir
            mkdir $DIR/$LIB.dir
            cd $DIR/$LIB.dir
            for a in ../$LIB.so*
            do
                ln -s $a 
            done
            cd ../../../..
        fi
    done
done
