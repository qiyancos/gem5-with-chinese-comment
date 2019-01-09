#!/bin/sh
# Script to test a gnu/linux toolchain by building the linux kernel
# Copyright (c) 2004 by Dan Kegel
# All rights reserved.  This script is provided under the terms of the GPL.
# For questions, comments or improvements see the crossgcc mailing
# list at http://sources.redhat.com/ml/crossgcc, but please do your homework first.

set -ex

#env | sort

abort() {
    echo $@
    exec false
}

test -z "${PREFIX}"           && abort "Please set PREFIX to where you want the toolchain installed."
test -z "${LINUX_DIR}"        && abort "Please set LINUX_DIR to the bare filename of the kernel tarball or directory"
test -z "${BUILD_DIR}"        && abort "Please set BUILD_DIR to the directory where the tools are to be built"
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${KERNELCONFIG}" || test -r "${KERNELCONFIG}"  || abort  "Can't read file KERNELCONFIG = $KERNELCONFIG, please fix."

# check for canadian cross
if test x"$GCC_HOST" != x; then
    echo "GCC_HOST set, so we can't run the compilers on the build machine, not testing"
    exit 0
fi

# map TARGET to Linux equivalent
case $TARGET in
    alpha*)   ARCH=alpha ;;
    arm*)     ARCH=arm ;;
    cris*)    ARCH=cris ;;
    hppa*)    ARCH=parisc ;;
    i*86*)    ARCH=i386 ;;
    i4004)    abort "ENOMEM" ;;
    ia64*)    ARCH=ia64 ;;
    mips*)    ARCH=mips ;;
    m68k*)    ARCH=m68k ;;
    powerpc64*) ARCH=ppc64 ;;
    powerpc*) ARCH=ppc ;;
    ppc*)     abort "Target $TARGET incompatible with binutils and gcc regression tests; use target powerpc-* or powerpc64-* instead";;
    s390*)    ARCH=s390 ;;
    sh*)      ARCH=sh ;;
    sparc64*) ARCH=sparc64 ;;
    sparc*)   ARCH=sparc ;;
    vax*)     ARCH=vax ;;
    x86_64*)  ARCH=x86_64 ;;
    *) abort "Bad target $TARGET"
esac

cd $BUILD_DIR

# autodetect kernel version from contents of Makefile
KERNEL_VERSION=`awk '/^VERSION =/ { print $3 }' $LINUX_DIR/Makefile`
KERNEL_PATCHLEVEL=`awk '/^PATCHLEVEL =/ { print $3 }' $LINUX_DIR/Makefile`

# Test the C compiler by building the Linux kernel
cd $LINUX_DIR

if test -f "$KERNELCONFIG" ; then
    cp $KERNELCONFIG .config
fi

ARCH=$ARCH CROSS_COMPILE=$PREFIX/bin/$TARGET-

case "$KERNEL_VERSION.$KERNEL_PATCHLEVEL.x" in
2.2.x|2.4.x) 
	test -f .config || abort "sorry, for 2.4 kernel you need a config file"
	yes "" | make ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- oldconfig
	make ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- clean
	make ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- dep
	yes "" | make V=1 $PARALLELMFLAGS ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- 
	;;
2.6.x)
	if test -f .config; then
		yes "" | make ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- oldconfig
	else
		yes "" | make ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- allnoconfig
	fi
	make ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- clean
	make V=1 $PARALLELMFLAGS ARCH=$ARCH  CROSS_COMPILE=$PREFIX/bin/$TARGET- 
	;;
esac

echo "Linux kernel build done."
