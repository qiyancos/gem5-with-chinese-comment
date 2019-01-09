#!/bin/sh

abort() {
    echo $@
    exec /bin/false
}

#
# crosstool.sh
# Build a GNU/Linux toolchain
#
# Copyright (c) 2001 by Bill Gatliff, bgat@billgatliff.com 
# Copyright (c) 2003 by Dan Kegel, dkegel@ixiacom.com, Ixia Communications, 
# All rights reserved.  This script is provided under the terms of the GPL.
# For questions, comments or improvements see the crossgcc mailing
# list at http://sources.redhat.com/ml/crossgcc, or contact the
# authors, but do your homework first.  As Bill says, "THINK!"
# 17-Sep-03: Modified for Newlib by Nicolas Moreau ngbmoreau@yahoo.com.au

#
# Meant to be invoked from another shell script.
# Usage: seven environment variables must be set, namely:
test -z "${PREFIX}"           && abort "Please set PREFIX to where you want the toolchain installed."
test -z "${BINUTILS_DIR}"     && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${GCC_DIR}"          && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${NEWLIB_DIR}"       && abort "Please set NEWLIB_DIR to the bare filename of the newlib tarball or directory"
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${TARGET_CFLAGS}"    && abort "Please set TARGET_CFLAGS to any compiler flags needed when building newlib (-O recommended)"

# Four environment variables are optional, namely:
test -z "${GCC_EXTRA_CONFIG}" && echo  "GCC_EXTRA_CONFIG not set, so not passing any extra options to gcc's configure script"

set -ex

# map TARGET to Linux equivalent
case $TARGET in
    alpha*)   ARCH=alpha ;;
    arm*)     ARCH=arm ;;
    cris*)    ARCH=cris ;;
    hppa*)    ARCH=parisc ;;
    i*86*)    ARCH=i386 ;;
    ia64*)    ARCH=ia64 ;;
    mips*)    ARCH=mips ;;
    m68k*)    ARCH=m68k ;;
    powerpc*) ARCH=ppc ;;
    ppc*)     abort "Target $TARGET incompatible with binutils and gcc regression tests; use target powerpc-* instead";;
    s390*)    ARCH=s390 ;;
    sh*)      ARCH=sh ;;
    sparc64*) ARCH=sparc64 ;;
    sparc*)   ARCH=sparc ;;
    x86_64*)  ARCH=x86_64 ;;
    *) abort "Bad target $TARGET"
esac

# Make all paths absolute (it's so confusing otherwise)
PREFIX=`cd $PREFIX; pwd`
BINUTILS_DIR=`cd $BINUTILS_DIR; pwd`
GCC_DIR=`cd $GCC_DIR; pwd`
NEWLIB_DIR=`cd $NEWLIB_DIR; pwd`

# make sure the build product's binaries are in the search path
PATH="${PREFIX}/bin:${PATH}"
export PATH

# test that we have write permissions to the install dir
mkdir -p ${PREFIX}/${TARGET}
touch ${PREFIX}/${TARGET}/test-if-write
test -f ${PREFIX}/${TARGET}/test-if-write || abort "You don't appear to have write permissions to ${PREFIX}/${TARGET}."

echo
echo Building for:
echo "    --target=$TARGET"
echo "    --prefix=$PREFIX"

# Set HOST to something almost, but not completely, identical to BUILD
# This strange operation causes gcc to always generate a cross-compiler
# even if the build machine is the same kind as the host.
BUILD=`$GCC_DIR/config.guess`
HOST=`echo $BUILD | sed s/-/-host_/`

#---------------------------------------------------------
echo Build binutils

mkdir -p build-binutils; cd build-binutils

if test '!' -f Makefile; then
    ${BINUTILS_DIR}/configure --target=$TARGET --prefix=$PREFIX
fi

make all 
make install 

cd ..

# test to see if this step passed
test -x ${PREFIX}/bin/${TARGET}-ld || abort Build failed during binutils 


#---------------------------------------------------------
echo "Build gcc-core (just enough to build newlib)"

mkdir -p build-gcc-core; cd build-gcc-core

if test '!' -f Makefile; then
    ${GCC_DIR}/configure --target=$TARGET --host=$HOST --prefix=$PREFIX \
	--with-local-prefix=${PREFIX}/${TARGET} \
	--disable-multilib \
	--with-newlib \
        ${GCC_EXTRA_CONFIG} \
        --without-headers \
	--disable-nls \
	--enable-threads=no \
	--enable-symvers=gnu \
        --enable-languages=c \
	--disable-shared
fi

make all-gcc install-gcc 

cd ..

test -x ${PREFIX}/bin/${TARGET}-gcc || abort Build failed during gcc-core 

#---------------------------------------------------------
echo Build Newlib

mkdir -p build-newlib; cd build-newlib

if test '!' -f Makefile; then
    
    ${NEWLIB_DIR}/configure --target=$TARGET --prefix=$PREFIX
fi

make
make install 

cd ..

test -f ${PREFIX}/${TARGET}/lib/libc.a || abort Building libc failed

#---------------------------------------------------------
echo Build final gcc

mkdir -p build-gcc; cd build-gcc

if test '!' -f Makefile; then
    # --enable-symvers=gnu really only needed for sh4 to work around a detection problem
    # only matters for gcc-3.2.x and later, I think
    # --disable-nls to work around crash bug on ppc405, but also because embedded
    # systems don't really need message catalogs...
    # Use --with-headers, else it will define disable_glibc while building libgcc, and you'll have no profiling
    ${GCC_DIR}/configure --target=$TARGET --host=$HOST --prefix=$PREFIX \
        ${GCC_EXTRA_CONFIG} \
        --with-headers=${PREFIX}/${TARGET}/include \
	--disable-nls \
	--enable-symvers=gnu \
        --enable-languages=c,c++ \
	--disable-shared \
	--enable-c99 \
        --enable-long-long
fi

make all 
make install 

esac

cd ..
cd ..

test -x ${PREFIX}/bin/${TARGET}-gcc || Build failed during final gcc 

#---------------------------------------------------------
echo Cross-toolchain build complete.
exit 0

