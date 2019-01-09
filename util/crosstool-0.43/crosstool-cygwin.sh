#!/bin/sh

abort() {
    echo $@
    exec false
}

#
# crosstool.sh
# Build a GNU/Linux toolchain
#
# Copyright (c) 2001 by Bill Gatliff, bgat@billgatliff.com 
# Copyright (c) 2003 by Dan Kegel, dkegel@ixiacom.com, Ixia Communications, 
# Copyright (c) 2004 by Dan Kegel, Google, Inc.
# All rights reserved.  This script is provided under the terms of the GPL.
# For questions, comments or improvements see the crossgcc mailing
# list at http://sources.redhat.com/ml/crossgcc, or contact the
# authors, but do your homework first.  As Bill says, "THINK!"
#
# 17 September 2004: Steve Papacharalambous (stevep@metrowerks.com)
#                    Modifications for building cygwin targets.
#
# Meant to be invoked from another shell script.
# Usage: nine environment variables must be set, namely:
# Stevep: Note only eight environment variables for cygwin target:
test -z "${PREFIX}"           && abort "Please set PREFIX to where you want the toolchain installed."
test -z "${BUILD_DIR}"        && abort "Please set BUILD_DIR to the directory where the tools are to be built"
test -z "${SRC_DIR}"          && abort "Please set SRC_DIR to the directory where the source tarballs are to be unpacked"
test -z "${BINUTILS_DIR}"     && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${GCC_DIR}"          && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${CYGWIN_DIR}"        && abort "Please set CYGWIN_DIR to the bare filename of the cygwin tarball or directory"
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${TARGET_CFLAGS}"    && abort "Please set TARGET_CFLAGS to any compiler flags needed when building glibc (-O recommended)"

# Fourteen are optional
# Stevep: Nine for cygwin targets:
test -z "${BINUTILS_EXTRA_CONFIG}" && echo "BINUTILS_EXTRA_CONFIG not set, so not passing any extra options to binutils' configure script"
test -z "${GCC_EXTRA_CONFIG}"      && echo "GCC_EXTRA_CONFIG not set, so not passing any extra options to gcc's configure script"
test -z "${USE_SYSROOT}"           && echo "USE_SYSROOT not set, so not configuring with --with-sysroot"
test -z "${GCC_BUILD}"             && echo "GCC_BUILD not set, assuming BUILD=output of config.guess"
test -z "${GCC_HOST}"              && echo "GCC_HOST not set, assuming HOST=BUILD"
test -z "${SHARED_MODE}" && SHARED_MODE="--enable-shared" && echo "SHARED_MODE not set, so defaulting to --enable-shared"
test -z "${GCC_LANGUAGES}"         && echo "GCC_LANGUAGES not set, assuming c,c++"
GCC_LANGUAGES=${GCC_LANGUAGES-"c,c++"}
TOP_DIR=${TOP_DIR-`pwd`}
BUILD=${GCC_BUILD-`$TOP_DIR/config.guess`}


# Check for a few prerequisites that have tripped people up.
awk '/x/' < /dev/null  || abort "You need awk to build a toolchain."
test -z "${CFLAGS}"    || abort "Don't set CFLAGS, it screws up the build"
test -z "${CXXFLAGS}"  || abort "Don't set CXXFLAGS, it screws up the build"

if test "$GCC_HOST" != ""; then
	# Modify $BUILD so gcc never, ever thinks $build = $host
	UNIQUE_BUILD=`echo $BUILD | sed s/-/-build_/`
	CANADIAN_BUILD="--build=$UNIQUE_BUILD"
	echo "canadian cross, configuring gcc & binutils with $CANADIAN_BUILD"
	# make sure we have a host compiler (since $GCC_HOST-gcc won't work)
	"$CC" --version || abort "Must set CC to a compiler targeting $GCC_HOST.  PATH is $PATH"
	"$AR" --version || abort "Must set AR to a version of 'ar' targeting $GCC_HOST.  PATH is $PATH"
        # make sure we have a target compiler (otherwise glibc configure will fail)
	"$TARGET-gcc" --version || abort "Could not execute $TARGET-gcc.  PATH is $PATH"
else
	GCC_HOST=$BUILD
	CANADIAN_BUILD=""
fi

# Modify GCC_HOST to never be equal to $BUILD or $TARGET
# This strange operation causes gcc to always generate a cross-compiler
# even if the build machine is the same kind as the host.
# This is why CC has to be set when doing a canadian cross;
# you can't find a host compiler by appending -gcc to our whacky $GCC_HOST
GCC_HOST=`echo $GCC_HOST | sed s/-/-host_/`

set -ex

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

# Make all paths absolute (it's so confusing otherwise)
# FIXME: this doesn't work well with some automounters
PREFIX=`cd $PREFIX; pwd`
BUILD_DIR=`cd $BUILD_DIR; pwd`
SRC_DIR=`cd $SRC_DIR; pwd`
BINUTILS_DIR=`cd ${SRC_DIR}/${BINUTILS_DIR}; pwd`
GCC_DIR=`cd ${SRC_DIR}/${GCC_DIR}; pwd`
CYGWIN_DIR=`cd ${SRC_DIR}/${CYGWIN_DIR}; pwd`

# If user isn't doing a canadian cross, add the target compiler's bin to
# the path, so we can use the compiler we build to build glibc etc.
if test "$CANADIAN_BUILD" = ""; then
	PATH="${PREFIX}/bin:${PATH}"
	export PATH
fi

# test that we have write permissions to the install dir
mkdir -p ${PREFIX}/${TARGET}
touch ${PREFIX}/${TARGET}/test-if-write
test -w ${PREFIX}/${TARGET}/test-if-write || abort "You don't appear to have write permissions to ${PREFIX}/${TARGET}."
rm -f ${PREFIX}/${TARGET}/test-if-write

if test -z "$USE_SYSROOT"; then
    # plain old way.  all libraries in prefix/target/lib
    SYSROOT=${PREFIX}/${TARGET}
    HEADERDIR=$SYSROOT/include
    # hack!  Always use --with-sysroot for binutils.
    # binutils 2.14 and later obey it, older binutils ignore it.
    # Lets you build a working 32->64 bit cross gcc
    BINUTILS_SYSROOT_ARG="--with-sysroot=${SYSROOT}"
    # Use --with-headers, else final gcc will define disable_glibc while building libgcc, and you'll have no profiling
    GCC_SYSROOT_ARG_CORE="--without-headers"
    GCC_SYSROOT_ARG="--with-headers=${HEADERDIR}"
else
    # spiffy new sysroot way.  libraries split between
    # prefix/target/sys-root/lib and prefix/target/sys-root/usr/lib
    SYSROOT=${PREFIX}/${TARGET}/sys-root
    HEADERDIR=$SYSROOT/usr/include
    BINUTILS_SYSROOT_ARG="--with-sysroot=${SYSROOT}"
    GCC_SYSROOT_ARG="--with-sysroot=${SYSROOT}"
    GCC_SYSROOT_ARG_CORE=$GCC_SYSROOT_ARG
    # glibc's prefix must be exactly /usr, else --with-sysroot'd
    # gcc will get confused when $sysroot/usr/include is not present
    # Note: --prefix=/usr is magic!  See http://www.gnu.org/software/libc/FAQ.html#s-2.2
fi

# Make lib directory in sysroot, else the ../lib64 hack used by 32 -> 64 bit
# crosscompilers won't work, and build of final gcc will fail with 
#  "ld: cannot open crti.o: No such file or directory"
mkdir -p $SYSROOT/lib
mkdir -p $SYSROOT/usr/lib

echo
echo "Building for --target=$TARGET, --prefix=$PREFIX"

#---------------------------------------------------------
echo Build binutils

mkdir -p build-binutils; cd build-binutils

if test '!' -f Makefile; then
    ${BINUTILS_DIR}/configure $CANADIAN_BUILD --target=$TARGET --host=$GCC_HOST --prefix=$PREFIX --disable-nls ${BINUTILS_EXTRA_CONFIG} $BINUTILS_SYSROOT_ARG
fi

make $PARALLELMFLAGS all 
make install 

cd ..

# test to see if this step passed
test -x ${PREFIX}/bin/${TARGET}-ld || abort Build failed during binutils 

#---------------------------------------------------------
echo "Install cygwin headers needed to build bootstrap compiler."

cd ${CYGWIN_DIR}

cp -a include ${SYSROOT}
cp -a newlib/libc/include ${SYSROOT}
cp -af winsup/cygwin/include ${SYSROOT}

cd ${BUILD_DIR}

#---------------------------------------------------------
echo "Build gcc-core (just enough to build glibc)"

mkdir -p build-gcc-core; cd build-gcc-core

# Use --with-local-prefix so older gccs don't look in /usr/local (http://gcc.gnu.org/PR10532)

if test '!' -f Makefile; then
    ${GCC_DIR}/configure $CANADIAN_BUILD --target=$TARGET --host=$GCC_HOST --prefix=$PREFIX \
	--with-local-prefix=${SYSROOT} \
	--disable-multilib \
	--with-newlib \
        ${GCC_EXTRA_CONFIG} \
	${GCC_SYSROOT_ARG_CORE} \
	--disable-nls \
	--enable-threads=no \
	--enable-symvers=gnu \
        --enable-languages="c,c++" \
	--disable-shared
fi

test "$CANADIAN_BUILD" = "" || make $PARALLELMFLAGS all-build-libiberty || true
make $PARALLELMFLAGS all
make install

cd ..

test -x ${PREFIX}/bin/${TARGET}-gcc || abort Build failed during gcc-core 

#---------------------------------------------------------
echo "Build cygwin."

mkdir -p build-cygwin; cd build-cygwin

${CYGWIN_DIR}/configure --target=${TARGET} \
                        --host=${GCC_HOST} \
                        --prefix=${PREFIX}
make all
make install
cd ..

test -f ${SYSROOT}/lib/libc.a || abort Building cygwin failed

#---------------------------------------------------------
echo Build final gcc

mkdir -p build-gcc; cd build-gcc

if test '!' -f Makefile; then
    # --enable-symvers=gnu really only needed for sh4 to work around a detection problem
    # only matters for gcc-3.2.x and later, I think
    # --disable-nls to work around crash bug on ppc405, but also because embedded
    # systems don't really need message catalogs...
    ${GCC_DIR}/configure $CANADIAN_BUILD --target=$TARGET --host=$GCC_HOST --prefix=$PREFIX \
        ${GCC_EXTRA_CONFIG} \
        $GCC_SYSROOT_ARG \
	--with-local-prefix=${SYSROOT} \
	--disable-nls \
	--enable-threads=posix \
	--enable-symvers=gnu \
        --enable-languages="$GCC_LANGUAGES" \
        $SHARED_MODE \
	--enable-c99 \
        --enable-long-long
fi

test "$CANADIAN_BUILD" = "" || make $PARALLELMFLAGS all-build-libiberty || true
make $PARALLELMFLAGS all 
make install 

# FIXME: shouldn't people who want this just --disable-multilib in final gcc and be done with it?
echo "kludge: If the chip does not have a floating point unit "
echo "(i.e. if GLIBC_EXTRA_CONFIG contains --without-fp),"
echo "and there are shared libraries in /lib/nof, copy them to /lib"
echo "so they get used by default."
echo "FIXME: only rs6000/powerpc seem to use nof.  See MULTILIB_DIRNAMES"
echo "in $GCC_DIR/gcc/config/$TARGET/* to see what your arch calls it."
case "$GLIBC_EXTRA_CONFIG" in
   *--without-fp*)
      if test -d ${SYSROOT}/lib/nof; then
          cp -af ${SYSROOT}/lib/nof/*.so* ${SYSROOT}/lib || echo "Warning: lib/nof not found.  Ignoring."
      fi
      ;;
esac

cd ..

test -x ${PREFIX}/bin/${TARGET}-gcc || Build failed during final gcc 

# Set up to let user install individual shared libraries in /etc/ld.so.conf easily
cd $TOP_DIR
sh mkoverride.sh

#---------------------------------------------------------
echo Cross-toolchain build complete.  Result in ${PREFIX}.
exit 0

