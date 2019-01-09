#!/bin/sh

abort() {
    echo crosstool: $@
    exec false
}

# Used to log success or failure of each stage
logresult() {
    if test -x $2; then
        echo crosstool: $1 built ok
    else
        abort Build failed during $1
    fi
}

#
# crosstool.sh
# Build a GNU/Linux toolchain
#
# Copyright (c) 2001 by Bill Gatliff, bgat@billgatliff.com 
# Copyright (c) 2003 by Dan Kegel, Ixia Communications, 
# Copyright (c) 2004,2005 by Dan Kegel, Google, Inc.
# All rights reserved.  This script is provided under the terms of the GPL.
# For questions, comments or improvements see the crossgcc mailing
# list at http://sources.redhat.com/ml/crossgcc, or contact the
# authors, but do your homework first.  As Bill says, "THINK!"
#
# Meant to be invoked from another shell script.
# Usage: nine environment variables must be set, namely:
test -z "${PREFIX}"           && abort "Please set PREFIX to where you want the toolchain installed."
test -z "${BUILD_DIR}"        && abort "Please set BUILD_DIR to the directory where the tools are to be built"
test -z "${SRC_DIR}"          && abort "Please set SRC_DIR to the directory where the source tarballs are to be unpacked"
test -z "${BINUTILS_DIR}"     && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${GCC_DIR}"          && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${GLIBC_DIR}"        && abort "Please set GLIBC_DIR to the bare filename of the glibc tarball or directory"
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${TARGET_CFLAGS}"    && abort "Please set TARGET_CFLAGS to any compiler flags needed when building glibc (-O recommended)"
if test -z "${LINUX_SANITIZED_HEADER_DIR}" ; then
    test -z "${LINUX_DIR}"        && abort "Please set either LINUX_DIR or LINUX_SANITIZED_HEADER_DIR to the bare filename of the tarball or directory containing the kernel headers"
    LINUX_HEADER_DIR="${LINUX_DIR}"
else
    test -n "${LINUX_DIR}"        && echo "You set both LINUX_DIR and LINUX_SANITIZED_HEADER_DIR - ignoring LINUX_DIR"
    LINUX_HEADER_DIR="${LINUX_SANITIZED_HEADER_DIR}"
fi


# Seventeen or so are optional
if test -z "${GCC_CORE_DIR}"; then
    echo "GCC_CORE_DIR not set, so using $GCC_DIR for bootstrap compiler"
    GCC_CORE_DIR="${GCC_DIR}"
fi
test -z "${BINUTILS_EXTRA_CONFIG}" && echo "BINUTILS_EXTRA_CONFIG not set, so not passing any extra options to binutils' configure script"
test -z "${GCC_EXTRA_CONFIG}"      && echo "GCC_EXTRA_CONFIG not set, so not passing any extra options to gcc's configure script"
test -z "${GLIBC_EXTRA_CONFIG}"    && echo "GLIBC_EXTRA_CONFIG not set, so not passing any extra options to glibc's configure script"
test -z "${GLIBC_EXTRA_ENV}"       && echo "GLIBC_EXTRA_ENV not set, so not passing any extra environment variables to glibc's configure script"
test -z "${GLIBC_EXTRA_CC_ARGS}"   && echo "GLIBC_EXTRA_CC_ARGS not set, so not passing any extra options to gcc when building glibc"
test -z "${EXTRA_TARGET_CFLAGS}"   && echo "EXTRA_TARGET_CFLAGS not set, so not passing any extra cflags to gcc when building glibc"
test -z "${USE_SYSROOT}"           && echo "USE_SYSROOT not set, so not configuring with --with-sysroot"
test -z "${GCC_BUILD}"             && echo "GCC_BUILD not set, assuming BUILD=output of config.guess"
test -z "${GCC_HOST}"              && echo "GCC_HOST not set, assuming HOST=BUILD"
test -z "${KERNELCONFIG}" && test ! -f ${LINUX_DIR}/.config  && echo  "KERNELCONFIG not set, and no .config file found, so not configuring linux kernel"
test -z "${KERNELCONFIG}" || test -r "${KERNELCONFIG}"  || abort  "Can't read file KERNELCONFIG = $KERNELCONFIG, please fix."
test -z "${SHARED_MODE}" && SHARED_MODE="--enable-shared" && echo "SHARED_MODE not set, so defaulting to --enable-shared"
test -z "${GCC_LANGUAGES}"         && echo "GCC_LANGUAGES not set, assuming c,c++"
GCC_LANGUAGES=${GCC_LANGUAGES-"c,c++"}
TOP_DIR=${TOP_DIR-`pwd`}
chmod 755 $TOP_DIR/config.guess
BUILD=${GCC_BUILD-`$TOP_DIR/config.guess`}
test -z "$BUILD" && abort "bug: BUILD not set?!"

if test -z "${GLIBC_ADDON_OPTIONS}"; then
   echo "GLIBC_ADDON_OPTIONS not set, so guessing addons from GLIBCTHREADS_FILENAME and GLIBCCRYPT_FILENAME"
   # this is lame, need to fix this for nptl later?
   # (nptl is an addon, but it's shipped in the main tarball)
   GLIBC_ADDON_OPTIONS="="
   case "${GLIBCTHREADS_FILENAME}" in
     *linuxthreads*) GLIBC_ADDON_OPTIONS="${GLIBC_ADDON_OPTIONS}linuxthreads," ;;
   esac
   # crypt is only an addon for glibc-2.1.x
   test -z "${GLIBCCRYPT_FILENAME}"   || GLIBC_ADDON_OPTIONS="${GLIBC_ADDON_OPTIONS}crypt,"
fi

# Add some default glibc config options if not given by user.  These used to be hardcoded.
DEFAULT_GLIBC_EXTRA_CONFIG=""
case "${GLIBC_EXTRA_CONFIG}" in
*enable-kernel*) ;;
*) DEFAULT_GLIBC_EXTRA_CONFIG="${DEFAULT_GLIBC_EXTRA_CONFIG} --enable-kernel=2.4.3"
esac
case "${GLIBC_EXTRA_CONFIG}" in
*-tls*) ;;
*) DEFAULT_GLIBC_EXTRA_CONFIG="${DEFAULT_GLIBC_EXTRA_CONFIG} --without-tls"
esac
case "${GLIBC_EXTRA_CONFIG}" in
*-__thread*) ;;
*) DEFAULT_GLIBC_EXTRA_CONFIG="${DEFAULT_GLIBC_EXTRA_CONFIG} --without-__thread"
esac

# One is forbidden
test -z "${LD_LIBRARY_PATH}" || abort  "glibc refuses to build if LD_LIBRARY_PATH is set.  Please unset it before running this script."

# And one is derived if unset.
test -z "${GLIBCTHREADS_FILENAME}" &&
GLIBCTHREADS_FILENAME=`echo $GLIBC_DIR | sed 's/glibc-/glibc-linuxthreads-/'`

# Check for a few prerequisites that have tripped people up.
awk '/x/' < /dev/null  || abort "You need awk to build a toolchain."
test -z "${CFLAGS}"    || abort "Don't set CFLAGS, it screws up the build"
test -z "${CXXFLAGS}"  || abort "Don't set CXXFLAGS, it screws up the build"
bison --version > /dev/null || abort "You don't have bison installed"
flex --version > /dev/null || abort "You don't have flex installed"

#---------------------------------------------------------

# Save configuration; output every envrionment variable these
# scripts reference to a text file for future reference.  Name the
# text file after the target so in a future version of crosstool,
# when we start installing toolchains for several targets into
# the same PREFIX, we don't overwrite.
# FIXME: this is a bit fragile; when adding variables elsewhere,
# it's easy to forget to add them here.
# FIXME: this documents some variables obeyed only by e.g.
# getandpatch.sh and all.sh, so if you invoked crosstool.sh
# by hand, some of these might be misleading.
# NOTE: surround with "Begin/end" and echo to stdout so we can grep out of the log later.

echo "Begin saving environment"
> $PREFIX/$TARGET.crosstoolconfig.txt

set -x
for var in \
AR \
BINUTILS_DIR \
BINUTILS_EXTRA_CONFIG \
BUILD \
BUILD_DIR \
CC \
DEJAGNU \
EXTRA_TARGET_CFLAGS \
GCC_BUILD \
GCC_CORE_DIR \
GCC_DIR \
GCC_EXTRA_CONFIG \
GCC_HOST \
GCC_LANGUAGES \
GDB_DIR \
GLIBC_ADDON_OPTIONS \
GLIBC_DIR \
GLIBC_EXTRA_CC_ARGS \
GLIBC_EXTRA_CONFIG \
GLIBC_EXTRA_ENV \
JUST_DOWNLOAD \
KERNELCONFIG \
LINUX_DIR \
LINUX_SANITIZED_HEADER_DIR \
NO_DOWNLOAD  \
PREFIX \
PTXDIST_DIR \
SHARED_MODE \
SRC_DIR \
TARBALLS_DIR \
TARGET \
TARGET_CFLAGS \
TOP_DIR \
USE_SYSROOT \
; do 
  eval echo $var=\$$var
  eval echo $var=\$$var >> $PREFIX/$TARGET.crosstoolconfig.txt
done
set +x
echo "End saving environment"

#---------------------------------------------------------

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
# Kludge: it is reported that the above causes canadian crosses with
# cygwin hosts to fail, so avoid it just in that one case.  It would be
# cleaner to just move this into the non-canadian case
# above, but I'm afraid that might cause some configure script somewhere
# to decide that since build==host, they could run host binaries.
#
# if host is cygwin and this is not a canadian build, modify GCC_HOST
case "$GCC_HOST,$CANADIAN_BUILD," in
*cygwin*,?*,) ;;
*)            GCC_HOST=`echo $GCC_HOST | sed s/-/-host_/` ;;
esac


# If we're building compilers that run on Windows, remember that their
# filenames end in .exe
case "$GCC_HOST" in
*cygwin*) EXEEXT=".exe" ;;
*)        EXEEXT="" ;;
esac

set -ex

# map TARGET to Linux equivalent
case $TARGET in
    alpha*)   ARCH=alpha ;;
    arm*)     ARCH=arm ;;
    cris*)    ARCH=cris ;;
    hppa*)    ARCH=parisc ;;
    i*86*|pentium*)    ARCH=i386 ;;
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
GCC_CORE_DIR=`cd ${SRC_DIR}/${GCC_CORE_DIR}; pwd`
LINUX_HEADER_DIR=`cd ${SRC_DIR}/${LINUX_HEADER_DIR}; pwd`
GLIBC_DIR=`cd ${SRC_DIR}/${GLIBC_DIR}; pwd`

# Always install the bootstrap gcc (used to build glibc)
# somewhere it can't interfere with the final gcc.
CORE_PREFIX=$BUILD_DIR/gcc-core-prefix

# If user isn't doing a canadian cross, add the target compiler's bin to
# the path, so we can use the compiler we build to build glibc etc.
if test "$CANADIAN_BUILD" = ""; then
        PATH="${PREFIX}/bin:$CORE_PREFIX/bin:${PATH}"
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
    GLIBC_SYSROOT_ARG=prefix=
else
    # spiffy new sysroot way.  libraries split between
    # prefix/target/sys-root/lib and prefix/target/sys-root/usr/lib
    SYSROOT=${PREFIX}/${TARGET}/sys-root
    HEADERDIR=$SYSROOT/usr/include
    BINUTILS_SYSROOT_ARG="--with-sysroot=${SYSROOT}"
    GCC_SYSROOT_ARG="--with-sysroot=${SYSROOT}"
    GCC_SYSROOT_ARG_CORE=$GCC_SYSROOT_ARG
    GLIBC_SYSROOT_ARG=""
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
# Use sanitized headers, if available
if test -z "$LINUX_SANITIZED_HEADER_DIR" ; then
    echo Prepare kernel headers
else
    echo Copy sanitized headers
fi

cd $LINUX_HEADER_DIR
mkdir -p $HEADERDIR

# no indentation for now because indentation levels are rising too high
if test -z "$LINUX_SANITIZED_HEADER_DIR" ; then

if test -f "$KERNELCONFIG" ; then
    cp $KERNELCONFIG .config
fi
if test -f .config; then
    yes "" | make ARCH=$ARCH oldconfig
fi

# autodetect kernel version from contents of Makefile
KERNEL_VERSION=`awk '/^VERSION =/ { print $3 }' $LINUX_HEADER_DIR/Makefile`
KERNEL_PATCHLEVEL=`awk '/^PATCHLEVEL =/ { print $3 }' $LINUX_HEADER_DIR/Makefile`

case "$KERNEL_VERSION.$KERNEL_PATCHLEVEL.x" in
2.2.x|2.4.x) make ARCH=$ARCH symlinks    include/linux/version.h
             ;;
2.6.x)       case $ARCH in
             sh*)        # sh does secret stuff in 'make prepare' that can't be triggered separately,
                         # but happily, it doesn't use target gcc, so we can use it.
                         # Update: this fails on 2.6.11, as it installs elfconfig.h, which requires target compiler :-(
                         make ARCH=$ARCH prepare include/linux/version.h
                         ;;
             arm*|cris*) make ARCH=$ARCH include/asm include/linux/version.h include/asm-$ARCH/.arch
                         ;;
             mips*)      # for linux-2.6, 'make prepare' for mips doesn't 
                         # actually create any symlinks.  Hope generic is ok.
                         # Note that glibc ignores all -I flags passed in CFLAGS,
                         # so you have to use -isystem.
                         make ARCH=$ARCH include/asm include/linux/version.h
                         TARGET_CFLAGS="$TARGET_CFLAGS -isystem $LINUX_HEADER_DIR/include/asm-mips/mach-generic"
                         ;;
             *)          make ARCH=$ARCH include/asm include/linux/version.h
                         ;;
             esac
             ;;
*)           abort "Unsupported kernel version $KERNEL_VERSION.$KERNEL_PATCHLEVEL"
esac
cp -r include/asm-generic $HEADERDIR/asm-generic

fi # test -z "$LINUX_SANITIZED_HEADER_DIR"

cp -r include/linux $HEADERDIR
cp -r include/asm-${ARCH} $HEADERDIR/asm

cd $BUILD_DIR

#---------------------------------------------------------
echo Build binutils

mkdir -p build-binutils; cd build-binutils

if test '!' -f Makefile; then
    ${BINUTILS_DIR}/configure $CANADIAN_BUILD --target=$TARGET --host=$GCC_HOST --prefix=$PREFIX --disable-nls ${BINUTILS_EXTRA_CONFIG} $BINUTILS_SYSROOT_ARG
fi

make $PARALLELMFLAGS all 
make install 

if test x"$CORE_PREFIX" != x"$PREFIX"; then
    # if we're using a different core compiler, make binutils available to it
    # gcc searches in $CORE_PREFIX/$TARGET/bin for tools like 'ar', 'as', and 'ld'
    # instead of the location its configure script claims it searches (gcc_cv_as), grr
    mkdir -p $CORE_PREFIX/$TARGET/bin
    for tool in ar as ld strip; do
       # Remove old symlink to avoid clash on rerun
       # ln -snf is safe, but not portable.  
       # Can't test for existence of symlinks reliably
       # rm -f returns nonzero status on Solaris if it fails, so || true to keep script from aborting
       rm -f $CORE_PREFIX/$TARGET/bin/$tool || true
       ln -s $PREFIX/bin/$TARGET-$tool $CORE_PREFIX/$TARGET/bin/$tool
    done
fi

cd ..

# test to see if this step passed
logresult binutils ${PREFIX}/bin/${TARGET}-ld${EXEEXT}

#---------------------------------------------------------
echo "Install glibc headers needed to build bootstrap compiler -- but only if gcc-3.x"

# Only need to install bootstrap glibc headers for gcc-3.0 and above?  Or maybe just gcc-3.3 and above?
# See also http://gcc.gnu.org/PR8180, which complains about the need for this step.
# Don't install them if they're already there (it's really slow)
if grep -q 'gcc-[34]' ${GCC_CORE_DIR}/ChangeLog && test '!' -f $HEADERDIR/features.h; then
    mkdir -p build-glibc-headers; cd build-glibc-headers

    if test '!' -f Makefile; then
        # The following three things have to be done to build glibc-2.3.x, but they don't hurt older versions.
        # 1. override CC to keep glibc's configure from using $TARGET-gcc. 
        # 2. disable linuxthreads, which needs a real cross-compiler to generate tcb-offsets.h properly
        # 3. build with gcc 3.2 or later
        # Compare these options with the ones used when building glibc for real below - they're different.
        # As of glibc-2.3.2, to get this step to work for hppa-linux, you need --enable-hacker-mode
        # so when configure checks to make sure gcc has access to the assembler you just built...
        # Alternately, we could put ${PREFIX}/${TARGET}/bin on the path.
        # Set --build so maybe we don't have to specify "cross-compiling=yes" below (haven't tried yet)
        # Note: the warning
        # "*** WARNING: Are you sure you do not want to use the `linuxthreads'"
        # *** add-on?"
        # is ok here, since all we want are the basic headers at this point.
        # Override libc_cv_ppc_machine so glibc-cvs doesn't complain
        # 'a version of binutils that supports .machine "altivec" is needed'.
        libc_cv_ppc_machine=yes \
        CC=gcc \
            ${GLIBC_DIR}/configure --prefix=/usr \
            --build=$BUILD --host=$TARGET \
            --without-cvs --disable-sanity-checks --with-headers=$HEADERDIR \
            --enable-hacker-mode
    fi

    if grep -q GLIBC_2.3 ${GLIBC_DIR}/ChangeLog; then
        # glibc-2.3.x passes cross options to $(CC) when generating errlist-compat.c, which fails without a real cross-compiler.
        # Fortunately, we don't need errlist-compat.c, since we just need .h files, 
        # so work around this by creating a fake errlist-compat.c and satisfying its dependencies.
        # Another workaround might be to tell configure to not use any cross options to $(CC).
        # The real fix would be to get install-headers to not generate errlist-compat.c.
        # Note: BOOTSTRAP_GCC is used by patches/glibc-2.3.5/glibc-mips-bootstrap-gcc-header-install.patch
        libc_cv_ppc_machine=yes \
                make CFLAGS=-DBOOTSTRAP_GCC sysdeps/gnu/errlist.c
        mkdir -p stdio-common
        # sleep for 2 seconds for benefit of filesystems with lousy time resolution, like FAT,
        # so make knows for sure errlist-compat.c doesn't need generating
        sleep 2
        touch stdio-common/errlist-compat.c
    fi
    # Note: BOOTSTRAP_GCC is used by patches/glibc-2.3.5/glibc-mips-bootstrap-gcc-header-install.patch
    libc_cv_ppc_machine=yes \
    make cross-compiling=yes install_root=${SYSROOT} CFLAGS=-DBOOTSTRAP_GCC $GLIBC_SYSROOT_ARG install-headers

    # Two headers -- stubs.h and features.h -- aren't installed by install-headers,
    # so do them by hand.  We can tolerate an empty stubs.h for the moment.
    # See e.g. http://gcc.gnu.org/ml/gcc/2002-01/msg00900.html

    mkdir -p $HEADERDIR/gnu
    touch $HEADERDIR/gnu/stubs.h
    cp ${GLIBC_DIR}/include/features.h $HEADERDIR/features.h
    # Building the bootstrap gcc requires either setting inhibit_libc, or
    # having a copy of stdio_lim.h... see
    # http://sources.redhat.com/ml/libc-alpha/2003-11/msg00045.html
    cp bits/stdio_lim.h $HEADERDIR/bits/stdio_lim.h

    # Following error building gcc-4.0.0's gcj:
    #  error: bits/syscall.h: No such file or directory
    # solved by following copy; see
    # http://sourceware.org/ml/crossgcc/2005-05/msg00168.html
    # but it breaks arm, see http://sourceware.org/ml/crossgcc/2006-01/msg00091.html
    # so uncomment this if you need it
    #cp misc/syscall-list.h $HEADERDIR/bits/syscall.h

    cd ..
fi

#---------------------------------------------------------
echo "Build gcc-core (just enough to build glibc)"

mkdir -p build-gcc-core; cd build-gcc-core

echo Copy headers to install area of bootstrap gcc, so it can build libgcc2
mkdir -p $CORE_PREFIX/$TARGET/include
cp -r $HEADERDIR/* $CORE_PREFIX/$TARGET/include

# Use --with-local-prefix so older gccs don't look in /usr/local (http://gcc.gnu.org/PR10532)
# Use funky prefix so it doesn't contaminate real prefix, in case GCC_DIR != GCC_CORE_DIR

if test '!' -f Makefile; then
    ${GCC_CORE_DIR}/configure $CANADIAN_BUILD --target=$TARGET --host=$GCC_HOST --prefix=$CORE_PREFIX \
        --with-local-prefix=${SYSROOT} \
        --disable-multilib \
        --with-newlib \
        ${GCC_EXTRA_CONFIG} \
        ${GCC_SYSROOT_ARG_CORE} \
        --disable-nls \
        --enable-threads=no \
        --enable-symvers=gnu \
        --enable-__cxa_atexit \
        --enable-languages=c \
        --disable-shared
fi

test "$CANADIAN_BUILD" = "" || make $PARALLELMFLAGS all-build-libiberty || true
make $PARALLELMFLAGS all-gcc 
make install-gcc

cd ..

logresult gcc-core $CORE_PREFIX/bin/${TARGET}-gcc${EXEEXT}

#---------------------------------------------------------
echo Build glibc and linuxthreads

mkdir -p build-glibc; cd build-glibc

# sh4 really needs to set configparms as of gcc-3.4/glibc-2.3.2
# note: this is awkward, doesn't work well if you need more than one line in configparms
echo ${GLIBC_CONFIGPARMS} > configparms

if test '!' -f Makefile; then
    # Configure with --prefix the way we want it on the target...
    # There are a whole lot of settings here.  You'll probably want
    # to read up on what they all mean, and customize a bit, possibly by setting GLIBC_EXTRA_CONFIG
    # Compare these options with the ones used when installing the glibc headers above - they're different.
    # Adding "--without-gd" option to avoid error "memusagestat.c:36:16: gd.h: No such file or directory" 
    # See also http://sources.redhat.com/ml/libc-alpha/2000-07/msg00024.html. 
    # Set BUILD_CC, or you won't be able to build datafiles
    # Set --build, else glibc-2.3.2 will think you're not cross-compiling, and try to run the test programs

    # For glibc 2.3.4 and later we need to set some autoconf cache
    # variables, because nptl/sysdeps/pthread/configure.in does not
    # work when cross-compiling.
    if test -d ${GLIBC_DIR}/nptl; then
	libc_cv_forced_unwind=yes
	libc_cv_c_cleanup=yes
	export libc_cv_forced_unwind libc_cv_c_cleanup
    fi

    BUILD_CC=gcc CFLAGS="$TARGET_CFLAGS $EXTRA_TARGET_CFLAGS" CC="${TARGET}-gcc $GLIBC_EXTRA_CC_ARGS" \
    AR=${TARGET}-ar RANLIB=${TARGET}-ranlib \
        ${GLIBC_DIR}/configure --prefix=/usr \
        --build=$BUILD --host=$TARGET \
        ${GLIBC_EXTRA_CONFIG} ${DEFAULT_GLIBC_EXTRA_CONFIG} \
        --without-cvs --disable-profile --disable-debug --without-gd \
        $SHARED_MODE \
        --enable-add-ons${GLIBC_ADDON_OPTIONS} --with-headers=$HEADERDIR
fi

if grep -l '^install-lib-all:' ${GLIBC_DIR}/Makerules > /dev/null; then
    # nptl-era glibc.
    # If the install-lib-all target (which is added by our make-install-lib-all.patch) is present,
    # it means we're building glibc-2.3.3 or later, and we can't build programs yet,
    # as they require libeh, which won't be installed until full build of gcc
    GLIBC_INITIAL_BUILD_RULE=lib
    GLIBC_INITIAL_INSTALL_RULE="install-lib-all install-headers"
    GLIBC_INSTALL_APPS_LATER=yes
else
    # classic glibc.  
    # We can build and install everything with the bootstrap compiler.
    GLIBC_INITIAL_BUILD_RULE=all
    GLIBC_INITIAL_INSTALL_RULE=install
    GLIBC_INSTALL_APPS_LATER=no
fi
# If this fails with an error like this:
# ...  linux/autoconf.h: No such file or directory 
# then you need to set the KERNELCONFIG variable to point to a .config file for this arch.
# The following architectures are known to need kernel .config: alpha, arm, ia64, s390, sh, sparc
# Note: LD and RANLIB needed by glibc-2.1.3's c_stub directory, at least on macosx
# No need for PARALLELMFLAGS here, Makefile already reads this environment variable
make LD=${TARGET}-ld RANLIB=${TARGET}-ranlib $GLIBC_INITIAL_BUILD_RULE
make install_root=${SYSROOT} $GLIBC_SYSROOT_ARG $GLIBC_INITIAL_INSTALL_RULE

# This doesn't seem to work when building a crosscompiler,
# as it tries to execute localedef using the just-built ld.so!?
#make localedata/install-locales install_root=${SYSROOT}

# Fix problems in linker scripts.
# 
# 1. Remove absolute paths
# Any file in a list of known suspects that isn't a symlink is assumed to be a linker script.
# FIXME: test -h is not portable
# FIXME: probably need to check more files than just these three...
# Need to use sed instead of just assuming we know what's in libc.so because otherwise alpha breaks
#
# 2. Remove lines containing BUG per http://sources.redhat.com/ml/bug-glibc/2003-05/msg00055.html,
# needed to fix gcc-3.2.3/glibc-2.3.2 targeting arm
#
# To make "strip *.so.*" not fail (ptxdist does this), rename to .so_orig rather than .so.orig
for file in libc.so libpthread.so libgcc_s.so; do
    for lib in lib lib64 usr/lib usr/lib64; do
      if test -f ${SYSROOT}/$lib/$file && test ! -h ${SYSROOT}/$lib/$file; then
        mv ${SYSROOT}/$lib/$file ${SYSROOT}/$lib/${file}_orig
        sed 's,/usr/lib/,,g;s,/usr/lib64/,,g;s,/lib/,,g;s,/lib64/,,g;/BUG in libc.scripts.output-format.sed/d' < ${SYSROOT}/$lib/${file}_orig > ${SYSROOT}/$lib/$file
      fi
    done
done
cd ..

test -f ${SYSROOT}/lib/libc.a || test -f ${SYSROOT}/lib64/libc.a || test -f ${SYSROOT}/usr/lib/libc.a || test -f ${SYSROOT}/usr/lib64/libc.a || abort Building libc failed

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
        --enable-__cxa_atexit \
        --enable-languages="$GCC_LANGUAGES" \
         $SHARED_MODE \
        --enable-c99 \
        --enable-long-long
fi

test "$CANADIAN_BUILD" = "" || make $PARALLELMFLAGS all-build-libiberty || true

# Idea from <cort.dougan at gmail.com>:
# Fix lib/lib64 confusion for GCC 3.3.3 on PowerPC64 and x86_64.
# GCC 3.4.0 and up don't suffer from this confusion, and don't need this kludge.
# FIXME: we should patch gcc's source rather than uglify crosstool.sh.
# FIXME: is this needed for gcc-3.3.[56]?

case `basename ${GCC_DIR}` in
  gcc-3.3.[34])
    case ${TARGET} in
        powerpc64-unknown-linux-gnu|x86_64-unknown-linux-gnu)
            for x in `find ${SYSROOT} -name lib -type d -empty`; do
                echo $x
                if [ -d `dirname $x`/lib64 ] ; then
                    rm -rf $x
                    ln -s `dirname $x`/lib64 $x
                fi
            done
            ;;
        *)
          ;;
    esac
    ;;
  *)
    ;;
esac

make $PARALLELMFLAGS all 
make install 

# FIXME: shouldn't people who want this just --disable-multilib in final gcc and be done with it?
# This code should probably be deleted, it was written long ago and hasn't been tested in ages.
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

logresult "final gcc" ${PREFIX}/bin/${TARGET}-gcc${EXEEXT}

# Finally, build and install glibc programs, now that libeh (if any) is installed
# Don't do this unless needed, 'cause it causes glibc-2.{1.3,2.2} to fail here with
# .../gcc-3.4.1-glibc-2.1.3/build-glibc/libc.so.6: undefined reference to `__deregister_frame_info'
# .../gcc-3.4.1-glibc-2.1.3/build-glibc/libc.so.6: undefined reference to `__register_frame_info'
if test x$GLIBC_INSTALL_APPS_LATER = xyes;
then
  cd build-glibc
  make LD=${TARGET}-ld RANLIB=${TARGET}-ranlib
  # note: should do full install and then fix linker scripts, but this is faster
  make install_root=${SYSROOT} $GLIBC_SYSROOT_ARG install-bin install-rootsbin install-sbin install-data install-others
fi

#---------------------------------------------------------

# Create masquerade directory $PREFIX/distributed/bin for distcc
# Relies on $EXEEXT being set to .exe if the compilers run on cygwin
export EXEEXT
cd $PREFIX
sh $TOP_DIR/masq.sh

# Make it easier to run apps built with new gcc on systems that don't
# have the libgcc_s.so or libstdc++.so, libssp.so, or libmudflap.so installed, by creating 
# directories containing a copy of libgcc_s.so, libstdc++.so, libssp.so and libmudflap.so alone.
cd $TOP_DIR
sh mkoverride.sh

# Build little program that lets user move resulting toolchain to different prefix
cd $TOP_DIR
gcc fix-embedded-paths.c -o $PREFIX/bin/fix-embedded-paths


#---------------------------------------------------------
echo Cross-toolchain build complete.  Result in ${PREFIX}.
exit 0

