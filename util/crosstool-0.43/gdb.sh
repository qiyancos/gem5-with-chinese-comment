#!/bin/sh
# Scriptlet to build a cross-gcc and a native gdbserver.
# Dan Kegel
# Copyright 2004, Andre Ancelin (andrea@adtecinc.com)
# Copyright 2005, Google
set -ex

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

test -z "${PREFIX}"           && abort "Please set PREFIX to where you want the toolchain installed."
test -z "${BUILD_DIR}"        && abort "Please set BUILD_DIR to the directory where the tools are to be built"
test -z "${SRC_DIR}"          && abort "Please set SRC_DIR to the directory where the source tarballs are to be unpacked"
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${GCC_HOST}"         && echo "GCC_HOST not set, assuming HOST=BUILD"
test -z "${GCC_BUILD}"        && echo "GCC_BUILD not set, assuming BUILD=output of config.guess"
TOP_DIR=${TOP_DIR-`pwd`}
BUILD=${GCC_BUILD-`$TOP_DIR/config.guess`}
test -z "$BUILD" && abort "bug: BUILD not set?!"
if test "$GCC_HOST" = ""; then
        GCC_HOST=$BUILD
fi
test -z "${GDB_DIR}"          && abort "Please set GDB_DIR to the bare filename of the gdb tarball or directory"
# If we're building compilers that run on Windows, remember that their
# filenames end in .exe
case "$GCC_HOST" in
*cygwin*) EXEEXT=".exe" ;;
*)        EXEEXT="" ;;
esac

PATH="${PREFIX}/bin:${PATH}"
export PATH

GDB_DIR=`cd ${SRC_DIR}/${GDB_DIR}; pwd`

cd $BUILD_DIR

#---------------------------------------------------------
echo Build gdb

mkdir -p build-gdb; cd build-gdb

if test '!' -f Makefile; then
    # Remember- gdb will run on the host using host resources.
    # As such, we compile to run natively BUT with the target of interest.
    ${GDB_DIR}/configure --target=$TARGET --host=$GCC_HOST --prefix=$PREFIX
fi

make $PARALLELMFLAGS all
make install

cd ..
 
logresult gdb ${PREFIX}/bin/${TARGET}-gdb${EXEEXT}
 
#---------------------------------------------------------
echo Build gdbserver

# FIXME: binary should be named $TARGET-gdbserver or located in e.g. $PREFIX/$TARGET 
# to follow gcc scheme of overlay multiple targets' toolchains in same $PREFIX

mkdir -p build-gdbserver; cd build-gdbserver

if test '!' -f Makefile; then
    # For some reason, and unlike $GDB_DIR/configure, $GDB_DIR/gdb/gdbserver/configure
    # is distrubuted with no executable permissions (at least for 5.2.1 & 6.0).
    # Gotta be executable, so we do so here.
    chmod +x ${GDB_DIR}/gdb/gdbserver/configure

    # As this runs on the $TARGET, we need to use the target's compiler tool chain.
    # Also note that we set --host to $TARGET, not --target. Same reason as above.
    CC=${TARGET}-gcc \
    ${GDB_DIR}/gdb/gdbserver/configure --host=$TARGET --prefix=$PREFIX ${GDBSERVER_EXTRA_CONFIG}
fi

make $PARALLELMFLAGS all
make install

cd ..

logresult gdbserver ${PREFIX}/bin/gdbserver

# Best to minimize impact on target resources, so this should almost always be stripped.
# If you need to debug the remote debugger you will not want to do this,
# though this should only apply to people using low level hardware based debuggers (JTAG, BDM, etc.)
# This file is the only required target file for remote debugging,
# and it should be copied into the target's /usr/bin directory.
# Well, ok, I lied.  This is a dynamic executable, so it requires
# three other files on the target: {/lib|/lib64}/{libc,libpthread,libpthread_db}.so.*
# We could link a static one, but that's tricky.  
# See e.g. http://sources.redhat.com/ml/gdb/2003-04/msg00230.html
${TARGET}-strip ${PREFIX}/bin/gdbserver


