#!/bin/sh
set -x -e

abort() {
    echo $@
    exec false
}

#
# crosstest.sh
# Test a GNU/Linux toolchain
#
# Copyright (c) 2003 by Ixia Communications.
# Written by Dan Kegel, dkegel@ixiacom.com, 
# All rights reserved.  This script is provided under the terms of the GPL.
# For questions, comments or improvements see the crossgcc mailing
# list at http://sources.redhat.com/ml/crossgcc, or contact the
# authors, but do your homework first.  As Bill Gatliff says, "THINK!"
#
# Meant to be invoked from another shell script.
# Usage: eight environment variables must be set, namely:
test -z "${PREFIX}"         && abort "Please set PREFIX to where you want the toolchain installed."
test -z "${BUILD_DIR}"      && abort "Please set BUILD_DIR to the directory build-gcc etc. live in"
test -z "${BINUTILS_DIR}"   && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${GCC_DIR}"        && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${GLIBC_DIR}"      && abort "Please set GLIBC_DIR to the bare filename of the glibc tarball or directory"
test -z "${TARGET}"         && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${TARGET_CFLAGS}"  && abort "Please set TARGET_CFLAGS to any compiler flags needed when building glibc (-O recommended)"
test -z "${TOP_DIR}"        && abort "Please set TOP_DIR to where the crosstool scripts live"

if test -z "${DEJAGNU}"; then
    DEJAGNU=$TOP_DIR/boards/master.exp
    export DEJAGNU
    echo  "Setting DEJAGNU to $DEJAGNU"
fi

# Check for a few prerequisites that have tripped people up.
expect -v  || abort "Please install 'expect' or add it to your PATH.  (See also http://gcc.gnu.org/PR12096)"

# Make all paths absolute (it's so confusing otherwise).  This may break if you use an automounter.  FIXME
PREFIX=`cd $PREFIX; pwd`
BINUTILS_DIR=`cd $BINUTILS_DIR; pwd`

STRIPDIR=$PREFIX/bin
STRIP=`echo $STRIPDIR/*-strip`
test -x "$STRIP" || abort "Error: $STRIP not executable"

# make sure the build product's binaries are in the search path
# Also make sure our own private copy of dejagnu is used
PATH="${PREFIX}/bin:$TOP_DIR/result/dejagnu/bin:${PATH}"
export PATH
test -x $TOP_DIR/result/dejagnu/bin/runtest || abort 'Please run mkdejagnu.sh to build a private copy of dejagnu'

# make sure we won't run out of rsh ports
test -f /proc/sys/net/ipv4/tcp_tw_recycle && test `cat /proc/sys/net/ipv4/tcp_tw_recycle` = 0 && abort 'Please set /proc/sys/net/ipv4/tcp_tw_recycle to 1'

# Set up the files boards/master.exp and boards/$TARGET.exp for dejagnu

mkdir -p $TOP_DIR/boards
cat > $TOP_DIR/boards/master.exp <<_EOF_
lappend boards_dir $TOP_DIR/boards
lappend boards_dir /usr/share/dejagnu
set myboard \$target_triplet
set target_list [list \$target_triplet]
_EOF_

cat > $TOP_DIR/boards/$TARGET.exp <<_EOF_
load_generic_config "unix";
set_board_info hostname $TARGET
set_board_info username root-jail
# Note: compiler paths must point into gcc build directory, or g++ regression tests will fail strangely
set_board_info compiler $BUILD_DIR/build-gcc/gcc/xgcc
set_board_info c++compiler $BUILD_DIR/build-gcc/gcc/xg++
# Uncomment to build static binaries (in case shared libraries fail or you need to run outside the jail)
#set_board_info cflags "-static"
# Uncomment to run outside the jail (if you only have a normal user account
# on the machine, or if the jail is broken somehow)
# This may cause DNS routines to load the wrong shared helper library, but that's minor.
#set_board_info username root
_EOF_

# Build and init the jail.  
# (Note: you can skip this if you're only testing static executables)
cd $BUILD_DIR

# Assume that remote test environment for $TARGET is at hostname $TARGET
REMOTE=$TARGET
rm -rf jail_etc_passwd || true
rcp root@$REMOTE:/jail/etc/passwd jail_etc_passwd
if test -x $BUILD_DIR/ptxdist-0.3.23/root/bin/busybox; then
	echo Grabbing busybox from ptxdist
	sh $TOP_DIR/mkjail.sh $PREFIX/$TARGET jail_etc_passwd $BUILD_DIR/ptxdist-0.3.23/root
else
	sh $TOP_DIR/mkjail.sh $PREFIX/$TARGET jail_etc_passwd
fi
# Transfer and install the chroot environment
rcp $TOP_DIR/initjail.sh root@$REMOTE:
# Run initjail script.  Sadly, exit status of initjail.sh does not propagate.
zcat jail.tar.gz | rsh -l root $REMOTE /bin/sh initjail.sh /jail
# Verify that initjail script ran (it won't if rm -rf failed because
# .nfsXXXX files are laying about,
# as when some app or shared library we're deleting is in use).
# by copying file that should exist.  If it doesn't, we abort.
rcp root@$REMOTE:/jail/lib/libm.so /tmp/bogus_libm.so.$$
rm /tmp/bogus_libm.so.$$

# Remove prior run's logfiles
find . -name '*.sum' | xargs rm -f
find . -name '*.log' | xargs rm -f

#--- glibc tests ---
if true; then
cd build-glibc

make tests

# Use a crufty, execrable kludge to extract the commands needed to run glibc's tests.  
# Skip the stuff not needed in embedded systems, namely
#   locales 
#   timezones
#   tst-utmp*
# Skip tests which use more than 10MB of RAM:
#   reldep6
# Skip things hard to run with this kludge, namely:
#  shell scripts
#  tst-exec and tst-spawn, which need $(run-program-prefix) to be /lib/ld.so
# And skip tests that are only run on the host system, namely
#   tst-svc
# FIXME: glibc doesn't provide a way to run tst-svc unless you're doing a native build
REMOTE_TOP=
make -n check cross-compiling=no run-program-prefix=xyzzy > make.out
cat make.out \
    | egrep -v 'locale|timezone|tst-utmp|tst-exec|tst-spawn' \
    | egrep -v 'reldep6' \
    | egrep -v 'tst-svc' \
    | egrep 'xyzzy|Entering' \
    | sed 's,xyzzy,,g'  \
    | sed "s,$BUILD_DIR,$REMOTE_TOP,g" \
    | sed '1,/test-gencat.sh/d' \
    | sed 's/make.*: Entering directory `/cd /' \
    | sed "/^cd /s/'\$//" \
    | grep -v '/bin/sh' \
    | sed 's/^cd \(.*\)/mkdir -p \1; cd \1/' \
    | sed 's/$/|| echo Error $?/' \
    > ../tests.out

echo "#!/bin/sh" > ../glibctest.sh
echo "set -x" >> ../glibctest.sh
# be nice to the tests in dlfcn and elf, which contain hardcoded paths
echo "rm -rf $BUILD_DIR || true" >> ../glibctest.sh

# this fails on one of my systems with EDIRTOOLONG (glibc problem?), so break it up
#echo "mkdir -p $BUILD_DIR; ln -s /build-glibc $BUILD_DIR/build-glibc" >> ../glibctest.sh
echo "mkdir -p $TOP_DIR" >> ../glibctest.sh >> ../glibctest.sh
REST=`echo $BUILD_DIR | sed 's,$TOP_DIR/,,'`
echo "(cd $TOP_DIR; mkdir -p $REST; ln -s /build-glibc $REST/build-glibc)" >> ../glibctest.sh

# For some reason, several dlfcn and elf tests don't pass without LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH=../../build-glibc/elf:../../build-glibc/dlfcn; export LD_LIBRARY_PATH" >> ../glibctest.sh
cat ../tests.out >> ../glibctest.sh

# Grab files needed to run tests
cd ..
TESTPROGRAMS=`grep = tests.out | sed 's/[A-Z_]*=[^ ]* *//g;s/ .*//' | sort -u | sed "s,$REMOTE_TOP/,,"`
INPUTFILES="glibc*/*/*.input glibc*/stdio-common/xbug.c glibc*/io/Makefile glibc*/io/bug-ftw2.c"
TESTLIBS=`ls build-glibc/elf/*.so build-glibc/dlfcn/*.so | grep -v ld.so`
tar -czf test.tar.gz glibctest.sh $TESTPROGRAMS $INPUTFILES $TESTLIBS
# Strip the executables we're going to send to the target - makes a difference when compiled -g
rm -rf tmp || true
mkdir tmp
cd tmp
tar -xzf ../test.tar.gz
$TARGET-strip $TESTPROGRAMS
$TARGET-strip $TESTLIBS
tar -czf ../test.tar.gz *
cd ..
rm -rf tmp

# Transfer to target system and run.  
rsh -n -l root-jail $REMOTE rm -rf build-glibc
cat test.tar.gz | rsh -l root-jail $REMOTE tar -xzf -
rsh -n -l root-jail $REMOTE sh -e glibctest.sh > glibctest.out 2>&1

if grep Error glibctest.out; then
   echo "Some glibc test failed.  Check logfiles on target in /jail/build-glibc/*/*.out."
fi
# it's not really a summary yet, but let's copy it up anyway
cp glibctest.out $TOP_DIR/$TARGET-$GCC_DIR-$GLIBC_DIR.glibc.sum
fi

#--- gcc tests ---
cd build-gcc

# Run gcc's built-in regression test.
# Don't use "make check", as it just adds layers of complexity without any benefit

# First, do one test from each of several sections of the tests to get quick
# feedback about whether cross-compiling and remote execution is working
cd gcc
 make testsuite/site.exp
 cd testsuite
   runtest --tool=gcc --target=$TARGET -v -v -v -v -a execute.exp=20000112-1.c
   if grep UNRESOLVED gcc.log; then
      abort "Remote execution failed.  Make sure you can rcp a simple executable to $TARGET and execute it with rsh."
   fi

   runtest --tool=g++ --target=$TARGET -v -v -v -v -a dg.exp=bitfield1.C
   if grep UNRESOLVED g++.log; then
      abort "Sample g++ dg test failed.  Probably a bug in crosstest.sh."
   fi

   runtest --tool=g++ --target=$TARGET -v -v -v -v -a old-deja.exp=900520_06.C
   if grep UNRESOLVED g++.log; then
      abort "Sample g++ old-deja test failed.  Probably a bug in crosstest.sh."
   fi
cd ../..

# gcc-2.95 called the c++ library libstdc++.
# gcc-3.[0123] called the c++ library libstdc++-v3.
# gcc-3.4 calls the c++ library libstdc++, but it still lives in the libstdc++-v3 directory, grumble.
# See http://gcc.gnu.org/ml/libstdc++/2003-08/msg00034.html
if test -d $TARGET/libstdc++-v3; then
   LIBSTDCXX_DIR=libstdc++-v3
   if test -f $BUILD_DIR/$GCC_DIR/libstdc++-v3/testsuite/lib/libstdc++.exp; then
      LIBSTDCXX_TEST=libstdc++
   else
      LIBSTDCXX_TEST=libstdc++-v3
   fi
elif test -d $TARGET/libstdc++; then
   LIBSTDCXX_DIR=libstdc++
   LIBSTDCXX_TEST=libstdc++
else
   abort "can't find libstdc++ or libstdc++-v3"
fi

cd $TARGET/$LIBSTDCXX_DIR/testsuite
   make site.exp
   runtest --tool=$LIBSTDCXX_TEST --target=$TARGET -v -v -v -v -a dg.exp=sort.cc
   if grep UNRESOLVED $LIBSTDCXX_TEST.log; then
      abort "Sample $LIBSTDCXX_TEST test failed.  Probably a bug in crosstest.sh."
   fi
cd ../../..

# Now do full tests
cd gcc/testsuite
   runtest --tool=g++ --target=$TARGET -v || echo Some g++ test failed...
   runtest --tool=gcc --target=$TARGET -v || echo Some gcc test failed...
cd ../..

cd $TARGET/$LIBSTDCXX_DIR/testsuite
   runtest --tool=$LIBSTDCXX_TEST --target=$TARGET -v || echo Some $LIBSTDCXX_TEST test failed...
cd ../../..

# Run standard test log analysis tool
sh ../${GCC_DIR}/contrib/test_summary  | sed '1d;/EOF/,$d' > test_summary.log

cp test_summary.log $TOP_DIR/$TARGET-$GCC_DIR-$GLIBC_DIR.gcc.sum

cd ..

cd build-binutils

# Run binutils' built-in regression test.
RUNTESTFLAGS="--target=$TARGET -v" make -k check || true

rm -f test_summary.log || true
for file in binutils/binutils.sum gas/testsuite/gas.sum ld/ld.sum; do
	echo "[---------------------------- $file ---------------------------]" >> test_summary.log
	cat $file >> test_summary.log
	echo "" >> test_summary.log
done

cp test_summary.log $TOP_DIR/$TARGET-$GCC_DIR-$GLIBC_DIR.binutils.sum

cd ..

echo $0 done.
