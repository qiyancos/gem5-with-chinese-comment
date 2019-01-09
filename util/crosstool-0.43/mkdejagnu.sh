#!/bin/sh
set -xe
mkdir -p result/dejagnu
PREFIX=`pwd`/result/dejagnu

# Compile local patched copy of dejagnu (and eventually tcl and expect)
# for use by crosstool, since the versions shipped with most Linux
# distributions are too buggy 

wget -c http://mirrors.usc.edu/pub/gnu/dejagnu/dejagnu-1.4.3.tar.gz
# FIXME: use same tarball directory as crosstool.sh
tar -xzvf dejagnu-1.4.3.tar.gz
cd dejagnu-1.4.3
for a in ../patches/dejagnu-1.4.3/*.patch; do
	patch -p1 < $a
done
./configure --prefix=$PREFIX
make
make install

# new stuff below, not yet turned on
#----------------------------------------------------------------
# Note: Tcl, expect, and dejagnu are only needed for running gcc's 
# regression tests.  The standard version of expect has a bug that
# causes spurious test failures, so we have to build our own.
# The spurious test failures are 
# described by http://gcc.gnu.org/PR12096, and are
# fixed by patches/expect-5.39/pr12096.patch
# A few example failures this fixes:
# FAIL: gcc.dg/cpp/tr-warn3.c 
# FAIL: gcc.dg/format/c90-printf-1.c 
# FAIL: gcc.dg/format/c99-printf-1.c
# FAIL: gcc.dg/format/ext-1.c
# I haven't modified crosstest.sh to use this version of expect/dejagnu,
# and tcl/expect/dejagnu take FOREVER to build and install on cygwin,
# so comment out the rest of this for now.
# FIXME
exit 0

#----- Tcl -----
TCL=tcl8.4.5
TCLDIR=tcl8.4.5
# !$1#$@!& sourceforge.net doesn't support wget -c ?!
if test ! -f $TARBALLS_DIR/$TCL-src.tar.gz ; then
    wget -P $TARBALLS_DIR http://unc.dl.sourceforge.net/sourceforge/tcl/$TCL-src.tar.gz
fi
rm -rf $TCLDIR
tar -xzvf $TARBALLS_DIR/$TCL-src.tar.gz
cd $TCLDIR
for a in ../../patches/$TCL/*.patch; do
    if test -f $a; then
	patch -g0 -p1 < $a
    fi
done
cd unix
./configure --prefix=$PREFIX
make
make install
cd ../..

#----- Expect -----
EXPECT=expect-5.39.0
EXPECTDIR=expect-5.39
wget -P $TARBALLS_DIR -c http://expect.nist.gov/old/$EXPECT.tar.gz
rm -rf $EXPECTDIR
tar -xzvf $TARBALLS_DIR/$EXPECT.tar.gz
cd $EXPECTDIR
for a in ../../patches/$EXPECT/*.patch; do
    if test -f $a; then
	patch -g0 -p1 < $a
    fi
done
./configure --prefix=$PREFIX
make
make install
cd ..

#----- Dejagnu -----
DEJAGNU=dejagnu-1.4.3
wget -P $TARBALLS_DIR -c http://mirrors.usc.edu/pub/gnu/dejagnu/$DEJAGNU.tar.gz
rm -rf $DEJAGNU
tar -xzvf $TARBALLS_DIR/$DEJAGNU.tar.gz
cd $DEJAGNU
for a in ../../patches/$DEJAGNU/*.patch; do
    if test -f $a; then
	patch -g0 -p1 < $a
    fi
done
./configure --prefix=$PREFIX
make
make install
cd ..

