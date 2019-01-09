#!/bin/sh
# Download and unpack gnu toolchain source tarballs, and apply any local patches.
# Local patches are found in subdirectories of patches/ with the same name as the tarball but without .tar.gz
# Copyright 2003 Ixia Communications
# Licensed under the GPL
# 17-Sep-03: Modified for Newlib by Nicolas Moreau ngbmoreau@yahoo.com.au

set -xe

abort() {
	echo $@
	exec /bin/false
}

# Meant to be invoked from another shell script.
# Usage: eight environment variables must be set, namely:
test -z "${BINUTILS_DIR}"     && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${BUILD_DIR}"        && abort "Please set BUILD_DIR to the directory where the tools are to be built"
test -z "${GCC_DIR}"          && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${NEWLIB_DIR}"       && abort "Please set NEWLIB_DIR to the bare filename of the newlib tarball or directory"
test -z "${TARBALLS_DIR}"     && abort "Please set TARBALLS_DIR to the directory to download tarballs to."
test -z "${TARGET_CFLAGS}"    && abort "Please set TARGET_CFLAGS to any compiler flags needed when building glibc (-O recommended)"
test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"

# Download, unpack, and patch the given tarball.
# Assumes that the tarball unpacks to a name guessable from its url,
# and that patches already exist locally in a directory named after the tarball.
getUnpackAndPatch() {
	ARCHIVE_NAME=`echo $1 | sed 's,.*/,,;'`
        BASENAME=`echo $ARCHIVE_NAME | sed 's,\.tar\.gz$,,;s,\.tar\.bz2$,,;'`
	ZIP_METHOD=`echo $ARCHIVE_NAME | sed 's,.*\.tar\.,,;'`
	cd $TARBALLS_DIR

	# Download if not present
	# FIXME: supposedly red hat 9's wget can't fetch from gnu.org.  passive needed?
	test -f $ARCHIVE_NAME || wget -c $1
	test -f $ARCHIVE_NAME || { echo "file $ARCHIVE_NAME not found"; return 1 ; }

	cd $BUILD_DIR

	# unpack unconditionally
	rm -rf $BASENAME

	if test $ZIP_METHOD = "gz" ; then
	    if ! tar -xzvf $TARBALLS_DIR/$ARCHIVE_NAME; then
	        abort "Corrupt file $ARCHIVE_NAME"
	    fi
	elif test $ZIP_METHOD = "bz2"; then
	    if ! tar -xjvf $TARBALLS_DIR/$ARCHIVE_NAME; then
		abort "Corrupt file $ARCHIVE_NAME"
	    fi
	else abort "Bad compress format $ZIP_METHOD for tarball"
	fi


	# Apply any patches for this component
	# -f is required for patches that delete files, like
	# patches/glibc-2.2.5/hhl-powerpc-fpu.patch,
	# else patch will think the patch is reversed :-(
	cd $BASENAME
	for p in $TOP_DIR/patches/$BASENAME/*.patch; do
	    if test -f $p; then
	        patch -p1 -f < $p
	    fi
	done
	cd $TOP_DIR
}

# Remember where source is.
TOP_DIR=${TOP_DIR-`pwd`}

mkdir -p $BUILD_DIR $TARBALLS_DIR

# Download, unpack, and patch all the needed source tarballs,
getUnpackAndPatch $BINUTILS_URL/$BINUTILS_DIR.tar.bz2 || getUnpackAndPatch $BINUTILS_URL/$BINUTILS_DIR.tar.gz
# fixme: if it looks like a snapshot (e.g. 3.3-20030721), we should get it from ftp://gcc.gnu.org/pub/gcc/snapshots
case $GCC_DIR in
   gcc-3.4-*)
      dir=`echo $GCC_DIR | sed 's/gcc-//'`
      getUnpackAndPatch ftp://gcc.gnu.org/pub/gcc/snapshots/$dir/$GCC_DIR.tar.bz2 ;;
   *)
      getUnpackAndPatch ftp://ftp.gnu.org/pub/gnu/gcc/$GCC_DIR.tar.gz ;;
esac
#Get Newlib 
getUnpackAndPatch $NEWLIB_URL/$NEWLIB_DIR.tar.bz2 || getUnpackAndPatch $NEWLIB_URL/$NEWLIB_DIR.tar.gz

# gcc's contrib/test_summary expects version stamp, normally created by contrib/update_gcc
test -f $BUILD_DIR/$GCC_DIR/LAST_UPDATED || echo $GCC_DIR > $BUILD_DIR/$GCC_DIR/LAST_UPDATED
