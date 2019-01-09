#!/bin/sh
set -ex

# Build userspace stuff with ptxdist (http://www.pengutronix.de/software/ptxdist_en.html)
# but use existing toolchain

test -z "${TARGET}"           && abort "Please set TARGET to the Gnu target identifier (e.g. pentium-linux)"
test -z "${GCC_DIR}"          && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${GLIBC_DIR}"        && abort "Please set GLIBC_DIR to the bare filename of the glibc tarball or directory"
test -z "${PTXDIST_DIR}"      && abort "Please set PTXDIST_DIR to the bare filename of the ptxdist tarball or directory"

TOOLCOMBO=$GCC_DIR-$GLIBC_DIR
TOP_DIR=${TOP_DIR-`pwd`}
BUILD_DIR=$TOP_DIR/build/$TARGET/$TOOLCOMBO

TARBALLS_DIR=${TARBALLS_DIR-$TOP_DIR/tarballs}
RESULT_TOP=${RESULT_TOP-$TOP_DIR/result}
PREFIX=${PREFIX-$RESULT_TOP/$TOOLCOMBO/$TARGET}

PATH="$PREFIX/bin:$PATH"
export PATH

cd $BUILD_DIR/$PTXDIST_DIR

# For some reason, there's a src directory in cvs now.  Should be empty;
# let's replace it with a symlink to our tarballs area.
rm -rf src
ln -s $TARBALLS_DIR src

# pull in a config file that all the bits we want...
cp $TOP_DIR/ptx.config .config
# and point it at our already-compiled toolchain 
export PREFIX
sh scripts/settoolchain.sh 

make get
make extract
yes '' | make prepare
make compile
make install

# OK, system image in $PREFIX/target
# Pull in standard C libraries and binaries from $PREFIX/$TARGET
# Following lists of libraries and binaries may need updating from time to time
# FIXME: use some other method of deciding what to copy, like, say, everything

cd $PREFIX/$TARGET

for lib in \
 ld libBrokenLocale libSegFault libanl libc libcrypt libdl libgcc_s libgcc_s_nof libm \
 libmemusage libnsl libnss_compat libnss_dns libnss_files libnss_hesiod libnss_nis \
 libnss_nisplus libpcprofile libpthread libresolv librt libmudflap libssp libstdc++ libthread_db libutil; do
	ls     lib/$lib[-.]*so* || true
	ls usr/lib/$lib[-.]*so* || true
done 2> /dev/null | cpio -pvm $PREFIX/target

for prog in \
 catchsegv gencat getconf getent glibcbug iconv ldd locale \
 localedef mtrace pcprofiledump rpcgen sprof tzselect xtrace \
 iconvconfig ldconfig nscd nscd_nischeck rpcinfo sln zdump zic; do
	ls      bin/$prog || true
	ls  usr/bin/$prog || true
	ls     sbin/$prog || true
	ls usr/sbin/$prog || true
done 2> /dev/null |  cpio -pvm $PREFIX/target

echo userspace complete, result in $PREFIX/target
