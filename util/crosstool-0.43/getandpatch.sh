#!/bin/sh
# Download and unpack gnu toolchain source tarballs, and apply any local patches.
# Local patches are found in subdirectories of patches/ with the same name as the tarball but without .tar.gz
# Copyright 2003 Ixia Communications
# Licensed under the GPL
set -xe

abort() {
	echo $@
	exec false
}

# Meant to be invoked from another shell script.

# To disable all downloads, and use only already-downloaded tarballs,
# set NO_DOWNLOAD to a nonempty string.
# To just download tarballs and not unpack or patch,
# set JUST_DOWNLOAD to a nonempty string.

test -z "$NO_DOWNLOAD" || echo "NO_DOWNLOAD set, not downloading"
test -z "$JUST_DOWNLOAD" || echo "JUST_DOWNLOAD set, not unpacking"

# Usage: set the following environment variables:
test -z "${BINUTILS_DIR}"  && abort "Please set BINUTILS_DIR to the bare filename of the binutils tarball or directory"
test -z "${JUST_DOWNLOAD}" && test -z "${SRC_DIR}"  && abort "Please set SRC_DIR to the directory where the source tarballs are to be unpacked"
test -z "${GCC_DIR}"       && abort "Please set GCC_DIR to the bare filename of the gcc tarball or directory"
test -z "${GCC_CORE_DIR}"  && echo "GCC_CORE_DIR not set, so using $GCC_DIR for bootstrap compiler"
test -z "${GDB_DIR}"       && echo "GDB_DIR not set, so not downloading gdb sources"

# When building a cygwin target the following are not needed.
if test "${CYGWIN_DIR}" = ""; then
  test -z "${GLIBC_DIR}"        && abort "Please set GLIBC_DIR to the bare filename of the glibc tarball or directory"
  test -z "${LINUX_SANITIZED_HEADER_DIR}" && echo "Not downloading linux-libc-headers. Set LINUX_SANITIZED_HEADER_DIR to do so"
  test -z "${LINUX_DIR}"        && echo "Not downloading kernel sources. Set LINUX_DIR if you want to do so"
  # And one is derived if not set explicitly.
  test -z "${GLIBCTHREADS_FILENAME}" &&
  GLIBCTHREADS_FILENAME=`echo $GLIBC_DIR | sed 's/glibc-/glibc-linuxthreads-/'`
fi

test -z "${TARBALLS_DIR}"     && abort "Please set TARBALLS_DIR to the directory to download tarballs to."

# Make all paths absolute (it's so confusing otherwise)
# FIXME: this doesn't work well with some automounters
test -z "$JUST_DOWNLOAD" || SRC_DIR=`cd $SRC_DIR; pwd`

# Check if extractions should be verbose.
if test -z "$QUIET_EXTRACTIONS"; then VERBOSE=-v;else VERBOSE=; fi

# Pattern in a patch log to indicate failure
PATCHFAILMSGS="^No file to patch.  Skipping patch.|^Hunk .* FAILED at"

# Get a well-defined sort order so patches always in same order
LANG=C
export LANG

# Download the given file to $TARBALLS_DIR
downloadFile()
{
	echo downloadFile $1

	# Hacky little kludge.  Useful during buildsrpms.sh to avoid redownloading over and over.
	if test -n $TARBALLS_CACHE_DIR && test -f $TARBALLS_CACHE_DIR/`basename $1`; then
		cp $TARBALLS_CACHE_DIR/`basename $1` $TARBALLS_DIR
		return
	fi

	# If downloads are disabled, just fail 
	test -z "$NO_DOWNLOAD" || return 1

	case $1 in

	*glibc-200*gz)
		wget --tries=5 -P ${TARBALLS_DIR} -c $1 || wget --tries=5 --passive-ftp -P ${TARBALLS_DIR} -c $1 || \
		(cd $TARBALLS_DIR; sh $TOP_DIR/glibc-get.sh $1; )
		;; 
	*)
		# Note: if you need to use a proxy, try
		# export http_proxy=<proxy_host>:<port>
		wget --tries=5 -P ${TARBALLS_DIR} -c $1 || wget --tries=5 --passive-ftp -P ${TARBALLS_DIR} -c $1
		# FIXME: support curl on systems that don't have wget
		;;
	esac
}


# Download, unpack, and patch a tarball from any one of the given URLs.
# If the directory already exists, don't download and unpack it.
# If the tarball already exists, don't download it.
# Assumes that the tarball unpacks to a name guessable from its url,
# and that patches already exist locally in a directory named after the tarball.
getUnpackAndPatch() {
    set -x
    # Check to see if the tarball already exists
    exists=""
    for arg; do
        case $arg in
        *.gz|*.bz2|*.tgz) ;;
        *) abort "unknown suffix on url $arg" ;;
        esac

        ARCHIVE_NAME=`echo $arg | sed 's,.*/,,;'`
        BASENAME=`echo $ARCHIVE_NAME | sed 's,\.tar\.gz$,,;s,\.tar\.bz2$,,;s,\.tgz,,;'`

        # Done if already unpacked
        test -z "${JUST_DOWNLOAD}" && test -d ${SRC_DIR}/$BASENAME && { echo "directory $BASENAME already present"; return 0 ; }

        if test -f $TARBALLS_DIR/$ARCHIVE_NAME; then
            exists=$TARBALLS_DIR/$ARCHIVE_NAME
	    break
        fi
    done

    if test "x$exists" = "x"; then
        # Doesn't exist, so try urls until we fetch one
        for arg; do
            ARCHIVE_NAME=`echo $arg | sed 's,.*/,,;'`
            BASENAME=`echo $ARCHIVE_NAME | sed 's,\.tar\.gz$,,;s,\.tar\.bz2$,,;s,\.tgz,,;'`
            if downloadFile $arg && test -f ${TARBALLS_DIR}/$ARCHIVE_NAME ; then
                # we got it!
                break
            fi
        done
    fi

    test -f ${TARBALLS_DIR}/$ARCHIVE_NAME || abort "file $ARCHIVE_NAME not found"

    # If we're just downloading, don't unpack
    test -n "$JUST_DOWNLOAD" && return 0

    cd $SRC_DIR

    echo hmm maybe cd
    case $ARCHIVE_NAME in
    glibc-[a-z]*-2*) echo "It's a glibc addon, so cd into glibc"; cd $GLIBC_DIR ;;
    *) ;;
    esac
    set +x

    case $ARCHIVE_NAME in
    *.gz|*.tgz)
        tar $VERBOSE -xzf $TARBALLS_DIR/$ARCHIVE_NAME || abort cannot unpack $TARBALLS_DIR/$ARCHIVE_NAME ;;
    *.bz2)
        tar $VERBOSE -xjf $TARBALLS_DIR/$ARCHIVE_NAME || abort cannot unpack $TARBALLS_DIR/$ARCHIVE_NAME ;;
    *) 
        abort "Unrecognized suffix for tarball $ARCHIVE_NAME" ;;
    esac

    # Fix path of old linux source trees
    if [ -d linux ]; then
        mv linux $BASENAME
    fi
    # A few special distributions name the tree just kernel.
    if [ -d kernel ]; then
        mv kernel $BASENAME
    fi

    # Apply any patches for this component
    # -f is required for patches that delete files, like
    # patches/glibc-2.2.5/hhl-powerpc-fpu.patch,
    # else patch will think the patch is reversed :-(
    # Since -f tells patch to ignore failures, grep log to look for errors
    # use max --fuzz=1 since default fuzz is too dangerous for automation
    # Use -g0 else patch-2.5.8 on MacOSX tries to run perforce!
    if test -d $TOP_DIR/patches/$BASENAME; then
        case $ARCHIVE_NAME in
        glibc-[a-z]*-2*) ;;              # glibc addon, so we're already in right directory
        *)           cd $BASENAME ;;
        esac

        for p in $TOP_DIR/patches/$BASENAME/*patch* \
             $TOP_DIR/patches/$BASENAME/*.diff; do
        if test -f $p; then
            echo "applying patch $p"
            patch -g0 --fuzz=1 -p1 -f < $p > patch$$.log 2>&1 || { cat patch$$.log ; abort "patch $p failed" ; }
            cat patch$$.log
            egrep -q "$PATCHFAILMSGS" patch$$.log && abort "patch $p failed"
            rm -f patch$$.log
        fi
        done
    fi
}

# Remember where source is.
TOP_DIR=${TOP_DIR-`pwd`}

mkdir -p $TARBALLS_DIR
test -z "${JUST_DOWNLOAD}" || mkdir -p $SRC_DIR

# Download, unpack, and patch all the needed source tarballs,

# Grab it if we want to build userland...
if test -n "$PTXDIST_DIR" ; then
   getUnpackAndPatch http://www.kegel.com/crosstool/$PTXDIST_DIR.tar.gz
fi

if test -n "$GDB_DIR"; then
    getUnpackAndPatch ftp://ftp.gnu.org/pub/gnu/gdb/$GDB_DIR.tar.bz2 \
                      ftp://sources.redhat.com/pub/gdb/releases/$GDB_DIR.tar.bz2 \
                      ftp://sources.redhat.com/pub/gdb/old-releases/$GDB_DIR.tar.bz2 \
                      ftp://sources.redhat.com/pub/gdb/snapshots/current/$GDB_DIR.tar.bz2
fi

# No glibc for cygwin.
if test "${CYGWIN_DIR}" = ""; then
   case $GLIBC_DIR in
      glibc-200*) 
  	  getUnpackAndPatch \
		ftp://gcc.gnu.org/pub/glibc/snapshots/$GLIBC_DIR.tar.bz2 \
		ftp://gcc.gnu.org/pub/glibc/snapshots/$GLIBC_DIR.tar.gz ;;
      *)  
  	  getUnpackAndPatch \
		ftp://ftp.gnu.org/pub/gnu/glibc/$GLIBC_DIR.tar.bz2 \
		ftp://ftp.gnu.org/pub/gnu/glibc/$GLIBC_DIR.tar.gz \
		ftp://gcc.gnu.org/pub/glibc/releases/$GLIBC_DIR.tar.bz2 \
		ftp://gcc.gnu.org/pub/glibc/releases/$GLIBC_DIR.tar.gz ;;
   esac
else
  getUnpackAndPatch ${CYGWIN_URL}/${CYGWIN_DIR}-src.tar.bz2
fi
if test x"$BINUTILS_URL" = x; then
   case $BINUTILS_DIR in
      binutils-2.*.9*.0*) 
          BINUTILS_URL=http://www.kernel.org/pub/linux/devel/binutils ;;   # H.J.Lu's branch
      binutils-2.*.9*|binutils-05*)    
          BINUTILS_URL=ftp://gcc.gnu.org/pub/binutils/snapshots ;;
      *)  BINUTILS_URL=ftp://gcc.gnu.org/pub/binutils/releases ;;
   esac
fi
getUnpackAndPatch $BINUTILS_URL/$BINUTILS_DIR.tar.bz2 $BINUTILS_URL/$BINUTILS_DIR.tar.gz

for gcc in $GCC_DIR $GCC_CORE_DIR; do
 case $gcc in
   gcc-2.95.3)
      getUnpackAndPatch ftp://ftp.gnu.org/pub/gnu/gcc/$gcc.tar.gz ;;
   #gcc-3.3.3)
   #   getUnpackAndPatch ftp://ftp.gnu.org/pub/gnu/gcc/releases/$gcc/$gcc.tar.bz2 ;;
   gcc-3.3.5)
      getUnpackAndPatch ftp://gcc.gnu.org/pub/gcc/releases/$gcc/$gcc.tar.bz2 ;;
   gcc-4.[01234].[012345]-200*)
      dir=`echo $gcc | sed s/gcc-/prerelease-/`
      getUnpackAndPatch	ftp://gcc.gnu.org/pub/gcc/$dir/$gcc.tar.bz2 \
			ftp://gcc.gnu.org/pub/gcc/$dir/$gcc.tar.gz ;;
   gcc-4.[01234]-200*)
      snapshotdir=`echo $gcc | sed s/gcc-//`
      getUnpackAndPatch	ftp://gcc.gnu.org/pub/gcc/$dir/$gcc.tar.bz2 \
			ftp://gcc.gnu.org/pub/gcc/$dir/$gcc.tar.gz \
			ftp://gcc.gnu.org/pub/gcc/snapshots/$snapshotdir/$gcc.tar.bz2 ;;
   gcc-3.[3456]-200*|gcc-4.0-200*|gcc-4.1-200*)
      dir=`echo $gcc | sed 's/gcc-//'`
      getUnpackAndPatch ftp://gcc.gnu.org/pub/gcc/snapshots/$dir/$gcc.tar.bz2 \
			ftp://gcc.gnu.org/pub/gcc/snapshots/$dir/$gcc.tar.gz ;;
   *)
      getUnpackAndPatch ftp://ftp.gnu.org/pub/gnu/gcc/$gcc/$gcc.tar.bz2 \
			ftp://ftp.gnu.org/pub/gnu/gcc/$gcc/$gcc.tar.gz \
                        ftp://gcc.gnu.org/pub/gcc/releases/$gcc/$gcc.tar.bz2 \
			ftp://gcc.gnu.org/pub/gcc/releases/$gcc/$gcc.tar.gz ;;
 esac
done

# Linux and glibc addons not needed if building a cygwin target.
if test "${CYGWIN_DIR}" = ""; then
  case $LINUX_DIR in
    *2.4*) getUnpackAndPatch http://www.kernel.org/pub/linux/kernel/v2.4/$LINUX_DIR.tar.bz2 http://www.kernel.org/pub/linux/kernel/v2.4/$LINUX_DIR.tar.gz ;;
    *2.6*) getUnpackAndPatch http://www.kernel.org/pub/linux/kernel/v2.6/$LINUX_DIR.tar.bz2 http://www.kernel.org/pub/linux/kernel/v2.6/$LINUX_DIR.tar.gz ;;
    "") ;;
    *) abort "unknown version $LINUX_DIR of linux, expected 2.4 or 2.6 in name?" ;;
  esac
  # Fetch linux-libc-headers, if requested
  test -n "${LINUX_SANITIZED_HEADER_DIR}" && getUnpackAndPatch \
	http://ep09.pld-linux.org/~mmazur/linux-libc-headers/${LINUX_SANITIZED_HEADER_DIR}.tar.bz2 \
	ftp://ftp.lfs-matrix.net/pub/linux-libc-headers/${LINUX_SANITIZED_HEADER_DIR}.tar.bz2
  # Glibc addons must come after glibc
  getUnpackAndPatch     \
       ftp://ftp.gnu.org/pub/gnu/glibc/$GLIBCTHREADS_FILENAME.tar.bz2 \
       ftp://ftp.gnu.org/pub/gnu/glibc/$GLIBCTHREADS_FILENAME.tar.gz \
       ftp://gcc.gnu.org/pub/glibc/releases/$GLIBCTHREADS_FILENAME.tar.bz2 \
       ftp://gcc.gnu.org/pub/glibc/releases/$GLIBCTHREADS_FILENAME.tar.gz 

  test x$GLIBCCRYPT_FILENAME = x || getUnpackAndPatch     ftp://ftp.gnu.org/pub/gnu/glibc/$GLIBCCRYPT_FILENAME.tar.gz ftp://ftp.gnu.org/pub/gnu/glibc/$GLIBCCRYPT_FILENAME.tar.bz2
fi

if test -z "$JUST_DOWNLOAD"; then
  # gcc's contrib/test_summary expects version stamp, normally created by contrib/update_gcc
  test -f $SRC_DIR/$GCC_DIR/LAST_UPDATED || echo $GCC_DIR > $SRC_DIR/$GCC_DIR/LAST_UPDATED

  # binutils-2.14.90.0.3 and up want you to apply a patch
  if grep -q "/bin/sh patches/README" $SRC_DIR/$BINUTILS_DIR/patches/README; then
    if test '!' -f $SRC_DIR/$BINUTILS_DIR/patches/README.done; then
      cd $SRC_DIR/$BINUTILS_DIR
      /bin/sh patches/README
      touch patches/README.done
    fi
  fi
fi
