#!/bin/sh
# Script to retrieve glibc source via CVS as of a certain date
# Usage: grab.sh YYYYMMDD
# Creates glibc-YYYYMMDD.tar.gz and glibc-linuxthreads-YYYYMMDD.tar.gz
if test "$1" = ""; then
  echo Usage: grab.sh YYYYMMDD
  echo or: grab.sh glibc-YYYYMMDD.tar.gz
  exit 1
fi

set -ex

mkdir -p temp.$$
cd temp.$$

case $1 in
	*glibc-200*z)
		file=`echo $1 | sed 's/.*glibc-\([0-9]*\).tar.gz/\1/'`
		;;
	*)
		file=$1
		;;
esac

cvs -d :pserver:anoncvs@sources.redhat.com:/cvs/glibc co -D $file -d glibc-$file libc

tar --exclude CVS --exclude linuxthreads* -czf ../glibc-$file.tar.gz glibc-$file
cd glibc-$file
tar -czf ../../glibc-linuxthreads-$file.tar.gz linuxthreads* 

cd ..
cd ..
rm -rf temp.$$
