#!/bin/sh
# Script to demonstrate building srpms and rpms from scratch
# Useful for testing changes to crosstool.spec.in
# Uses old syntax for --with and --without so they can be used on red hat 7.1
set -x

TOOLCOMBOS="\
gcc-2.95.3-glibc-2.2.2 \
"
#gcc-3.4.3-glibc-2.3.3 \
#gcc-3.4.3-glibc-2.2.2 \
export TOOLCOMBOS

WANT_CPUS="\
i686 \
"
#x86_64 \
export WANT_CPUS

set -ex
tar -czvf crosstool-0.43.tar.gz crosstool-0.43

sh crosstool-0.43/buildsrpms.sh 

for TOOLCOMBO in $TOOLCOMBOS; do
  cp rpmbuild/SRPMS/crosstool-$TOOLCOMBO-0.43-1.src.rpm .
  rpm -i crosstool-$TOOLCOMBO-0.43-1.src.rpm 

  rpmbuild -bb rpmbuild/SPECS/crosstool-$TOOLCOMBO.spec
done

# Oh, yeah: crosstool-common.  The runt.  Only there so
# users have a config.guess to locate a toolchain that will
# run on the current system.  (Hmm, maybe that should be
# set at install time via symlinks in /etc/crosstool/...
# sort of like Debian's alternatives.)
