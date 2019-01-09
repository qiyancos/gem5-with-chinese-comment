#!/bin/sh
# Simple demo of how to use buildsrpms to build RPMs for just a few combinations
# of toolchains and targets
#
# User must have created ~/.rpmmacros containing at least a line like
# %_topdir      %(echo $HOME)/rpmbuild
# This script assumes that rpmbuild is in the current directory, and can be wiped out.
#
# You may want to install fedora-rpmdevtools
# (http://www.fedoraproject.org/wiki/fedora_2drpmdevtools)
# and use fedora-buildrpmtree to set up your ~/.rpmrc and rpmbuild directory

set -ex
#tar -xzvf crosstool-0.43.tar.gz
TOOLCOMBOS="gcc-3.4.3-glibc-2.2.2" 
export TOOLCOMBOS
sh crosstool-0.43/buildsrpms.sh 
mkdir -p result
for toolcombo in $TOOLCOMBOS; do
   cp rpmbuild/SRPMS/crosstool-$toolcombo*.src.rpm result
   rpm -i result/crosstool-$toolcombo*.src.rpm
   rpmbuild -bb  rpmbuild/SPECS/crosstool-$toolcombo*.spec --without all --with i686 
   cp rpmbuild/RPMS/*/crosstool-$toolcombo*.rpm result
done
