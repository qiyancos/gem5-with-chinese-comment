#!/bin/sh
# Demo of how to set up a a hetrogenous compile cluster
# where all the nodes simply mount a common directory to get
# the compilers.
# In this example, all machines are assumed to mount /opt/distcrosstool,
# and the compiler to run on $GNU_BUILD for $GNU_TARGET is in
# /opt/distcrosstool/$GNU_BUILD/$GNU_TARGET/$TOOLCOMBO.
# The real compilers are in that directory's bin subdir;
# the distcc client wrappers for those compilers are in that directory's
# distributed/bin subdir.
#
# Run this script once on each *kind* of computer in the cluster.
# I have tested this with a cluster containing one machine of
# each of the following types:
#  Mac OS X, Linux x86, Linux x86_64, and Cygwin
# Each server can handle compile requests coming from any of
# the machines, even though the incoming request is usually to
# run a compiler compiled for some other build machine type;
# it simply maps the incoming request to a compiler it knows how to run.

set -ex

TARBALLS_DIR=$HOME/downloads
TIPPY_TOP=/opt/crosstool

if test ! -w $TIPPY_TOP; then
    echo "Cannot write to $TIPPY_TOP.  This makes it hard to install stuff there :-)"
    exit 1
fi
# Clear a few variables known to cause problems if set randomly
unset PREFIX

GNU_BUILD=`./config.guess`
if test x$GNU_BUILD = x; then echo "config.guess broken?"; exit 1; fi
RESULT_TOP=$TIPPY_TOP/$GNU_BUILD
export RESULT_TOP TARBALLS_DIR
GCC_LANGUAGES="c,c++"
export GCC_LANGUAGES

umask 022

mkdir -p $RESULT_TOP

# Build all the compilers you want your compile cluster to support.
#eval `cat i686.dat gcc-2.95.3-glibc-2.1.3.dat`   sh all.sh --notest
#eval `cat i686.dat gcc-3.3.3-glibc-2.1.3.dat`    sh all.sh --notest 
 eval `cat i686.dat gcc-3.4.0-glibc-2.1.3.dat`    sh all.sh --notest 
#eval `cat x86_64.dat gcc-3.3.3-glibc-2.3.2.dat`  sh all.sh --notest
 eval `cat x86_64.dat gcc-3.4.0-glibc-2.3.2.dat`  sh all.sh --notest

# Build distcc.
sh mkdistcc.sh

# Install distcc links for all the compilers.
(cd $RESULT_TOP; sh common/bin/mkdistcclinks.sh)

# Fix permissions in case the umask change above didn't do the trick
# This just propagates the 'r' and 'x' bits from user to group and world
find $NOARCH_TOP -perm -400 -not -perm -044 | xargs chmod ag+r
find $NOARCH_TOP -perm -100 -not -perm -011 | xargs chmod ag+x

# Now, for each machine you want to be a distccd server, you need to install 
# distccd as a service.  A script to do this for your system
# was installed by mkdistcclinks.sh; you can run it with
#   sh $RESULT_TOP/common/bin/install-distccd.sh
# Then start the service.

# Then, for each machine you want to be a distcc client,
# you'll need to create a list of the server machines, put
# it in the environment variable DISTCC_HOSTS or the file
# $RESULT_TOP/common/etc/distcc/hosts before compiling,
# and refer to the compiler wrappers in $RESULT_TOP/$GNU_TARGET/$TOOLCOMBO/distributed/bin
# instead of the real compilers in $RESULT_TOP/$GNU_TARGET/$TOOLCOMBO/bin.
# FIXME:
# It's hard for developers to grok how to use this in their build scripts.
# We should install a copy of config.guess in $TIPPY_TOP/common/bin
# and create a shell script $TIPPY_TOP/common/bin/$GNU_TARGET-$TOOLCOMBO-env.sh 
# that sets environment variables, e.g.
#  GNU_TARGET=(whatever)
#  TOOLCOMBO=(whatever)
#  TIPPY_TOP=/opt/distcrosstool
#  GNU_BUILD=`$TIPPY_TOP/common/bin/config.guess`
#  CROSS=$TIPPY_TOP/$GNU_BUILD/$GNU_TARGET/$TOOLCOMBO/distributed/bin
#  CC=$CROSS-gcc
#  CXX=$CROSS-g++
#  export CROSS CC CXX
# so developers can just source that file interactively or in their build scripts...

echo Done.
