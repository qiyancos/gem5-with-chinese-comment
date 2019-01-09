#!/bin/sh
# Demo of how to set up a canadian cross, where there are three
# architectures involved:
# the BUILD machine, which builds a compiler that targets...
# the HOST machine, which builds a compiler that targets...
# the TARGET machine, which runs the code compiled on the HOST machine.
#
# In this example, all machines are assumed to mount /opt/distcrosstool,
# and the compiler to run on $BUILD for $TARGET is in
# /opt/distcrosstool/$BUILD/$TARGET/$TOOLCOMBO.
#

echo "not finished"
exit 1

set -ex

TARBALLS_DIR=$HOME/downloads
TIPPY_TOP=/opt/distcrosstool

if test ! -w $TIPPY_TOP; then
    echo "Cannot write to $TIPPY_TOP.  This makes it hard to install stuff there :-)"
    exit 1
fi
# Clear a few variables known to cause problems if set randomly
unset PREFIX

BUILD=`./config.guess`
export BUILD
if test x$BUILD = x; then echo "config.guess broken?"; exit 1; fi
RESULT_TOP=$TIPPY_TOP/$BUILD
export RESULT_TOP TARBALLS_DIR
GCC_LANGUAGES="c,c++"
export GCC_LANGUAGES

umask 022

mkdir -p $RESULT_TOP

TARGETS="powerpc-405:gcc-3.4.0-glibc-2.3.2"
HOSTS="x86_64:gcc-3.4.0-glibc-2.3.2"

set +x
# First, build crosscompilers targeting all hosts (so we can build the final compilers)
# and all the targets (so we can build the final libraries)
for target in `echo $TARGETS $HOSTS | sort -u`; do
   cpu=`echo $target | sed 's/:.*$//'`
   toolcombo=`echo $target | sed 's/^.*://'`
   gnu_target=`awk -F= '/^TARGET=/ {print $2}' $cpu.dat`
   echo cpu $cpu toolcombo $toolcombo gnu_target $gnu_target
   # Check to see if compiler already built
   if test -x $TIPPY_TOP/$BUILD/$gnu_target/bin/$gnu_target-gcc ; then
      echo "Compiler already present, not rebuilding"
   else
      #eval `cat $cpu.dat $toolcombo.dat` sh all.sh --notest
      :
   fi
done
exit 0

# Finally, build canadian crosscompilers to run on the hosts, targeting the targets.
for host in $HOSTS; do
   cpu=`echo $target | sed 's/:.*$//'`
   toolcombo=`echo $target | sed 's/^.*://'`
   HOST=`awk -F= '/^TARGET=/ {print $2}' $cpu.dat`
   export HOST

   # Check to see if compiler already built
   if test -x $TIPPY_TOP/$BUILD/$gnu_target/bin/$gnu_target-gcc ; then
      echo "Compiler already present, not rebuilding"
   else
      #eval `cat $cpu.dat $toolcombo.dat` sh all.sh --notest
      :
   fi
done

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
# and refer to the compiler wrappers in $RESULT_TOP/$TARGET/$TOOLCOMBO/distributed/bin
# instead of the real compilers in $RESULT_TOP/$TARGET/$TOOLCOMBO/bin.
# FIXME:
# It's hard for developers to grok how to use this in their build scripts.
# We should install a copy of config.guess in $TIPPY_TOP/common/bin
# and create a shell script $TIPPY_TOP/common/bin/$TARGET-$TOOLCOMBO-env.sh 
# that sets environment variables, e.g.
#  TARGET=(whatever)
#  TOOLCOMBO=(whatever)
#  TIPPY_TOP=/opt/distcrosstool
#  BUILD=`$TIPPY_TOP/common/bin/config.guess`
#  CROSS=$TIPPY_TOP/$BUILD/$TARGET/$TOOLCOMBO/distributed/bin
#  CC=$CROSS-gcc
#  CXX=$CROSS-g++
#  export CROSS CC CXX
# so developers can just source that file interactively or in their build scripts...

echo Done.
