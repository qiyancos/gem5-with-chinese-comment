#!/bin/sh
set -x
TARBALLS_DIR=$HOME/downloads
export TARBALLS_DIR RESULT_TOP

# Please read doc/crosstool-howto.html
# Here's a demo for the impatient, showing all the configurations I've tested.
# Total disk requirement: about 1.5GB per toolchain.
# It can build various toolchains for twelve processors: 
# alpha, arm, cris, i686, ia64, x86_64, m68k, mips, powerpc750, powerpc405, sh4, and sparc.
# It can almost, but not quite, build toolchains for two processors: hppa and s390.
#
# Uncomment the lines for the CPUs you want to target, and comment out the others.
# Please read the demo-$CPU.sh file for your CPUs before running!
#
# Once that works, please try running the regression test 
# by setting up a chroot environment as described in doc/crosstool-howto.html.
# and removing the --notest arg.  This will place regression test reports in
# $TARGET-$TOOLCOMBON-*.sum.  Please post these logs to the crossgcc mailing
# list.
#
# Details:
# Each of the demo-$CPU.sh files sets the destination of the build to /opt/crosstool,
# and the place to save tarballs to ~/tarballs,
# then has a line for each combination of gcc and glibc I've tried to build for that CPU.
# Generally, all but the most up-to-date working combination is commented out.
# The demo scripts use eval so it can store some of the environment variables in a file.
# If you don't like eval, you can set the environment variables some other way.
# Options to all.sh:
# --nounpack avoids unpacking the source tarballs and re-configuring; useful for quick redos.
# --nobuild  avoids building; useful if you just want to unpack sources or rerun regression tests.
# --notest   avoids running regression tests; they're hard to set up, so this is recommended when starting.

# Set this if you have a dual CPU build machine, or are using distcc
#PARALLELMFLAGS="-j2"
#export PARALLELMFLAGS

# Alpha
sh demo-alpha.sh > demo-alpha.log 2>&1

# Arm 
sh demo-arm.sh > demo-arm.log 2>&1

# Arm with softfloat
sh demo-arm-softfloat.sh > demo-arm-softfloat.log 2>&1

# Arm9tdmi
sh demo-arm9tdmi.sh > demo-arm.log 2>&1

# i686 / Pentium
sh demo-i686.sh > demo-i686.log 2>&1

# ia64 / Itanic
sh demo-ia64.sh > demo-ia64.log 2>&1

# m68k (680x0)
sh demo-m68k.sh > demo-m68k.log 2>&1

# Mips
sh demo-mipsel.sh > demo-mipsel.log 2>&1

# PPC
# note: must call target powerpc rather than ppc if you want to run testcases

# PPC 405
sh demo-ppc405.sh > demo-ppc405.log 2>&1

# PPC 750
sh demo-ppc750.sh > demo-ppc750.log 2>&1

# PPC 7450
sh demo-ppc7450.sh > demo-ppc7450.log 2>&1

# PPC 860
sh demo-ppc860.sh > demo-ppc860.log 2>&1

# PPC 970 (powerpc64)
sh demo-ppc970.sh > demo-ppc970.log 2>&1

# SH-3
sh demo-sh3.sh > demo-sh3.log 2>&1

# SH-4
sh demo-sh4.sh > demo-sh4.log 2>&1

# s390
sh demo-s390.sh > demo-s390.log 2>&1

# Sparc
sh demo-sparc.sh > demo-sparc.log 2>&1

# Sparc64
sh demo-sparc64.sh > demo-sparc64.log 2>&1

# x86_64 / AMD64 / Opteron / Hammer
sh demo-x86_64.sh > demo-x86_64.log 2>&1

#--------- semi-broken arches below -----
# Cris only builds with older glibc, but it does build.
sh demo-cris.sh > demo-cris.log 2>&1

#--------- broken arches below ----------

# Following architectures may not be supported by glibc-2.3.2.  Try getting glibc-2.3.3 from cvs!
# (No, getandpatch.sh doesn't know how to do that yet, but it wouldn't be too hard
# to add.)

# HP-PA / parisc
#eval `cat hppa.dat gcc-3.3-glibc-2.3.2.dat`  sh all.sh --notest
# Fails with error "errno-loc.c:39: error: `pthread_descr' undeclared" when building glibc.
#eval `cat hppa.dat gcc-3.3-20040112-glibc-2.3.2.dat` sh all.sh --notest

