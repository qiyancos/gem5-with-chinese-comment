#!/bin/sh
# Make distcc masquerade directory for this toolchain
# Assumes that this toolchain is installed at e.g.
#  /opt/crosstool/`config.guess`/$TARGET
# and that crosstool's patched distcc will be installed later
# in a fixed path relative to this toolchain, e.g.
#  /opt/crosstool/`config.guess`/common/bin/distcc

# On entry, `pwd` is top of this toolchain, e.g.
#  /opt/crosstool/`config.guess`/$TARGET
# and, if the compilers are to run on Cygwin, EXEEXT must be set to .exe

abort() {
    echo $@
    exec false
}
set -e

# Absolute path to directory containing compilers
# Usually /opt/crosstool/`config.guess`/$TARGET/bin
ABSBIN="`pwd`/bin"

# Absolute path to directory containing all toolchains for this build system
# Usually /opt/crosstool/`config.guess`
ABSTOP=`cd ..; pwd`

# Which distcc to use
DISTCC=distcc

# Get list of executables to run via distcc
# This is a bit messy, since executables in Cygwin usually end in .exe, yet
# gcc's Makefile installs the versioned gcc executable without .exe  
DISTRIB_APPS=`cd bin; find . -type f -perm -100 -name "*-c++$EXEEXT" -o -name "*-g++$EXEEXT" -o -name "*-gcc$EXEEXT" -o -name "*-gcc-[0-9]*[0-9]"  -o -name "*-gcc-[0-9]*[0-9]$EXEEXT"`

rm -rf distributed
mkdir distributed
cd distributed

# Make symlinks to all subdirs of real directory
ln -s ../* .
# but remove symlinks to special cases
rm bin distributed
mkdir bin

# Make symlinks to all apps in real bin
cd bin
for app in `cd ../../bin; ls`; do
    ln -s ../../bin/$app .
done
# but remove symlinks to special cases
rm $DISTRIB_APPS

# and create shell scripts for the special cases
for app in $DISTRIB_APPS; do
  if test x$EXEEXT != x; then
    # shell scripts must not end in $EXEEXT in Windows
    # else they are misinterpreted in an amusing and fatal way
    app=`echo $app | sed "s/$EXEEXT//"`
  fi
  # canonicalize path by getting rid of extra ./ by find .
  app=`echo $app | sed "s,^\./,,g"`
  cat > $app <<_EOF_
#!/bin/sh
$DISTCC $ABSBIN/$app "\$@"
_EOF_
  chmod 755 $app 
done
