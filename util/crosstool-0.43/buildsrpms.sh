#!/bin/sh
# Given a crosstool source tarball, 
# create specfiles suitable for building crosstool RPMs with rpmbuild,
# then build SRPMS containing the specfiles and all neccessary
# source tarballs.
# Usage:
# tar -xzvf crosstool-0.43.tar.gz
# sh crosstool-0.43/buildsrpms.sh

abort() {
    echo $@
    exec false
}

set -ex

# User must have created ~/.rpmmacros containing at least a line like
# %_topdir      %(echo $HOME)/rpmbuild
# This script assumes that rpmbuild is in the current directory, and can be wiped out.

#FIXME: CROSSTOOLVERSION needs to be updated every time crosstool's version changes, ewww
CROSSTOOLVERSION=0.43
PKGPREFIX=${PKGPREFIX:-"crosstool"}

# Edit this line to specify which toolchain combos to build specfiles for
# Or override the environment variable (see rerpm.sh for example)
TOOLCOMBOS=${TOOLCOMBOS-"\
gcc-2.95.3-glibc-2.1.3 \
gcc-2.95.3-glibc-2.2.5 \
gcc-3.3.6-glibc-2.2.5 \
gcc-3.3.6-glibc-2.3.2 \
gcc-3.3.6-glibc-2.3.5 \
gcc-3.4.5-glibc-2.2.5 \
gcc-3.4.5-glibc-2.3.2 \
gcc-3.4.5-glibc-2.3.5 \
gcc-4.0.2-glibc-2.3.2 \
gcc-4.0.2-glibc-2.3.5 \
"}

# I prefer /opt/crosstool, but rpmlint objects less to /usr/crosstool
RESULT_TOP=${RESULT_TOP-/usr/crosstool}
test -f crosstool.sh && abort "Don't run this inside the crosstool directory!"
test -f crosstool-$CROSSTOOLVERSION.tar.gz || abort "Can't find crosstool-$CROSSTOOLVERSION.tar.gz"
test -f crosstool-$CROSSTOOLVERSION/crosstool.sh || abort "Can't find crosstool-$CROSSTOOLVERSION/crosstool.sh; please unpack crosstool-$CROSSTOOLVERSION.tar.gz"

rm -rf rpmbuild
mkdir -p rpmbuild/{BUILD,RPMS,SOURCES,SPECS,SRPMS,tmp}

cp crosstool-$CROSSTOOLVERSION.tar.gz rpmbuild/SOURCES/
rm -rf crosstool-$CROSSTOOLVERSION/specs
mkdir crosstool-$CROSSTOOLVERSION/specs


# Only generate CPU choices that are known to build a toolchain.
# Requires regtest-run.sh to have run once.
for TOOLCOMBO in $TOOLCOMBOS; do
  # Download source tarballs into an empty directory so we can tell
  # which tarballs this toolcombo needs (yeah, this is a kludge)
  TARBALLS_DIR=build/tarballs-$TOOLCOMBO
  export TARBALLS_DIR
  rm -rf $TARBALLS_DIR
  mkdir -p $TARBALLS_DIR
  export TOOLCOMBO
  JUST_DOWNLOAD=1
  export JUST_DOWNLOAD
  eval `cat crosstool-$CROSSTOOLVERSION/$TOOLCOMBO.dat` sh crosstool-$CROSSTOOLVERSION/getandpatch.sh

  # Turn the list of tarballs into the form needed by the spec file
  # Do in two steps to avoid quoting hell
  # The backslash on the ends of lines in sources.txt is required by the c command in sed (see below)
  ls build/tarballs-$TOOLCOMBO | sed 's,.*/,,' | awk '{print "Source" NR ":", $0 "\\"}' > sources.txt
  SOURCES=`cat sources.txt`

  # Generate the spec file
  # rpm 4.0 doesn't work with nested conditionals, so expand __if %have_sharedlibs ... __endif here.
  case $TOOLCOMBO in
  *gcc-2*) HAVE_SHAREDLIBS="/__IF %have_sharedlibs/,/__ENDIF/d" ;;
  *) HAVE_SHAREDLIBS="/__IF %have_sharedlibs/d;/__ENDIF/d" ;;
  esac

  # rpm does not allow dashes in variable names, so use underscores (ouch, ouch),
  # and undo the underscores once we're inside the spec file
  # Betcha never seen the c command in sed before!  
  # I got this from http://www-106.ibm.com/developerworks/linux/library/l-sed2.html
  # /__SOURCES__/c etc. replaces the line __SOURCES__ with the multiple lines in $SOURCES
  WORKING_CPUS=`grep -l toolchain=PASS crosstool-$CROSSTOOLVERSION/buildlogs/*-$TOOLCOMBO*dat.txt | sed "s/-$TOOLCOMBO.*//;s/-/_/g;s/.*buildlogs.//" | sort -u | tr '\012' ' '`
  echo WORKING_CPUS is $WORKING_CPUS
  WANT_CPUS=${WANT_CPUS-$WORKING_CPUS}
  # Only include CPUs that both work and, if the user specified a list, are in the user's list
  # Must translate dashes to underscores, since you can't form an rpm variable with a - in the name
  # (anyway, dashes in WORKING_CPUS are already translated, and we have to match)
  CPUS=`echo $WANT_CPUS $WORKING_CPUS | sed 's/-/_/g' | tr ' ' '\012' | sort | uniq -c | awk '$1 == 2 {print $2}' | tr '\012' ' '`
  echo generating specfile for following CPUs: $CPUS
  if test x"$CPUS" = x""; then
    abort "No CPUs supported for this toolcombo?"
  fi
  cat crosstool-$CROSSTOOLVERSION/crosstool.spec.in |
   sed "s,__PKGPREFIX__,$PKGPREFIX,g;s,__TOOLCOMBO__,$TOOLCOMBO,g;s,__RESULT_TOP__,$RESULT_TOP,g;s,__CPUS__,$CPUS,g;s,__CROSSTOOLVERSION__,$CROSSTOOLVERSION,g"  |
   sed "$HAVE_SHAREDLIBS" |
   sed "/__SOURCES__/c\\
$SOURCES
" | 
    perl crosstool-$CROSSTOOLVERSION/expandForLoop.pl crosstool-$CROSSTOOLVERSION > rpmbuild/SPECS/crosstool-$TOOLCOMBO.spec

  # Save the specfile in the crosstool directory for posterity
  # (not directly useful, since it's hard to build without the tarballs,
  # but the curious may find them interesting)
  cp rpmbuild/SPECS/crosstool-$TOOLCOMBO.spec crosstool-$CROSSTOOLVERSION/specs

  # Build the .srpm 
  # Specify --without all so none of the subpackages are built
  cp build/tarballs-$TOOLCOMBO/* rpmbuild/SOURCES
  #rpmbuild -ba rpmbuild/SPECS/crosstool-$TOOLCOMBO.spec --without all
  rpmbuild -ba rpmbuild/SPECS/crosstool-$TOOLCOMBO.spec --define "_without_all --without-all"
done

echo SRPMS are in rpmbuild/SRPMS:
ls -l rpmbuild/SRPMS

