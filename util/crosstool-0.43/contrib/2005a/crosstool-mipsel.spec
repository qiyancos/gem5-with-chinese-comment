%define version 0.25
%define gccvers 3.3.2
%define glibcvers 2.3.2
%define binutilsvers 2.14.90.0.5
%define kernelversion 2.4.21
%define lrelease cb1
%define release gcc_%{gccvers}_glibc_%{glibcvers}_%{lrelease}

Summary: GNU Compiler Collection (GCC) for Linux/MIPSEL cross-development
Name: crosstool-gcc
Version: %{version}
Release: %{release}
License: GPL
Group: Development/Languages
%define base_url http://www.kegel.com/crosstool/
URL: %{base_url}
Source0: %{base_url}/crosstool-%{version}.tar.gz
Source1: gcc-%{gccvers}.tar.gz
Source2: glibc-%{glibcvers}.tar.bz2
Source3: glibc-linuxthreads-%{glibcvers}.tar.bz2
Source4: binutils-%{binutilsvers}.tar.bz2
Source5: linux-%{kernelversion}.tar.bz2
BuildRequires: /bin/pwd, /bin/sh, autoconf, binutils >= 2.9.4, bison
BuildRequires: bzip2, diffutils, fileutils, findutils, flex
BuildRequires: glibc-devel >= 2.1, gzip, m4 >= 1.1, make >= 3.77, patch >= 2.5
BuildRequires: rpm >= 3.0, sed, tar, textutils
#BuildRoot: /var/tmp/%{name}-%{version}-root

%description
A GNU cross-compiler for the little-ended MIPS chipset, built using Dan Kegel's
crosstool script set.

%changelog

%prep
%setup -q -n crosstool-%{version}

%build
export TARBALLS_DIR=$RPM_SOURCE_DIR
echo $TARBALLS_DIR
export RESULT_TOP=/opt/crosstool
if [ -e $RESULT_TOP/mipsel-unknown-linux-gnu/gcc-%{gccvers}-glibc-%{glibcvers}/ ]
then
   echo "target directory $RESULT_TOP/mipsel-unknown-linux-gnu/gcc-%{gccvers}-glibc-%{glibcvers}/ exists"
   echo "check what you are doing, and remove the target directory if appropriate"
   exit 1
else
   mkdir -p $RESULT_TOP/mipsel-unknown-linux-gnu/gcc-%{gccvers}-glibc-%{glibcvers}/
   if [ \! -d $RESULT_TOP/mipsel-unknown-linux-gnu/gcc-%{gccvers}-glibc-%{glibcvers} ]
   then
      echo "could not create target directory"
      exit 2
   fi
fi

KERNELCONFIG=$(pwd)/mipsel.config \
  TARGET=mipsel-unknown-linux-gnu \
  TARGET_CFLAGS="-O2 -finline-limit=10000" \
  BINUTILS_DIR=binutils-%{binutilsvers} \
  BINUTILS_URL=http://www.kernel.org/pub/linux/devel/binutils \
  GCC_DIR=gcc-%{gccvers} \
  GLIBC_DIR=glibc-%{glibcvers} \
  LINUX_DIR=linux-%{kernelversion} \
  GLIBCTHREADS_FILENAME=glibc-linuxthreads-%{glibcvers} \
  sh all.sh --notest

%install
# install already done during build

%files
%defattr(-,root,root)
%dir /opt/crosstool/mipsel-unknown-linux-gnu/gcc-%{gccvers}-glibc-%{glibcvers}/
/opt/crosstool/mipsel-unknown-linux-gnu/gcc-%{gccvers}-glibc-%{glibcvers}/*
