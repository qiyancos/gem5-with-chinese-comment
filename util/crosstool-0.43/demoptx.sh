#!/bin/sh

RESULT_TOP=/opt/crosstool
export RESULT_TOP
TARBALLS_DIR=${HOME}/downloads
export TARBALLS_DIR

#TARGET=powerpc-405-linux-gnu
TARGET=powerpc-750-linux-gnu
#TARGET=sh4-unknown-linux-gnu

#GCC_DIR=gcc-3.3.1
GCC_DIR=gcc-2.95.3

#GLIBC_DIR=glibc-2.3.2
GLIBC_DIR=glibc-2.2.2
#GLIBC_DIR=glibc-2.2.5

export TARGET GCC_DIR GLIBC_DIR
sh ptx.sh
