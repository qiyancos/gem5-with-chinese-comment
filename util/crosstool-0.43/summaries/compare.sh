#!/bin/sh
# Trivial example of how to get a list of regressions between gcc-3.3.1 and a gcc-3.4 snapshot
set -x
for cpu in powerpc-405 powerpc-750 sh4-unknown; do
   for a in gcc glibc; do
      diff -u $cpu-linux-gnu-gcc-3.3.1-glibc-2.3.2.$a.sum  $cpu-linux-gnu-gcc-3.4-20030813-glibc-2.3.2.$a.sum 
   done
done
