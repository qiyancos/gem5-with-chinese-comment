#!/bin/sh
#
# Copyright 2004 Google Inc.
# This script is provided under the terms of the GPL.
#
# Adds (or, if --uninstall given, removes) a directory from /etc/ld.so.conf, then runs ldconfig.

abort() {
  echo $@
  exec false
}

#set -x

FILE=/etc/ld.so.conf
BAK=$FILE.bak
NEW=$FILE.new
DEBUG=1

while [ $# -gt 1 ]; do
  case "$1" in
    --uninstall|-uninstall)
      opt_uninstall=1
      ;;
    *)
      abort "Usage: $0 [--uninstall] directory"
      ;; 
  esac
  shift
done

cp $FILE $BAK

NEWDIR=$1

if test "$opt_uninstall" = "1"; then
  awk "\$0 != \"$NEWDIR\" {print}" $FILE > $NEW
else
  awk "BEGIN {print \"$NEWDIR\"}; \$0 != \"$NEWDIR\" {print}" $FILE > $NEW
fi
mv $NEW $FILE

ldconfig
