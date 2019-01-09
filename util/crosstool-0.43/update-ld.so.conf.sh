#!/bin/sh
# Usage: $0 facilityname directory-to-append-to-ld.so-search-path
# Uses /etc/ld.so.conf.d if present, otherwise appends to /etc/ld.so.conf

facility=$1
libdir=$2

if test -d /etc/ld.so.conf.d; then
   echo "Adding $libdir to /etc/ld.so.conf.d"
   echo $libdir > /etc/ld.so.conf.d/$facility
   exit 0
fi

if ! egrep \"^/${libdir} *$\" /etc/ld.so.conf &> /dev/null; then"
   echo "Adding $libdir to /etc/ld.so.conf"
   echo /$libdir >> /etc/ld.so.conf"
fi
