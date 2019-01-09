#!/bin/sh
set -x
cd /tmp

ps augxw | grep $USER | egrep 'jobdir|gcc.*glibc.*running|crosstool.sh|make' | grep -v grep | awk '{print $2}' > procs
if test -s procs; then
	kill `cat procs`
	sleep 1
	ps augxw | grep $USER | egrep 'jobdir|gcc.*glibc.*running|crosstool.sh|make' | grep -v grep | awk '{print $2}' > procs
	if test -s procs; then
		kill -9 `cat procs`
	fi
fi
rm -f procs
