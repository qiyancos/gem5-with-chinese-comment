#!/bin/sh
# Script to complete jail initialization.  Run on target.

if test $# -ne 1; then
    echo Usage: sh initjail.sh jaildir
    echo Removes and recreates given directory, then unpacks jail.tar.gz inside it.
    echo As a safety measure, jaildir must contain the string 'jail' somewhere in it.
    exit 1
fi

set -x -e

case $1 in
*jail*) echo "jaildir is $1";;
*) echo "jaildir must contain the word 'jail' in it somewhere"; exit 1;;
esac

mkdir -p $1
JAIL=`cd $1; pwd`
ORIGDIR=`pwd`

if test "$JAIL" = ""; then echo "bug"; exit 1; fi

# Undo any earlier mount
umount  $JAIL/proc 2> /dev/null || true

# Unpack compressed tarball from stdin
rm -rf $JAIL/*
cd $JAIL
tar -xvf -

cd dev
mknod null   c 1 3
mknod zero   c 1 5
mknod random c 1 8
chmod 666 *
cd ..

if test -x /bin/busybox ; then
	PROGS="rcp busybox"
else
	PROGS="true ln ls rm mv cp su rcp"
	# ash has fewer dependencies -- no ncurses, etc
	if test -x /bin/ash ; then
		PROGS="$PROGS ash"
	else
		PROGS="$PROGS sh"
	fi
fi

for prog in $PROGS; do
	for dir in /bin /usr/bin /sbin /usr/sbin; do
		if test -x $dir/$prog && test ! -x $JAIL/$dir/$prog; then cp $dir/$prog $JAIL/$dir/$prog; fi
	done
done
test -f $JAIL/bin/ash && ln -s ash $JAIL/bin/sh

sed 's,/sbin/chrootshell,/bin/sh,;s,/jail,/,' < /etc/passwd  >  $JAIL/etc/passwd

test -f /etc/shadow && cp /etc/shadow       $JAIL/etc
cp /etc/group        $JAIL/etc
cp /etc/services     $JAIL/etc
if [ -d /etc/pam.d ]; then
	cp -r /etc/pam.d      $JAIL/etc
	cp -r /lib/libpam*    $JAIL/lib
	cp -r /lib/security   $JAIL/lib/security
fi
cp -d /usr/lib/libz.* $JAIL/usr/lib
mount -t proc proc  $JAIL/proc

if [ -x /bin/busybox ]; then
	/usr/sbin/chroot $JAIL /bin/busybox --install -s
fi

