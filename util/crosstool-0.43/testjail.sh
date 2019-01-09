#!/bin/sh
# Demo of how to rebuild a jail remotely, then test it
# Example use:
#  TARGET=sh4-unknown-linux-gnu PREFIX=result/gcc-3.3-glibc-2.2.5/sh4-unknown-linux-gnu sh testjail.sh

set -x -e

# Assume there's an entry in /etc/hosts for the target to run on
REMOTE=$TARGET

# Assume jail already has a good etc/passwd containing an entry for root-jail
rcp root@$REMOTE:/jail/etc/passwd jail_etc_passwd
sh mkjail.sh $PREFIX/$TARGET jail_etc_passwd
rcp initjail.sh root@$REMOTE:
cat jail.tar.gz | rsh -l root $REMOTE /bin/sh initjail.sh /jail

# Now test it.  Make sure we can rsh to it, and rcp to and from it.
rsh -l root-jail $REMOTE ls
rm -rf /tmp/xyzzy
rcp root-jail@$REMOTE:/etc/passwd /tmp/xyzzy
rcp /etc/passwd root-jail@$REMOTE:/tmp/xyzzy
