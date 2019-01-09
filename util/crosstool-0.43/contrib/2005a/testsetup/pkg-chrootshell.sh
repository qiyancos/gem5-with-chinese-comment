#!/bin/sh
# Script to compile chrootshell and store the binaries in a tarball
# along with a startup script

set -xe

build=pentium-linux
target=sh4-unknown-linux-gnu
toolcombo=gcc-3.3.2-glibc-2.3.2
targetcflags="-m4 -el -static"

test -d tmp && rm -rf tmp
mkdir -p tmp/sbin tmp/init.d

CC=/opt/crosstool/${target}/${toolcombo}/bin/${target}-gcc

$CC -DDEBUG_PRINTS chrootshell.c $targetcflags -o tmp/sbin/chrootshell

# use quotes to avoid interpreting `pwd` while creating following script
cat > tmp/init.d/chrootlogin.sh <<"_EOF_"
#!/bin/sh

dir=`dirname $0`
dir=`cd $dir; pwd`
case "$1" in
start)
	# install without taking any precious RAMdisk space
	ln -sf $dir/../sbin/chrootshell /sbin/chrootshell
	chmod 4755 /sbin/chrootshell
	echo "root-jail::0:0:Outer Jail Account:/jail:/sbin/chrootshell" >> /etc/passwd
	;;
*)
	echo "Usage: $0 start"
	exit 1
esac

_EOF_
chmod +x tmp/init.d/chrootlogin.sh

cd tmp
tar -czvf ../chrootlogin.tgz *
echo Done.


