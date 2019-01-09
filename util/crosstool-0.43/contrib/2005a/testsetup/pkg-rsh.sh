#!/bin/sh
# Script to compile rshd and rcp and store the binaries in a tarball
# along with a start script that starts rshd under inetd
set -xe

build=pentium-linux
target=sh4-unknown-linux-gnu
toolcombo=gcc-3.3.2-glibc-2.3.2
targetcflags="-m4 -el -static"

test -d inetutils-1.4.2 && rm -rf inetutils-1.4.2
wget -c ftp://ftp.gnu.org/gnu/inetutils/inetutils-1.4.2.tar.gz
tar -xzvf inetutils-1.4.2.tar.gz
cd inetutils-1.4.2

CC=/opt/crosstool/${target}/${toolcombo}/bin/${target}-gcc PATH=$PATH:/opt/crosstool/${target}/${toolcombo}/bin  CFLAGS=${targetcflags} ./configure --host=${target} --build=${build} \
  --disable-talkd   \
  --disable-telnetd \
  --disable-tftpd   \
  --disable-uucpd   \
  --disable-ftp     \
  --disable-ping    \
  --disable-rlogin  \
  --disable-logger  \
  --disable-talk    \
  --disable-telnet  \
  --disable-tftp    \
  --disable-whois   \
  --disable-ifconfig

PATH=$PATH:/opt/crosstool/${target}/${toolcombo}/bin  make

cd ..
test -d tmp && rm -rf tmp
mkdir -p tmp/bin tmp/sbin tmp/init.d
cp inetutils-1.4.2/rshd/rshd tmp/sbin
cp inetutils-1.4.2/rcp/rcp tmp/bin
cd tmp
# use quotes to avoid interpreting `pwd` while creating following script
cat > init.d/rshd.sh <<"_EOF_"
#!/bin/sh

dir=`dirname $0`
dir=`cd $dir; pwd`
case "$1" in
start)
	# install without taking any precious RAMdisk space
	ln -sf $dir/../sbin/rshd /sbin/rshd
	ln -sf $dir/../bin/rcp /bin/rcp
	echo "shell   stream  tcp     nowait.1000  root    /sbin/rshd" >> /etc/inetd.conf
	killall -HUP inetd
	;;
*)
	echo "Usage: $0 start"
	exit 1
esac

_EOF_
chmod +x init.d/rshd.sh

tar -czvf ../rsh.tgz *
echo Done.
