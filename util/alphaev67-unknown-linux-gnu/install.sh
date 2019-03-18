#! /bin/bash
touch /a.txt 2> /dev/null
if [ $? != 0 ]
then 
	echo "You need to use sudo to install alpha toolchain!"
	exit 1
else rm /a.txt
fi

root=`dirname $0`
cd $root
dir=`basename $PWD`
cd ..
cp -r ./$dir /usr/
chmod -R 755 /usr/$dir
echo "export PATH=\$PATH:/usr/$dir/bin" >> /etc/profile
echo "$dir already installed!"
echo "You may use \"source /etc/profile\" to use alpha toolchain" 
