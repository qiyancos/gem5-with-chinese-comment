#! /bin/bash
if [ ${1}x = x -o ${1}x = -hx ]
	then echo "Usage: sim_fs [options <arguments>]"
	echo "[start] <mode> start simulation with full_system[fs] or benchmark[spec] mode "
	echo "    fs mode: this script will also mount system image to /mnt/0 "
	echo "        before the simulation."
	echo "    spec mode: not supportted here, please use script \"se\" in "
	echo "        the gem5 root directory instead!"
	echo "[link] <port> use m5term to link with the simulation system."
	echo "[mount] <dir> mount the system image file to the given directory."
	exit
fi

root=`dirname $0`
cd $root
root=$PWD
fsDir=$root/gem5_fs_images/x86-system
export M5_PATH=$fsDir

sudo=0
touch /test.txt &> /dev/null
if [ $? = 0 ]
then 
	sudo=1
	rm /test.txt
fi

if [ ${1}x = linkx ]
	then 
		if [ ${2}x = x ]
			then 
				echo "Port number must be provided!"
				exit
			else
				if [ ! -f /usr/local/bin/m5term ]
				then 
					cd $root/util/term
					make
					if [ $sudo != 1 ]
					then
						echo "We cannot install m5term for you without sudo!"
						m5termDir=$root/util/term/
					else sudo make install
					fi
				fi
				${m5termDir}m5term 127.0.0.1 $2
		fi
	exit
fi
if [ ${1}x = startx ]
then
	if [ ${2}x = fsx ]
	then 
		echo "System is now booting!"
		echo -e "You may use \"fs [link]\" to connect with the system in another terminal."
		$root/build/X86/gem5.opt $root/configs/example/fs.py # --disk-image=$fsDir/disks/linux-x86.img --kernel=$fsDir/binaries/x86_64-vmlinux-2.6.22.9
		exit
	fi
fi

if [ ${1}x = mountx ]
then 
	if [ ${2}x != x ]
	then 
		if [ -d $2 ]
		then mountDir=$2
		else 
			echo "$2: No such file or directory!"
			exit
		fi
	else mountDir=$root/mount/system_img
	fi

	if [ ! -d $mountDir/sbin ]
	then
		if [ $sudo = 0 ]
		then 
			echo "You must run with sudo to mount image!"
			exit
		else
			mkdir -p $mountDir
			mount -o loop,offset=32256 $fsDir/disks/linux-x86.img $mountDir
			echo "Your system image has been mounted to $mountDir !"
		fi
	else echo "Your system image file has already been mounted to $mountDir!"
	fi
else echo "Bad argument! Use -h for help."
fi
