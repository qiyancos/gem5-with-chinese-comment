#! /bin/bash
set -e
if [ ${1}x = x -o ${1}x = -hx ]
	then echo "Usage: sim_fs [start <mode>] [link <port>]"
	echo "[start] start simulation with full_system[fs] or benchmark[spec] mode "
	echo "        fs mode: this script will also mount system image to /mnt/0 "
	echo "                 before the simulation."
	echo "        spec mode: not supportted here, please use script \"sim_se\" in "
	echo "                   the gem5 root directory instead!"
	echo "[link] use m5term to link with the simulation system."
	exit
fi

root=`dirname $0`
if [ $root = . ]
then root=$PWD
fi
cd $root
fsDir=$root/gem5_fs_images/x86-system

if [ ${1}x = linkx ]
	then 
		if [ ${2}x = x ]
			then echo "Port number must be provided!"
			exit
			else m5term 127.0.0.1 $2
		fi
	exit
fi
if [ ${1}x = startx ]
	then mode=$2
	if [ ${2}x = fsx ]
		then echo "System is now booting!"
		echo -e "You may use \"fs_sim.sh [link]\" to connect with the system in another terminal."
		if [ ! -d /mnt/0/sbin ]
			then sudo mount -o loop,offset=32256 $fsDir/disks/linux-x86.img /mnt/0
		fi
		$root/build/X86/gem5.opt configs/example/fs.py --disk-image=$fsDir/disks/linux-x86.img
		exit
	else 
		echo "Bad argument! Use -h for help."
		exit
	fi
fi
