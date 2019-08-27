#! /bin/bash 
#set -e
if [ ${1}x = x -o ${1}x = -hx ]
	then echo "Usage: `basename $0` [run/build <config file name>] [runRaw]"
	exit
fi
root=`dirname $0`
if [ $root = . ]
	then root=$PWD
fi
source $root/shrc
testSet="train"
cd $root/benchspec/CPU2006

if [ -f /bin/lsnc ]
then file_list=`lsnc`
else file_list=`ls`
fi

if [ ${2}x = runRawx ]
then rm -rf $root/runSpec
fi

for file in $file_list
do
	name=(`echo $file | sed 's/\./ /g'`)
	#echo "${name[0]:0:1}"
	if [ ${2}x = x -a ${1}x != runRawx ]
		then echo "No config file has been specified!"
		exit
	fi
	if [ ${1}x = runx ]
	then 
		if [ ${name[0]:0:1} = '4' -o ${name[0]:0:1} = '9' ]
		then
			echo "===============================================================" 
			echo "Runing Benchamark ${name[1]}"
			timeout -k 60 60 runspec -i $testSet --config=$root/config/$2 -iteration=1 ${name[1]}
			echo "Benchamark test over!"
			echo "==============================================================="
		fi
	elif [ ${1}x = buildx ]
	then
		if [ ${name[0]:0:1} = '4' -o ${name[0]:0:1} = '9' ]
		then 
			sed -i 's/lock=1/lock=0/g' $root/benchspec/CPU2006/$file/run/list
			echo "==============================================================="
			echo "Building Benchamark ${name[1]}"
			runspec --config=$root/config/$2 --action=build ${name[1]}
			echo "Benchamark build over!"
			echo "==============================================================="
		fi
	elif [ ${1}x = runRawx ]
    then $root/runSpec -p ${name[1]} | tee -a ./run_raw.log
    else echo "Unavailable argument! Use -h for help!"
	fi
done
#my-lsch
