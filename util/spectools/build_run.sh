#! /bin/bash 
#set -e
if [ ${1}x = x -o ${1}x = -hx ]
then 
    echo "Usage: `basename $0` [run/build <config file name>]"
    exit    
fi

if [ ${2}x = x ]
then 
    echo "No config file has been specified!"
    exit
fi

testSet="train"

root=`dirname $0`
if [ $root = . ]
then root=$PWD
fi

source $root/shrc
pushd $root/benchspec/CPU2006

if [ -f /bin/lsnc ]
then file_list=`lsnc`
else file_list=`ls`
fi

popd

for file in $file_list
do
    name=(`echo $file | sed 's/\./ /g'`)
    if [[ ${file:0:1} =~ ^[0-9]$ ]]
    then
        if [ ${1}x = runx ]
        then 
            echo "===============================================================" 
            echo "Runing Benchamark ${name[1]}"
            timeout -k 60 60 runspec -i $testSet --config=$root/config/$2 \
                    -iteration=1 ${name[1]}
            echo "Benchamark test over!"
            echo "==============================================================="
        elif [ ${1}x = buildx ]
        then
            sed -i 's/lock=1/lock=0/g' $root/benchspec/CPU2006/$file/run/list
            echo "==============================================================="
            echo "Building Benchamark ${name[1]}"
            runspec --config=$root/config/$2 --action=build ${name[1]}
            echo "Benchamark build over!"
            echo "==============================================================="
        else echo "Unavailable argument! Use -h for help!"
        fi
    fi
done
