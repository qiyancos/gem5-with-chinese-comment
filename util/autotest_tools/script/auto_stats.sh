#! /bin/bash
localDir=$PWD
root=`dirname $0`
cd $root/..
root=$PWD

cpuNum=`lscpu | awk '/^CPU\(s\):/{print $2}'`
testList=`$root/script/se -l`
# testFile=`ls $root/test_script`
eval "$(grep "^testTarget=" $root/script/auto_run_single_mix.sh)"
testTaskNums="8"
statsTitle="TestName, Task Number, Test Subset"

singleStatsList=("")
singleStatsString=("")
multiStatsList=("IPC"
        "L1D_Pref_Issued" "L1D_Pref_Identidied" "L1D_Pref_BufferHit"
        "L1D_Pref_CacheHit" "L1D_Pref_RemoveFull" "L1D_Pref_RemoveCrossPage"
        "L2_Pref_Issued" "L2_Pref_Identidied" "L2_Pref_BufferHit"
        "L2_Pref_CacheHit" "L2_Pref_RemoveFull" "L2_Pref_RemoveCrossPage")
multiStatsString=("ipc_total"
        "dcache.prefetcher.num_hwpf_issued" "dcache.prefetcher.pfIdentified"
        "dcache.prefetcher.pfBufferHit" "dcache.prefetcher.pfInCache"
        "dcache.prefetcher.pfRemovedFull" "dcache.prefetcher.pfSpanPage"
        "l2.prefetcher.num_hwpf_issued" "l2.prefetcher.pfIdentified"
        "l2.prefetcher.pfBufferHit" "l2.prefetcher.pfInCache"
        "l2.prefetcher.pfRemovedFull" "l2.prefetcher.pfSpanPage")

statsList="IPC"

################################################################

singleStats() {
    statsIndex=0
    while [ x${singleStatsList[$statsIndex]} != x ]
    do
        if [ ${singleStatsList[$statsIndex]} = $1 ]
        then break
        else statsIndex=$[statsIndex + 1]
        fi
    done
    if [ $statsIndex = ${#singleStatsList[*]} ]
    then
        echo "Error: Stats \"$1\" not found in single stats list."
        exit -1
    fi
    statsString=${singleStatsString[$statsIndex]}
    statsVal=`eval "awk '/$statsString/ {print \\$2}' $2"`
    if [ x$(eval "echo \$statsFlag$1") = x ]
    then
        statsTitle="$statsTitle, $1"
        eval "statsFlag$1=1"
    fi
    echo -n "$statsVal, "
}

multiStats() {
    statsIndex=0
    while [ x${multiStatsList[$statsIndex]} != x ]
    do
        if [ ${multiStatsList[$statsIndex]} = $1 ]
        then break
        else statsIndex=$[statsIndex + 1]
        fi
    done
    if [ $statsIndex = ${#multiStatsList[*]} ]
    then
        echo "Error: Stats \"$1\" not found in multi stats list."
        exit -1
    fi
    statsString=${multiStatsString[$statsIndex]}
    statsVal=(`eval "awk '/$statsString/ {print \\$2}' $2"`)
    index=0
    if [ x$(eval "echo \$statsFlag$1") = x ]
    then
        eval "statsFlag$1=1"
        index=0
        while [ $index -lt $coreNum ]
        do
            statsTitle="$statsTitle, ${1}_$index"
            echo -n "${statsVal[$index]}, "
            index=$[index + 1]
        done
    else
        index=0
        while [ $index -lt $coreNum ]
        do
            echo -n "${statsVal[$index]}, "
            index=$[index + 1]
        done
    fi
}

statsFile() {
    taskDir=$root/data/pre_test_1/$file/$testName/$taskNum
    fileName="$taskDir/stats*"
    if [ -f $fileName ]
    then
        echo -n "$file, $taskNum, $testName, " >> $1
        for stats in $statsList
        do
            if [ "x`echo ${multiStatsList[*]} | grep $stats`" != x ]
            then multiStats $stats $fileName >> $1
            elif [ "x`echo ${singleStatsList[*]} | grep $stats`" != x ]
            then singleStats $stats $fileName >> $1
            else
                echo "Error: Stats \"$stats\" not found in statsList."
                exit -1
            fi
        done
        echo >> $1
    else echo "$file, $taskNum, $testName, Error" >> $1
    fi
}

if [ x$1 = x ]
then
    echo ">> Result will be saved to $localDir/stats.csv"
    targetFile=$localDir/stats.csv
else targetFile=$1
fi
rm -rf $targetFile
mkdir -p `dirname $targetFile`
if [ $? != 0 ]
then
    echo "Error: Failed to build dir for target \"$targetFile\"."
    exit -1
fi

for target in $testTarget
do
    testFile=`cd $root/test_script && find ./$target -type f`
    for file in $testFile
    do
        coreNum=`basename $file | sed 's/core\([0-9]*\).*/\1/g'`
        echo "-- Processing Test File $file"
        for testName in $testList
        do
            for taskNum in $testTaskNums
            do statsFile $targetFile
            done
            echo "   $testName Done."
        done
    done
done
sed -i "1i\\$statsTitle" $targetFile
