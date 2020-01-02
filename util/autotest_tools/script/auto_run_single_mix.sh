#! /bin/bash
set -e
root=`dirname $0`
cd $root/..
root=$PWD

cpuNum=`lscpu | awk '/^CPU\(s\):/{print $2}'`
cpuNum=$[cpuNum * 3 / 5]
testList=`$root/script/se -l`
# testList=`$root/script/se -l | sed 's/\./ /g' | awk '{print $1}'`
# testTarget="core8_l1np_l2np  core8_l1p2_l2np  core8_l1p4_l2np core8_l1p8_l2np"
testTarget="3l_size_effect"
testFolder=pre_test_1
testTaskNums="8"
newTaskGap="1m"

##############################################################################

runTask() {
    echo "-- Start running test \"$file\" for $testName with $taskNum task(s):"
    taskDir=$root/data/$testFolder/$file/$testName/$taskNum
    mkdir -p $taskDir
    cd $root/data/$testFolder/$file/$testName/$taskNum
    rm -rf $taskDir/spec
    testNum=`echo $testName | sed 's/\..*//g'`
    unset realTestName realTestDir
    for((i = 0; i < $taskNum; i++))
    do
        realTestDir="${realTestDir}_${testNum}"
        realTestName="${realTestName} ${testNum}"
    done
    realTestDir=${realTestDir:1:$[${#realTestDir} - 1]}
    echo "-- Running Command: $root/test_script/$file -p $realTestName"
    set +e
    $root/test_script/$file -p $realTestName
    set -e
    if [ -s $taskDir/spec/$realTestDir/m5out/stats* ]
    then
        cp -r $taskDir/spec/$realTestDir/m5out/* $taskDir/
        echo "-- \"$file\" test over for $testName with $taskNum task(s)."
    else
        echo -n "Error: some errors occurred with test: "
        echo "$file-$testName-$taskNum"
        echo $task >> /tmp/retry.list
    fi
    rm -rf $taskDir/spec
}

initRunTask() {
    echo "-- Generating task list..."
    for target in $testTarget
    do
        testFile=`cd $root/test_script && find ./$target -type f`
        for file in $testFile
        do
            fileName=(`basename $file | sed 's/_/ /g'`)
            for taskNum in $testTaskNums
            do
                for testName in $testList
                do runTasks="$runTasks ${file}:${taskNum}:${testName}"
                done
            done
        done
    done
    
    if [ -f /tmp/mp.lock ]
    then
        echo "Error: Multi program is already locked."
        exit -1
    else
        lockID=`date | base64 -i`
        echo "$lockID" > /tmp/mp.lock
        echo "0 idle" > /tmp/mp.list
        threadID=1
        while [ $threadID -lt $cpuNum ]
        do
            echo "$threadID idle" >> /tmp/mp.list
            threadID=$[threadID + 1]
        done
        rm /tmp/mp.lock
    fi
}

atomicChange() {
    lockID=`date | base64 -i`
    echo ">> Trying get lock..."
    while [ -f /tmp/mp.lock ]
    do sleep 0.1
    done
    while [ ! -f /tmp/mp.lock ]
    do echo "$lockID" > /tmp/mp.lock
    done
    
    while [ x"`cat /tmp/mp.lock`" != x$lockID ]
    do
        while [ -f /tmp/mp.lock ]
        do sleep 0.1
        done
        while [ ! -f /tmp/mp.lock ]
        do echo "$lockID" > /tmp/mp.lock
        done
    done
    echo ">> Control file locked."
    sed -i "/^$1 /c$1 $2" /tmp/mp.list
    rm /tmp/mp.lock
    echo ">> Control file unlocked."
}

runAllTask() {
    for task in $runTasks
    do
        echo "-- Trying to find idle cpu for new task($task)..."
        unset idleThreadID
        unset nocpuInfo
        while [ 1 ]
        do
            threadID=0
            while [ $threadID -lt $cpuNum ]
            do
                if [ "x`grep "^$threadID idle" /tmp/mp.list`" != x ]
                then
                    idleThreadID=$threadID
                    break
                fi
                threadID=$[threadID + 1]
            done
            if [ x$idleThreadID = x ]
            then
                if [ x$nocpuInfo = x ]
                then
                    echo "-- No cpu available now."
                    nocpuInfo=1
                fi
                sleep 20
            else
                echo "-- Found cpu $idleThreadID available for task($task)."
                break
            fi
        done
        atomicChange $idleThreadID "run $task"
        taskArr=(`echo $task | sed 's/:/ /g'`)
        file=${taskArr[0]}
        taskNum=${taskArr[1]}
        testName=${taskArr[2]}
        {
            runTask
            atomicChange $idleThreadID "idle"
        } &
        sleep $newTaskGap
    done
}

retryRunTask() {
    retryTasks=$(cat /tmp/retry.list)
    if [ "x$retryTasks" != x ]
    then
        echo "-- Following task didn't run properly:"
        for task in $retryTasks
        do echo -e "\t$task"
        done
        runTasks=$retryTasks
        retryFlag=1
    else retryFlag=0
    fi
}

initRunTask
while [ x$retryFlag = x -o x$retryFlag = x1 ]
do
    rm -rf /tmp/retry.list
    echo "-- Start running left tasks."
    runAllTask
    echo "-- Left tasks running over."
    retryRunTask
    exit
done
