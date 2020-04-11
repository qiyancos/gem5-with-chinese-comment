#! /bin/bash
set -e
root=`dirname $0`
cd $root/..
root=$PWD

cpuNum=`lscpu | awk '/^CPU\(s\):/{print $2}'`
cpuNum=$[cpuNum * 7 / 10]
testList="400.perlbench 401.bzip2 403.gcc 435.gromacs 437.leslie3d 444.namd 445.gobmk 447.dealII 450.soplex 453.povray 456.hmmer 458.sjeng 459.GemsFDTD 462.libquantum 464.h264ref 470.lbm 471.omnetpp 473.astar 481.wrf 482.sphinx3 483.xalancbmk"
#"459.GemsFDTD 447.dealII 473.astar" #`$root/script/se -l`
testTarget="l2_ppf_simple_test"
testFolder=std_test
newTaskGap="30"

##############################################################################

runTask() {
    cd $taskDir
    echo "-- Start running test \"$file\" for $testName with $taskNum task(s):"
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
    $root/test_script/$file -p $realTestName 2>&1 | tee $taskDir/running_log
    set -e
    if [ $taskNum = 1 ]
    then realTestDir=$testName
    fi
    if [ -s $taskDir/spec/$realTestDir/m5out/stats* ]
    then
        cp -r $taskDir/spec/$realTestDir/m5out/* $taskDir/
        echo "-- \"$file\" test over for $testName with $taskNum task(s)."
        failFlag=0
    else
        echo -n "Error: some errors occurred with test: "
        echo "$file-$testName-$taskNum"
        echo $task >> /tmp/retry.list
        failFlag=1
    fi
    # rm -rf $taskDir/spec
}

initRunTask() {
    echo "-- Generating task list..."
    cd $root/test_script
    taskCount=0
    for target in `echo $testTarget`
    do
        testFile=`find ./$target -type f`
        for file in $testFile
        do
            fileName=(`basename $file | sed 's/_/ /g'`)
            taskNum=`basename $file | sed 's/task\([0-9]*\)_.*/\1/g'`
            for testName in $testList
            do
                echo "    ${file}:${taskNum}:${testName}"
                runTasks="$runTasks ${file}:${taskNum}:${testName}"
                taskCount=$[taskCount + 1]
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
        echo "Fail 0" >> /tmp/mp.list
        echo "Pass 0" >> /tmp/mp.list
        echo "Over 0" >> /tmp/mp.list
        echo "Total $taskCount" >> /tmp/mp.list
        rm -rf /tmp/mp.lock
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
    rm -rf /tmp/mp.lock
    echo ">> Control file unlocked."
}

runAllTask() {
    taskCount=0
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
        taskDir=$root/data/$testFolder/$file/$testName
        rm -rf $taskDir
        mkdir -p $taskDir
        {
            runTask
            failCount=`awk '/Fail/ {print $2}' /tmp/mp.list`
            passCount=`awk '/Pass/ {print $2}' /tmp/mp.list`
            taskCount=`awk '/Over/ {print $2}' /tmp/mp.list`
            if [ $failFlag = 1 ]
            then atomicChange "Fail" "$[failCount + 1]"
            else atomicChange "Pass" "$[passCount + 1]"
            fi
            atomicChange "Over" "$[taskCount + 1]"
            atomicChange $idleThreadID "idle"
        } &
        while [ "x`find $taskDir -name "m5out"`" = x ]
        do
            sleep 4
        done
        waitTime=0
        while [ 1 ]
        do
            sleep 4
            waitTime=$[waitTime + 4]
            if [ $waitTime -gt $newTaskGap ]
            then break
            fi
        done
    done
}

retryRunTask() {
    retryTasks=$(cat /tmp/retry.list)
    if [ "x$retryTasks" != x ]
    then
        echo "-- Following task didn't run properly:"
        for task in $retryTasks
        do echo -e "    $task"
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
