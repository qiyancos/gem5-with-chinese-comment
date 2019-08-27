#! /bin/bash
root=$PWD

################# settings for script #################
setScript(){
    gem5Dir="/home/lee/project/gem5"
    # root directory path for gem5
    imageDir="$gem5Dir/gem5_fs_images/parsec_disk"
    # root directory path for parsec image
    rcSDir="$gem5Dir/util/parsec_tools/rcSes"
    # directory that hold your rcS file or rcS generator
    tempDir=$root
    # Where the temporary files should be stored
    threads=`lscpu | awk '/^CPU\(s\):/{print $2}'`
    # num of threads you want to use when building gem5.opt
    ISA="X86"
    # [ARM ALPHA RISCV X86 MIPS]
    testSet="test" # [dev large medium small test]
    # Test_set setting
    bad_parsec=""
    # parsec programs that may have problems
    timingMode="Yes" #[Yes/No]
    # Decide if we shall print time-info for running
}

################# settings for gem5 #################
setGem5(){
    # build options
    buildModel="opt" # [fast debug opt]
    # Gem5 building mode
    cpuModel="all" # [AtomicSimpleCPU CheckerCPU MinorCPU O3CPU TimingSimpleCPU all no]
    # CPU Model you want to build into gem5.opt/fast/debug
    forceNotBuild="No" # [Yes/No] 
    # Set this to Yes to force this script not to build gem5 
    m5outDir=""
    # m5outDir option [default m5out]

    enableGlobal="Yes" # [Yes]
        cpuType="DerivO3CPU" # [TimingSimpleCPU AtomicSimpleCPU DerivO3CPU MinorCPU]
        # type of cpu to run with
        cpuVoltage="" # []
        # Top-level voltage for blocks running at system power supply
        sysClock="" # []
        # Top-level clock for blocks running at system speed
        cpuClock="" # []
        # Clock for blocks running at CPU speed
        fastForwardInsts="" # []
        # number of instruction put into warm up region
        maxInsts="" # []
        # max number of instructions will be run
        maxTime="" # []
        # Run to the specified absolute simulated time in seconds
        numCpus="4" # []
        # number of cpu you want to run with

    enableCheckPoint="No" # [Yes/No]
        enableCountAsInstruction="Yes" # [Yes/No]
        # Whether we should take the number as instruction count
        # for the following two options.
        loadCheckPointPos=""
        # num of instructions already run to load checkpoint
        saveCheckPointPos=""
        # num of instructions to take checkpoint
        checkPointDir=""
        # where to put or load the checkpoint file

    enableSimpoint="No" # [Yes/No]
    enableSmt="No" # [Yes/No]

    enableMem="Yes"
        memType="" 
        # ['HBM_1000_4H_1x128', 'DRAMCtrl', 'DDR3_2133_8x8', 'HBM_1000_4H_1x64',
        # 'GDDR5_4000_2x32', 'HMC_2500_1x32', 'LPDDR3_1600_1x32', 'WideIO_200_1x128', 
        # 'DDR4_2400_8x8', 'DDR3_1600_8x8', 'DDR4_2400_4x16', 'DDR4_2400_16x4', 
        # 'SimpleMemory', 'LPDDR2_S4_1066_1x32']
        # type of memory to use
        memChannels=""
        # number of memory channels
        memRanks=""
        # number of memory ranks per channel
        memSize="2GB"
        # Specify the physical memory size (single memory)

    enableCache="Yes"
        numDirs=""
        cacheLineSize=""
        L1Dsize=""
        L1Dassoc=""
        L1Isize=""
        L1Iassoc=""

    enableL2Cache="Yes"
        numL2Caches=""
        L2size=""
        L2assoc=""

    enableL3Cache="No"
        numL3Caches=""
        L3size=""
        L3assoc=""

    enableSWCache="No" # [Yes/No]
        numCpuPerGroup=2

    enableDebug="No" # [Yes/No]
        debugOutputDir="$gem5Dir"
        debugFlags="Commit" #
        # [Flag1,Flag2] use "-h gem5 --debug-help" to see the debug flags 
        debugStartTick=""
        debugEndTick=""
}

################# settings for settingParser #################
setArgLevel(){
    # argument level
    argLevel=("Global" "CheckPoint" "Simpoint" "Smt" "Mem" "Cache" "L2Cache" "L3Cache" "SWCache" "CountAsInstruction")
    formatLevel=("" "" "--simpoint" "--smt" "" "--caches" "--l2cache" "--l3cache" "--swcache" "--at-instruction")

    # argument list
    argListGlobal=("cpuType" "cpuVoltage" "sysClock" "cpuClock" "fastForwardInsts" "maxInsts" "maxTime" "numCpus")
    argListCheckPoint=("loadCheckPointPos" "saveCheckPointPos" "checkPointDir")
    argListMem=("memType" "memChannels" "memRanks" "memSize")
    argListCache=("numDirs" "cacheLineSize" "L1Dsize" "L1Dassoc" "L1Isize" "L1Iassoc")
    argListL2Cache=("numL2Caches" "L2size" "L2assoc")
    argListL3Cache=("numL3Caches" "L3size" "L3assoc")
    argListSWCache=("numCpuPerGroup")

    # format list for arguments
    formatListGlobal=("--cpu-type" "--sys-voltage" "--sys-clock" "--cpu-clock" "--fast-forward" "--maxinsts" "--maxtime" "--num-cpus")
    formatListCheckPoint=("--checkpoint-restore" "--take-checkpoints" "--checkpoint-dir")
    formatListMem=("--mem-type" "--mem-channels" "--mem-ranks" "--mem-size")
    formatListCache=("--num-dirs" "--cacheline_size" "--l1d_size" "--l1d_assoc" "--l1i_size" "--l1i_assoc")
    formatListL2Cache=("--num-l2caches" "--l2_size" "--l2_assoc")
    formatListL3Cache=("--num-l3caches" "--l3_size" "--l3_assoc")
    formatListSWCache=("--cpu_per_group")

    buildOption="CPU_MODELS=$cpuModel"
}

################# parse settings #################
settingParser(){
    ################# test for parsec ##################
    testList=("blackscholes" "bodytrack" "canneal" "dedup" "facesim" "ferret" "fluidanimate" "freqmine" "streamcluster" "swaptions" "vips" "x264" "rtview")
    ################# parse dir args ##################
    if [ ${GEM5_ROOT}x != x ]
    then
        echo "Warning: Environment variable GEM5_ROOT is set, use env-variable"
        echo "         instead of variable in this script!"
        gem5Dir=$GEM5_ROOT
    fi
    ################# parse debug args ##################
    if [ $testSet != "test" ]
    then testSet="sim$testSet"
    fi
    if [ ${enableDebug}x = Yesx ]
    then 
        if [ "${debugFlags}x" != x ]
        then gem5Options="$gem5Options --debug-flags=$debugFlags"
        fi
        if [ ${debugOutputDir}x = x ]
        then gem5Options="$gem5Options --debug-file=DEBUGDIR/DEBUGFILE"
        else
            if [ -d ${debugOutputDir} ]
            then gem5Options="$gem5Options --debug-file=$debugOutputDir/DEBUGFILE"
            else 
                echo "Error: No such directory \"$debugOutputDir\"!" 
                exit 1
            fi
        fi
        if [ ${debugStartTick}x != x ]
        then gem5Options="$gem5Options --debug-start=$debugStartTick"
        fi
        if [ ${debugEndTick}x != x ]
        then basicOptions="$basicOptions -debug-end=$debugEndTick"
        fi
    fi

    if [ ${m5outDir}x != x ]
    then
        if [ -f $m5outDir -a ! -d $m5outDir ]
        then 
            echo "Can not redirect output to $m5outDir: File exists!"
            exit
        else gem5Options="$gem5Options --outdir=$m5outDir"
        fi
    fi

    ################# parse other args ##################
    levelIdx=0
    for level in ${argLevel[*]}
    do
        #echo $level `eval echo -e '$'"enable$level"`
        if [ `eval echo -e '$'"enable$level"`x = Yesx ]
        then
            basicOptions="$basicOptions ${formatLevel[$levelIdx]}"
            argIdx=0
            for argName in `eval echo -e '$'"{argList$level[*]}"`
            do
                realArg=`eval echo -e '$'"$argName"`
                if [ ${realArg}x != x ]
                then basicOptions="$basicOptions `eval echo -e '$'"{formatList$level[$argIdx]}"`=$realArg"
                fi
                argIdx=$[argIdx + 1]
            done
        fi
        levelIdx=$[levelIdx + 1]
    done
    
    ################# settings for fs ##################
    case $ISA in
    "X86") kernelStdName="x86_64-vmlinux-2.6.22.9";;
    *)
        echo "Error: Not supported now."
        exit 1;;
    esac
}

################# print path for output files #################
printOutputInfo(){
    if [ ${runParsec}x != 1x ]
    then
        debugDir="$tempDir"
        gem5Options=`echo $gem5Options | sed "s%DEBUGDIR%$debugDir%"`
    else
        debugDir="$tempDir/parsec/$fullTarget/m5out"
        gem5Options=`echo $gem5Options | sed "s%DEBUGDIR%$debugDir%"`
    fi
    
    if [ ${m5outDir}x = x -a ${runParsec}x = 1x ]
    then 
        echo -e ">> Output files will be saved to \"\033[1;31m$tempDir/parsec/$fullTarget/m5out\033[0m\"!"
        m5outDir="$tempDir/parsec/$fullTarget/m5out"
    elif [ ${m5outDir}x = x ]
    then 
        echo -e ">> Output files will be saved to \"\033[1;31m$tempDir/m5out\033[0m\"!"
        m5outDir=$tempDir/m5out
    else 
        echo -e ">> Output files will be saved to \"\033[1;31m$m5outDir\033[0m\"!"
        m5outDir=$m5outDir
    fi

    if [ ${enableDebug}x = Yesx ]
    then
        if [ ${debugOutputDir}x = x -a ${runParsec}x = 1x ]
        then echo -e ">> Debug data will be saved to \"\033[1;31m$tempDir/parsec/$fullTarget/m5out\033[0m\"!"
        elif [ ${debugOutputDir}x = x ]
        then echo -e ">> Debug data will be saved to \"\033[1;31m$tempDir/m5out\033[0m\"!"
        else echo -e ">> Debug data will be saved to \"\033[1;31m$debugOutputDir\033[0m\"!"
        fi
    fi
}

################# build gem5.opt #################
buildGem5(){
    if [ ${needBuild}x = 1x ]
    then 
        if [ ! -f $gem5Dir/build/$ISA/gem5.$buildModel -o $forceNotBuild != Yes ]
        then 
            cd $gem5Dir
            scons -j$threads build/$ISA/gem5.$buildModel $buildOption
            if [ $? != 0 ]
            then exit
            fi
            if [ $USER != lee ]
            then chmod 777 -R $gem5Dir/build/ 2> /dev/null
            fi
            # This chmod should be enbaled on server!
        fi
    fi
}

################# running gem5 with generated options #################
runGem5(){
    mkdir -p $debugDir
    
    if [ ${m5outDir}x != x -a ! -d $m5outDir/ ]
    then mkdir -p $m5outDir
    else rm -rf $m5outDir/*
    fi
    
    if [ ${runParsec}x != 1x ]
    then
        startTime=`date +%s`
        if [ ${gem5Only}x = 1x ]
        then bash -c "`echo $gem5Dir/build/$ISA/gem5.$buildModel $options`"
        else
            cd $root 
            gem5Options=`echo $gem5Options | sed "s/DEBUGFILE/debug\.log/"`
            bash -c "`echo $gem5Dir/build/$ISA/gem5.$buildModel $gem5Options $gem5Dir/configs/example/fs.py $basicOptions $options`"
        fi
        endTime=`date +%s`
    else
        gem5Options=`echo $gem5Options | sed "s/DEBUGFILE/debug.log/"`
        echo -e ">> Start Running Parsec \033[1;31m[Name: $targetName Thread: $threadNum, TestSet: $testSet]\033[0m ..."
        startTime=`date +%s`
        bash -c "`echo $gem5Dir/build/$ISA/gem5.$buildModel $gem5Options $gem5Dir/configs/example/fs.py $basicOptions $options`"
        endTime=`date +%s`
    fi
}

################# print Infomation for running time #################
printTimingInfo(){
    if [ $timingMode = No ]
    then return
    fi
    echo -e "\033[1;31m[Timing Report]\033[0m"
    echo -e "\033[1;34m    TestSet\tRunning_Time (Hour:Min:Second)\033[0m"
    if [ ${startTime}x != x ]
    then 
        if [ ${#targetName} = 0 ]
        then echo -ne "\033[1;37m    Total\t\033[0m"
        elif [ ${#targetName} -lt 4 ]
        then echo -ne "\033[1;37m    ${targetName}\t\t\033[0m"
        else echo -ne "\033[1;37m    ${targetName}\t\033[0m"
        fi
        if [ ${endTime}x != x ]
        then 
            realTime=$[endTime - startTime]
            realTime="$[realTime / 3600]:$[realTime % 3600 / 60]:$[realTime % 60]"
            echo -e "\033[0;33m${realTime}\033[0m"
        else echo -e "\033[0;31mError\033[0m"
        fi
    fi
}

################# help infomation #################
printHelpInfo(){
    echo "Usage: se [OPTION]" 
    echo "    -h [-h <target file> <more options>]"
    echo "        Use -h to see help info or use more options for gem5.opt and config file!" 
    echo "        If <more options> exists, it will be directly passed to the file specified" 
    echo "        by <target file>; Otherwise, help information will be showed!"
    echo
    echo "        -->Target file options:"
    echo "           <gem5> Show help information for gem5.opt"
    echo "           <fs> Show help information for fs.py"
    echo 
    echo "    -c [-c <port number>] Use -c to connect to the running full-system simulation."
    echo "    -l [-l] Use -l to list valid parsec programs!"
    echo "    -p [-p <parsec program name> <thread number>]  If the parsec program is not given,"
    echo "        then it will be chosen manually!"
    echo "    -mp [-mp <programs list>] (Not support now.) Running multiple programs! Program"
    echo "        list should be like \"prog1=arguments prog2=arguments\"!"
}

################# parse script args #################
argParser(){
    ################# help infomation #################
    if [ "${arguments[1]}x" = -hx -o "${arguments[1]}x" = x ] && [ "${arguments[2]}x" = x ]
    then printHelpInfo;exit
    fi

    case "${arguments[1]}x" in
    ################# connect to fs simulation #################
    "-cx")
        if [ ${arguments[1]}x = x ]
        then
            echo "Warnning: Port number is not given, it will set as 3456 in default."
            portNumber=3456
        else portNumber=${arguments[1]}
        fi
        m5termDir=/usr/local/bin
        if [ ! -f ${m5termDir}/m5term ]
        then
            cd $gem5Dir/util/term
            touch /test.txt &> /dev/null
            if [ $? != 0 ]
            then
                echo "Can not install m5term to your system without sudo."
                m5termDir=$gem5Dir/util/term
            else rm /test.txt
            fi
            echo ">> Installing m5term..."
            make
            sudo make install
        fi
        ${m5termDir}/m5term 127.0.0.1 $portNumber;;
    ################# run multiple programs #################
    "-mpx")
        echo "Error: -mp not supported now. Exit..."
        exit 1
        if [ ${gem5Dir}x = x ]
        then 
            echo "Error: Gem5 root directory not set!"
            exit 1
        fi
        progIdx=0
        while [ "${arguments[$[progIdx + 2]]}x" != x ]
        do
            prog=(`echo ${arguments[$[progIdx + 2]]} | sed 's/=/ /'`)
            if [ -x ${prog[0]} -a ! -d ${prog[0]} ]
            then 
                progList[$progIdx]="${prog[0]}@%"
                argList[$progIdx]="${prog[*]:1:$[${#prog[*]} - 1]}@%"
            else 
                echo "${prog[0]}: No such file or this file is not excutable!"
                exit
            fi
            progIdx=$[progIdx + 1]
        done
        if [ $progIdx = 0 ]
        then
            echo "You must provide at least one program to run!"
            exit
        else 
            progList=`echo "${progList[*]} " | sed 's/@% /;/g'`
            argList=`echo "${argList[*]} " | sed 's/@% /;/g'`
            progList=${progList:0:$[${#progList} - 1]}
            argList=${argList:0:$[${#argList} - 1]}
        fi
        if [ "${argList}x" != x ]
        then options="-c '$progList' -o '$argList'"
        else options="-c '$progList'"
        fi
        needBuild=1;;
    
    ################# run with given args #################
    "-hx")
        if [ ${gem5Dir}x = x ]
        then 
            echo "Error: Gem5 root directory not set!"
            exit 1
        fi
        case "${arguments[2]}x" in
        gem5x)
            if [ ! -f $gem5Dir/build/$ISA/gem5.$buildModel ]
            then needBuild=1
            fi
            gem5Only=1
            if [ "${arguments[3]}x" = x ]
            then options="-h"
            else options=${arguments[*]:3:$[${#arguments[*]} - 3]}
            fi;;
        fsx)
            if [ ! -f $gem5Dir/build/$ISA/gem5.$buildModel ]
            then needBuild=1
            fi
            if [ "${arguments[3]}x" = x ]
            then options="-h"
            else
                argIdx=3
                while [ "${arguments[$argIdx]}x" != x ]
                do 
                    if [ `echo ${arguments[$argIdx]} | sed 's/;/ /g' | wc -w` -gt 1 ]
                    then options="$options '${arguments[$argIdx]}'"
                    else options="$options ${arguments[$argIdx]}"
                    fi
                    argIdx=$[argIdx + 1]
                done
            
                if [ "`echo $options | grep "\-c\|\-\-cmd"`"x != x ]
                then needBuild=1
                fi
            fi;;
        *)
            echo "Bad argument \"${arguments[2]}\"! Use -h for help!"
            exit;;
        esac;;
    
    ################# list available parsec programs #################
    "-lx")
        if [ ! -d $rcSDir ]
        then
            echo "Error: could not locate rcS dir \"$rcSDir\"!"
            exit 1
        fi
        for item in ${testList[*]}
        do echo $item
        done
        exit;;

    ################# select available parsec programs #################
    "-px")
        if [ ${gem5Dir}x = x ]
        then 
            echo "Error: Gem5 root directory not set!"
            exit 1
        fi
        if [ ! -d $rcSDir ]
        then
            echo "Error: could not locate rcS dir \"$rcSDir\"!"
            exit 1
        fi
        
        if [[ ! "${arguments[2]}x" =~ ^[0-9]*x$ ]]
        then
            if [ "${arguments[3]}x" != x ]
            then threadNum=${arguments[3]}
            else threadNum=4
            fi
            argIdx=0
            while [ $argIdx -lt ${#testList[*]} ]
            do
                if [ "${arguments[2]}" = ${testList[$argIdx]} ]
                then 
                    targetName=${arguments[2]}
                    break
                fi
                argIdx=$[argIdx + 1]
            done
            if [ ${targetName}x = x ]
            then 
                echo "${arguments[2]}: Bad program name or the parsec program is not valid!"
                echo "Use -l to see the available parsec program list!"
                exit
            fi
        else
            select name in ${testList[*]}
            do 
                targetName=$name
                break
            done
            if [ "${arguments[2]}x" != x ]
            then threadNum=${arguments[2]}
            else
                while [[ ! "${threadNum}x" =~ ^[1-9][0-9]*x$ ]]
                do
                    echo -n "Please input the thread num for benchmrak [$targetName]: "
                    read threadNum
                done
            fi
        fi
        runParsec=1
        needBuild=1;;
    *) 
        echo "Bad argument \"${arguments[1]}\"! Use -h for help!"
        exit;;
    esac
}

################# mkdir and generate options for parsec #################
preBuildForParsec(){
    if [ ${runParsec}x != 1x ]
    then return 
    fi
    
    mkdir -p $tempDir/parsec/$targetName
    
    if [ ! -f $rcSDir/${targetName}_${threadNum}c_${testSet}.rcS ]
    then
        if [ ! -x $rcSDir/writescript.pl ]
        then
            echo "Error: Can not find executable perl script to generate rcS file."
            exit 1
        fi
        echo ">> Building rcS script for $targetName with thread number $threadNum..."
        cd $rcSDir
        $rcSDir/writescript.pl $targetName $threadNum
        if [ ! -f $rcSDir/${targetName}_${threadNum}c_${testSet}.rcS ]
        then
            echo "Error: Failed to generate rcS file."
            exit 1
        fi
    fi
    
    cd $tempDir/parsec/$fullTarget

    if [ ! -f $imageDir/binaries/$kernelStdName ]
    then
        kernelPath=(`find $imageDir -name "*vmlinux*"`)
        if [ ${#kernelPath[*]} == 0 ]
        then
            echo "Error: Can not find linux kernel in $imageDir."
            exit 1
        elif [ ${#kernelPath[*]} -gt 1 ]
        then
            echo ">> We have found multiple candidate for kernel. You will have to choose by yourself:"
            select path in ${kernelPath[*]}
            do
                kernelPath=$path
                break;
            done
        fi
        ln -s $kernelPath $imageDir/binaries/$kernelStdName
    fi
    kernelPath=$imageDir/binaries/$kernelStdName

    diskImagePath=(`find $imageDir -name "*.img" | sed /swap/d`)
    if [ ${#diskImagePath[*]} == 0 ]
    then
        echo "Error: Can not find linux file system image in $imageDir."
        exit 1
    elif [ ${#diskImagePath[*]} -gt 1 ]
    then
        echo ">> We have found multiple candidate for system image. You will have to choose by yourself:"
        select path in ${diskImagePath[*]}
        do
            diskImagePath=$path
            break;
        done
    fi

    if [ ! -f $gem5Dir/configs/common/SysPaths.py.bak ]
    then cp $gem5Dir/configs/common/SysPaths.py $gem5Dir/configs/common/SysPaths.py.bak
    fi
    if [ "`sed -n /poolfs/p $gem5Dir/configs/common/SysPaths.py.bak`x" = x ]
    then
        echo "Error: Your config file $gem5Dir/configs/common/SysPaths.py(.bak) seems to be changed unexpectedly."
        exit 1
    else
        sed "s%/n/poolfs/z/dist/m5/system%$imageDir%g" \
                $gem5Dir/configs/common/SysPaths.py.bak \
                > $gem5Dir/configs/common/SysPaths.py
    fi

    options="--kernel=$kernelPath --disk-image=$diskImagePath --script=$rcSDir/${targetName}_${threadNum}c_${testSet}.rcS"
}

argIdx=1
arguments[0]="$0"
while [ "${1}x" != x ]
do
    arguments[$argIdx]="$1"
    shift 1
    argIdx=$[argIdx + 1]
done

setScript
setGem5
setArgLevel
settingParser
argParser
buildGem5
set -e
preBuildForParsec
printOutputInfo
runGem5
printTimingInfo
