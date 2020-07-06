#! /bin/bash
root=`dirname $0`
cd $root/..
root=$PWD

TaskCountList="1 2 4 8 16"
L1DSizeList="16kB 32kB 64kB"
L2SizeList="128kB 256kB 512kB"
L3SizeList="8MB 16MB 32MB"
PrefetcherList="TaggedPrefetcher BOPPrefetcher DCPTPrefetcher SignaturePathPrefetcherV2 StridePrefetcher"
TaggedPrefDegreeList="2 4 8 16"
StridePrefConfThreshList="2 4 6"
StridePrefDegreeList="2 4 8 16"

genL2PPFDebugTest() {
    echo ">> Generating test scripts for L2PPFDebugTest..."
    targetDir=$root/test_script/l2_ppf_debug_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 2 4 8 16
    do
        conf=4
        for degree in 0 16
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    $root/script/se > \
                    $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
        done
    done
}

genL2PPFFullTest() {
    echo ">> Generating test scripts for L2PPFFullTest..."
    targetDir=$root/test_script/l2_ppf_full_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 2 4 6 8 10 12 14 16
    do
        conf=4
        degree=16
        for useFilter in True False
        do
            if [ $useFilter == True ]
            then targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_filter
            else targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_nofilter
            fi
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/enableFilter=True/enableFilter=$useFilter/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
        degree=0
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                $root/script/se > \
                $targetDir/task${taskNum}_${pref}_${conf}_${degree}
        chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
    done
}

genL2PPFSimpleTest() {
    echo ">> Generating test scripts for L2PPFSimpleTest..."
    targetDir=$root/test_script/l2_ppf_simple_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 16
    do
        conf=4
        degree=16
        for useFilter in True False
        do
            if [ $useFilter == True ]
            then targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_filter
            else targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_nofilter
            fi
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/enableFilter=True/enableFilter=$useFilter/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
        degree=0
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                $root/script/se > \
                $targetDir/task${taskNum}_${pref}_${conf}_${degree}
        chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
    done
}

genL2PPFInferenceTest() {
    echo ">> Generating test scripts for L2PPFInferenceTest..."
    targetDir=$root/test_script/l2_ppf_inference_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    conf=4
    degree=16
    for taskNum in 1 2 4 8 16
    do
        # DRAM Channels
        for channels in 1 2 4
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/memChannels=\"2\"/memChannels=\"$channels\"/" \
                    -e "s/enableFilter=True/enableFilter=False/" \
                    $root/script/se > \
                    $targetDir/task${taskNum}_${pref}_channels_${channels}
            chmod +x $targetDir/task${taskNum}_${pref}_channels_${channels}
        done
        # L2 MSHRs
        for mshr in 16 24 32 40
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/L2MSHR=\"20\"/L2MSHR=\"$mshr\"/" \
                    -e "s/enableFilter=True/enableFilter=False/" \
                    $root/script/se > \
                    $targetDir/task${taskNum}_${pref}_mshr_${mshr}
            chmod +x $targetDir/task${taskNum}_${pref}_mshr_${mshr}
        done
        # L2 PFQ Size
        for pfqSize in 16 32 48 64 
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/prefetchQueueSize=32/prefetchQueueSize=$pfqSize/" \
                    -e "s/enableFilter=True/enableFilter=False/" \
                    $root/script/se > \
                    $targetDir/task${taskNum}_${pref}_pfq_${pfqSize}
            chmod +x $targetDir/task${taskNum}_${pref}_pfq_${pfqSize}
        done
    done
}

genL2PPFTagBitsTest() {
    echo ">> Generating test scripts for L2PPFTagBitsTest..."
    targetDir=$root/test_script/l2_ppf_tag_bits_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 2 4 8 16
    do
        conf=4
        degree=16
        for tagBits in 6 12 -1
        do
            targetFile=$targetDir/task${taskNum}_${pref}_${tagBits}
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/tagBits=6/tagBits=$tagBits/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
    done
}

genL2PPFOriginPPFTest() {
    echo ">> Generating test scripts for L2PPFOriginPPFTest..."
    targetDir=$root/test_script/l2_ppf_origin_ppf_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 16
    do
        conf=4
        # Pref NoUpgrade
        degree=16
        enableFilter=True
        enableUpgrade=False
        useHarm=False
        targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_noupgrade_noharmtable
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                -e "s/enableFilter=True/enableFilter=$useFilter/" \
                -e "s/useHarmTable=True/useHarmTable=$useHarm/" \
                -e "s/enableUpgrade=True/enableUpgrade=$enableUpgrade/" \
                $root/script/se > $targetFile
        chmod +x $targetFile
    done
}

genL2PPFLevelUpTest() {
    echo ">> Generating test scripts for L2PPFLevelUpTest..."
    targetDir=$root/test_script/l2_ppf_level_up_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 16
    do
        conf=4
        # Pref NoUpgrade
        degree=16
        enableFilter=True
        enableUpgrade=False
        targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_noupgrade
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                -e "s/enableFilter=True/enableFilter=$useFilter/" \
                -e "s/enableUpgrade=True/enableUpgrade=$enableUpgrade/" \
                $root/script/se > $targetFile
        chmod +x $targetFile
    done
}

genL2PPFHarmTableTest() {
    echo ">> Generating test scripts for L2PPFHarmTableTest..."
    targetDir=$root/test_script/l2_ppf_harm_table_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 16
    do
        conf=4
        # Pref NoUpgrade
        degree=16
        useHarm=False
        targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_noharmtable
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                -e "s/useHarmTable=True/useHarmTable=$useHarm/" \
                $root/script/se > $targetFile
        chmod +x $targetFile
    done
}

genL2PPFTableSizeTest() {
    echo ">> Generating test scripts for L2PPFTableSizeTest..."
    targetDir=$root/test_script/l2_ppf_table_size_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 16
    do
        conf=4
        # Pref NoUpgrade
        degree=16
        for tableSize in 128 256 512 1024 2048 4096
        do
            targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_table_$tableSize
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/ppfTableSize=1024/ppfTableSize=$tableSize/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
    done
}

genL2PPFBestDegreeTest() {
    echo ">> Generating test scripts for L2PPFBestDegreeTest..."
    targetDir=$root/test_script/l2_ppf_best_degree_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 2 4 6 8 10 12 14 16
    do
        conf=4
        # Pref NoUpgrade
        for degree in 0 4 8 12 16 20 24 28 32
        do
            targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/enableFilter=True/enableFilter=False/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
    done
}

genL2PPFL1PrefTest() {
    echo ">> Generating test scripts for L2PPFL1PrefTest..."
    targetDir=$root/test_script/l2_ppf_l1_pref_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1
    do
        conf=4
        degree=16
        targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}
        sed -e "s/L1Dprefetcher=\"/L1Dprefetcher=\"$pref/" \
                -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                -e "s/enableFilter=True/enableFilter=False/" \
                $root/script/se > $targetFile
        chmod +x $targetFile
    done
}

genL2PPFEventQueueSizeTest() {
    echo ">> Generating test scripts for L2PPFEventQueueSizeTest..."
    targetDir=$root/test_script/l2_ppf_event_queue_size_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 16
    do
        conf=4
        # Pref NoUpgrade
        degree=16
        for queueSize in 2 8 -1
        do
            targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_queue_$queueSize
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/eventQueueSize=2/eventQueueSize=$queueSize/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
    done
}

genL2PPFForceTargetTest() {
    echo ">> Generating test scripts for L2PPFForceTargetTest..."
    targetDir=$root/test_script/l2_ppf_force_target_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1
    do
        conf=4
        # Pref NoUpgrade
        degree=16
        for forceTarget in 0 1 2
        do
            if [ $forceTarget = 0 ]
            then targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_dynamic
            else targetFile=$targetDir/task${taskNum}_${pref}_${conf}_${degree}_to_L$forceTarget
            fi
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                    -e "s/stridePrefDegree=16/stridePrefDegree=$degree/" \
                    -e "s/forceTarget=0/forceTarget=$forceTarget/" \
                    $root/script/se > $targetFile
            chmod +x $targetFile
        done
    done
}

genL2PPFPrefetcherTypeTest() {
    echo ">> Generating test scripts for L2PPFPrefetcherTypeTest..."
    targetDir=$root/test_script/l2_ppf_prefetcher_type_test
    rm -rf $targetDir
    mkdir -p $targetDir
    taskNum=1
    for pref in StridePrefetcher TaggedPrefetcher DCPTPrefetcher SBOOEPrefetcher IndirectMemoryPrefetcher SignaturePathPrefetcher SignaturePathPrefetcherV2 SlimAMPMPrefetcher AMPMPrefetcher
    do
        targetFile=$targetDir/task${taskNum}_${pref}_nofilter
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
                -e "s/enableFilter=True/enableFilter=False/" \
                $root/script/se > $targetFile
        chmod +x $targetFile
        targetFile=$targetDir/task${taskNum}_${pref}_withfilter
        sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
                $root/script/se > $targetFile
        chmod +x $targetFile
    done
}

genL2PPFPrefetcherConfigTest() {
    echo ">> Generating test scripts for L2PPFPrefetcherConfigTest..."
    targetDir=$root/test_script/l2_ppf_prefetcher_config_test
    rm -rf $targetDir
    mkdir -p $targetDir
    taskNum=1
    for l1pref in Null StridePrefetcher
    do
        for l2pref in Null StridePrefetcher
        do
            for  l3pref in Null StridePrefetcher
            do
                targetFile=$targetDir/task${taskNum}_${l1pref}_${l2pref}_${l3pref}_nofilter
                sed -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
                        -e "s/enableFilter=True/enableFilter=False/" \
                        $root/script/se > $targetFile
                if [ ${l1pref} != Null ]
                then sed -i "s/L1Dprefetcher=\"/L1Dprefetcher=\"$l1pref/" $targetFile
                fi
                if [ ${l2pref} != Null ]
                then sed -i "s/L2prefetcher=\"/L2prefetcher=\"$l2pref/" $targetFile
                fi
                if [ ${l3pref} != Null ]
                then sed -i "s/L3prefetcher=\"/L3prefetcher=\"$l3pref/" $targetFile
                fi
                chmod +x $targetFile
            done
        done
    done
    rm -rf $targetDir/task${taskNum}_Null_Null_Null_nofilter
    # l2pref+filter
    targetFile=$targetDir/task${taskNum}_Null_StridePrefetcher_Null_withl2filter
    sed -e "s/L2prefetcher=\"/L2prefetcher=\"StridePrefetcher/" \
            -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
            $root/script/se > $targetFile
    chmod +x $targetFile
    # l2pref+filter & l3pref
    targetFile=$targetDir/task${taskNum}_Null_StridePrefetcher_StridePrefetcher_withl2filter
    sed -e "s/useFilterL3=True/useFilterL3=False/" \
            -e "s/L2prefetcher=\"/L2prefetcher=\"StridePrefetcher/" \
            -e "s/L3prefetcher=\"/L3prefetcher=\"StridePrefetcher/" \
            -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
            $root/script/se > $targetFile
    chmod +x $targetFile
    # l2pref+filter & l3pref+filter
    targetFile=$targetDir/task${taskNum}_Null_StridePrefetcher_StridePrefetcher_withl2l3filter
    sed -e "s/L2prefetcher=\"/L2prefetcher=\"StridePrefetcher/" \
            -e "s/L3prefetcher=\"/L3prefetcher=\"StridePrefetcher/" \
            -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
            $root/script/se > $targetFile
    chmod +x $targetFile
    # l1pref & l2pref+filter & l3pref
    targetFile=$targetDir/task${taskNum}_StridePrefetcher_StridePrefetcher_StridePrefetcher_withl2filter
    sed -e "s/L1Dprefetcher=\"/L1Dprefetcher=\"StridePrefetcher/" \
            -e "s/L2prefetcher=\"/L2prefetcher=\"StridePrefetcher/" \
            -e "s/L3prefetcher=\"/L3prefetcher=\"StridePrefetcher/" \
            -e "s/useFilterL1D=True/useFilterL1D=False/" \
            -e "s/useFilterL3=True/useFilterL3=False/" \
            -e "s/stridePrefDegree=16/stridePrefDegree=16/" \
            $root/script/se > $targetFile
    chmod +x $targetFile
}

genL2PPFDebugTest
genL2PPFSimpleTest
genL2PPFFullTest
genL2PPFInferenceTest
genL2PPFTagBitsTest
genL2PPFOriginPPFTest
genL2PPFLevelUpTest
genL2PPFHarmTableTest
genL2PPFTableSizeTest
genL2PPFBestDegreeTest
genL2PPFL1PrefTest
genL2PPFEventQueueSizeTest
genL2PPFForceTargetTest
genL2PPFPrefetcherTypeTest
genL2PPFPrefetcherConfigTest
