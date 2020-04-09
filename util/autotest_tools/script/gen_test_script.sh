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

genL1SizeTest() {
    echo ">> Generating test scripts for L1SizeTest..."
    targetDir=$root/test_script/l1_size_test
    rm -rf $targetDir
    mkdir -p $targetDir
    for l1dsize in $L1DSizeList
    do
        sed "s/L1Dsize=\"32kB/L1Dsize=\"$l1dsize/" $root/script/se \
                > $targetDir/task16_$l1dsize
        chmod +x $targetDir/task16_$l1dsize
    done
}

genL2SizeTest() {
    echo ">> Generating test scripts for L2SizeTest..."
    targetDir=$root/test_script/l2_size_test
    rm -rf $targetDir
    mkdir -p $targetDir
    for l2size in $L2SizeList
    do
        sed "s/L2size=\"256kB/L2size=\"$l2size/" $root/script/se \
                > $targetDir/task16_$l2size
        chmod +x $targetDir/task16_$l2size
    done
}

genL3SizeTest() {
    echo ">> Generating test scripts for L3SizeTest..."
    targetDir=$root/test_script/l3_size_test
    rm -rf $targetDir
    mkdir -p $targetDir
    for l3size in $L3SizeList
    do
        sed "s/L3size=\"8MB/L3size=\"$l3size/" $root/script/se \
                > $targetDir/task16_$l3size
        chmod +x $targetDir/task16_$l3size
    done
}

genL1PrefAgressiveTest() {
    echo ">> Generating test scripts for L1PrefAgressiveTest..."
    targetDir=$root/test_script/l1_pref_agressive_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in $TaskCountList
    do
        for conf in $StridePrefConfThreshList
        do
            for degree in $StridePrefDegreeList
            do
                sed -e "s/L1Dprefetcher=\"/L1Dprefetcher=\"$pref/" \
                        -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                        -e "s/stridePrefDegree=4/stridePrefDegree=$degree/" \
                        $root/script/se > \
                        $targetDir/task${taskNum}_${pref}_${conf}_${degree}
                chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            done
        done
    done
}

genL2PrefAgressiveTest() {
    echo ">> Generating test scripts for L2PrefAgressiveTest..."
    targetDir=$root/test_script/l2_pref_agressive_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in $TaskCountList
    do
        for conf in $StridePrefConfThreshList
        do
            for degree in $StridePrefDegreeList
            do
                sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                        -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                        -e "s/stridePrefDegree=4/stridePrefDegree=$degree/" \
                        $root/script/se > \
                        $targetDir/task${taskNum}_${pref}_${conf}_${degree}
                chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            done
        done
    done
}

genL3PrefAgressiveTest() {
    echo ">> Generating test scripts for L3PrefAgressiveTest..."
    targetDir=$root/test_script/l3_pref_agressive_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in $TaskCountList
    do
        for conf in $StridePrefConfThreshList
        do
            for degree in $StridePrefDegreeList
            do
                sed -e "s/L3prefetcher=\"/L3prefetcher=\"$pref/" \
                        -e "s/stridePrefConfThresh=4/stridePrefConfThresh=$conf/" \
                        -e "s/stridePrefDegree=4/stridePrefDegree=$degree/" \
                        $root/script/se > \
                        $targetDir/task${taskNum}_${pref}_${conf}_${degree}
                chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            done
        done
    done
}

genL1PrefTypeTest() {
    echo ">> Generating test scripts for L1PrefTypeTest..."
    targetDir=$root/test_script/l1_pref_type_test
    rm -rf $targetDir
    mkdir -p $targetDir
    for taskNum in $TaskCountList
    do
        for pref in $PrefetcherList
        do
            sed -e "s/L1Dprefetcher=\"/L1Dprefetcher=\"$pref/" $root/script/se > \
                    $targetDir/task${taskNum}_${pref}
            chmod +x $targetDir/task${taskNum}_${pref}
        done
    done
}

genL2PrefTypeTest() {
    echo ">> Generating test scripts for L2PrefTypeTest..."
    targetDir=$root/test_script/l2_pref_type_test
    rm -rf $targetDir
    mkdir -p $targetDir
    for taskNum in $TaskCountList
    do
        for pref in $PrefetcherList
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" $root/script/se > \
                    $targetDir/task${taskNum}_${pref}
            chmod +x $targetDir/task${taskNum}_${pref}
        done
    done
}

genL3PrefTypeTest() {
    echo ">> Generating test scripts for L3PrefTypeTest..."
    targetDir=$root/test_script/l3_pref_type_test
    rm -rf $targetDir
    mkdir -p $targetDir
    for taskNum in $TaskCountList
    do
        for pref in $PrefetcherList
        do
            sed -e "s/L3prefetcher=\"/L3prefetcher=\"$pref/" $root/script/se > \
                    $targetDir/task${taskNum}_${pref}
            chmod +x $targetDir/task${taskNum}_${pref}
        done
    done
}

genL2PrefSimpleTest() {
    echo ">> Generating test scripts for L2PrefSimpleTest..."
    targetDir=$root/test_script/l2_pref_simple_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in $TaskCountList
    do
        conf=4
        for degree in 8 16
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefDegree=4/stridePrefDegree=$degree/" \
                    $root/script/se > \
                    $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            cp $root/script/se $targetDir/task${taskNum}_nopref
            chmod +x $targetDir/task${taskNum}_nopref
        done
        
    done
}

genL2PrefBestDegreeTest() {
    echo ">> Generating test scripts for L2PrefBestDegreeTest..."
    targetDir=$root/test_script/l2_pref_best_degree_test
    rm -rf $targetDir
    mkdir -p $targetDir
    pref="StridePrefetcher"
    for taskNum in 1 2 4 6 8 10 12 14 16 # $TaskCountList
    do
        conf=4
        for degree in 0 4 8 12 16 20 24 28 32
        do
            sed -e "s/L2prefetcher=\"/L2prefetcher=\"$pref/" \
                    -e "s/stridePrefDegree=4/stridePrefDegree=$degree/" \
                    $root/script/se > \
                    $targetDir/task${taskNum}_${pref}_${conf}_${degree}
            chmod +x $targetDir/task${taskNum}_${pref}_${conf}_${degree}
        done
    done
}

genL1SizeTest
genL2SizeTest
genL3SizeTest
genL1PrefAgressiveTest
genL2PrefAgressiveTest
genL3PrefAgressiveTest
genL1PrefTypeTest
genL2PrefTypeTest
genL3PrefTypeTest
genL2PrefSimpleTest
genL2PrefBestDegreeTest
