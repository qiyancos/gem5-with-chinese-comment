#!/bin/sh
# Simple parallelized way to test building a given crosstool tarball
# on a bunch of machines connected via ssh 
# No NFS needed
# Copyright (C) 2005, Dan Kegel, Google
# License: GPL
#
# Requires that you have already used ssh-keygen, ssh-agent, ssh-add, etc. 
# to set up a no-prompt way of using ssh to run remote commands
# Creates $HOME/crosstooltest and $HOME/tarballs on all machines
# Accumulates results in directory 'jobdir' on current machine
# Run regtest-report.sh afterwards to generate nice HTML matrix of build results

set -x

# Run this command as 'nohup ssh-agent sh crosstool-0.41/regtest-run.sh'
#ssh-add

#rm -rf jobdir
mkdir -p jobdir

# Which version of crosstool to test
CROSSTOOL=crosstool-0.43

# Edit this line to specify the hosts to run the script on
#ALLNODES="k8 fast fast2"
ROLE=produser


WORKDIR=/home/produser/crosstool-regtest

# The ones tested by crosstool-0.31
TOOLS="\
gcc-2.95.3-glibc-2.1.3 \
gcc-2.95.3-glibc-2.2.5 \
gcc-3.3.4-glibc-2.2.5 \
gcc-3.3.4-glibc-2.3.2 \
gcc-3.3.4-glibc-2.3.3 \
gcc-3.4.3-glibc-2.2.5 \
gcc-3.4.3-glibc-2.3.2 \
gcc-3.4.3-glibc-2.3.3 \
gcc-3.4.3-glibc-2.3.4 \
gcc-4.0.0-glibc-2.2.5 \
gcc-4.0.0-glibc-2.3.3 \
gcc-4.0.0-glibc-2.3.4 \
"

# Edit this line to specify which toolchain combos to build
# Choice is somewhat arbitrary, especially as to which kernel headers to use
TOOLS="\
gcc-2.95.3-glibc-2.1.3 \
gcc-2.95.3-glibc-2.2.2 \
gcc-2.95.3-glibc-2.2.5 \
gcc-3.2.3-glibc-2.2.5 \
gcc-3.2.3-glibc-2.3.2 \
gcc-3.2.3-glibc-2.3.2-tls \
gcc-3.3.6-glibc-2.1.3 \
gcc-3.3.6-glibc-2.2.2 \
gcc-3.3.6-glibc-2.2.5 \
gcc-3.3.6-glibc-2.3.2 \
gcc-3.3.6-glibc-2.3.2-tls \
gcc-3.3.6-glibc-2.3.5 \
gcc-3.3.6-glibc-2.3.5-tls \
gcc-3.3.6-glibc-2.3.6 \
gcc-3.3.6-glibc-2.3.6-tls \
gcc-3.4.5-glibc-2.2.2 \
gcc-3.4.5-glibc-2.2.5 \
gcc-3.4.5-glibc-2.3.2 \
gcc-3.4.5-glibc-2.3.2-tls \
gcc-3.4.5-glibc-2.3.5 \
gcc-3.4.5-glibc-2.3.5-tls \
gcc-3.4.5-glibc-2.3.6 \
gcc-3.4.5-glibc-2.3.6-tls \
gcc-4.0.2-glibc-2.2.2 \
gcc-4.0.2-glibc-2.3.2 \
gcc-4.0.2-glibc-2.3.2-tls \
gcc-4.0.2-glibc-2.3.5 \
gcc-4.0.2-glibc-2.3.5-tls \
gcc-4.0.2-glibc-2.3.6 \
gcc-4.0.2-glibc-2.3.6-tls \
gcc-4.1.0-glibc-2.2.2 \
gcc-4.1.0-glibc-2.3.2 \
gcc-4.1.0-glibc-2.3.2-tls \
gcc-4.1.0-glibc-2.3.5 \
gcc-4.1.0-glibc-2.3.5-tls \
gcc-4.1.0-glibc-2.3.6 \
gcc-4.1.0-glibc-2.3.6-tls \
gcc-4.1.1-glibc-2.2.2 \
gcc-4.1.1-glibc-2.3.2 \
gcc-4.1.1-glibc-2.3.5 \
gcc-4.1.1-glibc-2.3.5-tls \
gcc-4.1.1-glibc-2.3.6 \
gcc-4.1.1-glibc-2.3.6-tls \
gcc-4.2-glibc-2.2.2 \
gcc-4.2-glibc-2.3.2 \
gcc-4.2-glibc-2.3.5 \
gcc-4.2-glibc-2.3.5-tls \
gcc-4.2-glibc-2.3.6 \
gcc-4.2-glibc-2.3.6-tls \
"


# Edit this line to specify which CPUs to build for
CPUS="\
i686 \
x86_64 \
alpha \
arm \
arm-iwmmxt \
arm-softfloat \
arm-xscale \
arm9tdmi \
armeb \
armv5b-softfloat \
ia64 \
m68k \
mips \
mipsel \
powerpc-405 \
powerpc-603 \
powerpc-750 \
powerpc-860 \
powerpc-970 \
s390 \
sh3 \
sh4 \
sparc \
sparc64 \
"

## Quick subset
#CPUS="\
#i686 \
#sh4 \
#x86_64 \
#"

for cpu in $CPUS; do
   for toolcombo in $TOOLS; do
       cat > jobdir/$cpu-$toolcombo.sh <<_EOF_
set -x
cd $CROSSTOOL
# FIXME: Remove this next line
export NO_DOWNLOAD=1
TARBALLS_DIR=$HOME/tarballs
export TARBALLS_DIR
RESULT_TOP=$HOME/crosstooltest
export RESULT_TOP
GCC_LANGUAGES=c,c++
export GCC_LANGUAGES
QUIET_EXTRACTIONS=1
export QUIET_EXTRACTIONS
mkdir -p \$RESULT_TOP
mkdir -p \$TARBALLS_DIR
PATH=/usr/local/gcc323/bin:$PATH
#if  awk '/bogomips/ {if (n < 2) n++}; END {print n}' < /proc/cpuinfo > cpus; then
#	PARALLELMFLAGS=-j\`cat cpus\`
#	export PARALLELMFLAGS
#fi
rm -rf build

# Only try to build gdb-6.3 for glibc-2.2.2 or newer, as it requires a function that
# first appeared in glibc-2.2.3.  (I backported the function to glibc-2.2.2.)
do_gdb=
case $toolcombo in
*glibc-2.[23]*|*glibc-200[4-9]*) do_gdb=--gdb ;;
esac

time eval \`cat $cpu.dat $toolcombo.dat\` sh all.sh --notest --testlinux \$do_gdb
_EOF_
   done
done

# usage: runjobs node tarball
# Unpacks the given tarball on the remote node, then runs jobs on the given node until none are left
# Should work whether or not the nodes share a common filesystem
# node may begin with username@ for use with ssh
runjobs() {
    if test $# != 2; then
        echo "usage: runjobs node tarball"
        exec /bin/false
    fi
    node=$1
    tarball=$2
    # glibc hates building in directories with @ in their name
    NODEDIR=`echo $WORKDIR/jobdir.$node | sed 's/@/_/g'`

    ssh -n -x -T $ROLE@$node "rm -rf $NODEDIR; mkdir -p $NODEDIR"
    nice scp $tarball $ROLE@${node}:$NODEDIR
    nice ssh -n -x -T $ROLE@$node "cd $NODEDIR; tar -xzvf $tarball"
    cd jobdir
    # can't do ls *.sh; that fails if too many processes do it at same time
    # because a file might be moved away between wildcard expansion and stat
    while job=`echo *.sh | sed 's/ .*//'`; do
        if test $job = '*.sh'; then
            break
        fi
	if mv $job $job.$node.running; then
            # set timestamp so ls -ltr *.running shows you how long they've been going
            touch $job.$node.running

	    base=`echo $job | sed 's/\.sh$//'`
            dat=$base.dat.txt
            log=$base.log.txt

	    echo Starting job $job on node $node
	    echo Starting job $job on node $node > $base.log
	    scp $job.$node.running $ROLE@${node}:$NODEDIR
	    nice ssh -n -x -T $ROLE@$node "cd $NODEDIR; sh $job.$node.running" >> $base.log 2>&1 || true

	    # Extract last few lines of log for posterity
            tail -150 $base.log > $log
            # Kludge: rerun job if ssh died, job ended with a segfault, or a suspicious I/O failure
            # (Maybe overclocking isn't worth it after all...)
            if egrep "Error 139|Segmentation fault|Illegal instruction|Connection to.*closed by remote host|final link failed: File truncated|write stdout: Input/output error|Connection refused|Trace/breakpoint trap|^Received disconnect" $log; then
	      echo Failed job $job on node $node, rerunning
	      echo Failed job $job on node $node, rerunning >> $base.log
	      mv $job.$node.running $job
              if egrep "Connection refused" $log; then
                  echo "runjobs $node offline, exiting"
                  return
              fi
            else
	      echo Finished job $job on node $node
	      echo Finished job $job on node $node >> $base.log
	      mv $job.$node.running $job.ran

              # Create $dat file with parameters of and results for this run
	      # Extract crosstool parameters from log (shouldn't we do this differently?)

	      head -3000 $base.log | sed '1,/Begin saving environment/d;/End saving environment/,$d' | \
                 egrep '^[A-Z_]*=.*' | sed 's/^.. echo //' | sort -u > $dat
	      # Extract whether build secceeded
              tresult=FAIL
              kresult=FAIL
              gdbresult=n/a
              gdbserverresult=n/a
	      if grep "testhello: C compiler can in fact build a trivial program" $base.log > /dev/null; then
	         tresult="PASS"
	      elif fgrep "internal compiler error" $base.log > /dev/null; then
	         tresult="ICE"
              fi
	      if grep "Linux kernel build done" $base.log > /dev/null; then
	         kresult="PASS"
	      elif test $tresult = PASS && fgrep "internal compiler error" $base.log > /dev/null; then
	         kresult="ICE"
	      fi
              grep "^crosstool:" $base.log > $base.log2
              if grep -e --gdb $base.log; then
                gdbresult=FAIL
                gdbserverresult=FAIL
                 if grep "gdb built ok" $base.log2; then
                   gdbresult=PASS
                fi
                if grep "gdbserver built ok" $base.log2; then
                   gdbserverresult=PASS
                fi
              fi

	      # Append success/failure data to param file
	      printf "toolchain=$tresult\nkernel=$kresult\ngdb=$gdbresult\ngdbserver=$gdbserverresult\n" >> $dat

              # compress full log file (else they really add up)
              gzip $base.log
            fi
	fi
	sleep 1
    done
}

for NODE in $ALLNODES; do
	runjobs $NODE $CROSSTOOL.tar.gz > $NODE.log 2>&1 &
done

time while ls jobdir/*.sh || ls jobdir/*.running; do
	sleep 10
done
wait

echo "all jobs done."
ls -l jobdir

