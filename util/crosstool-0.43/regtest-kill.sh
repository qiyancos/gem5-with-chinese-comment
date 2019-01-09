#!/bin/sh
set -x
#ALLNODES="k8 fast fast2 dual2"

dn0=`dirname $0`
SOURCEDIR=`cd $dn0; pwd` 

ps augxw | egrep "regtest-run|rerun.sh" | grep -v kill | grep -v grep | awk '{print $2}' > procs
if test -s procs; then
  kill `cat procs`
  sleep 1
  ps augxw | egrep "regtest-run|rerun.sh" | grep -v kill | grep -v grep | awk '{print $2}' > procs
  if test -s procs; then
    kill -9 `cat procs`
  fi
fi
for node in $ALLNODES; do
  scp $SOURCEDIR/regtest-kill1.sh ${node}:/tmp &
done
wait
for node in $ALLNODES; do
  echo $node
  ssh -n -x -T $node "sh /tmp/regtest-kill1.sh"
done

ps augxw | grep "ssh.*jobdir.*running" | grep -v kill | grep -v grep | awk '{print $2}' > procs
if test -s procs; then
  kill `cat procs`
  sleep 1
  ps augxw | grep "ssh.*jobdir.*running" | grep -v kill | grep -v grep | awk '{print $2}' > procs
  if test -s procs; then
    kill -9 `cat procs`
  fi
fi

rm procs
