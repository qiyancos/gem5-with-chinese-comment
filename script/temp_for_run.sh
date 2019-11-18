# !/bin/bash
ARGN=$#
if [[ "$ARGN" < 1 ]]; then
        echo "input some parameter such 400"
        exit
fi

ARG=$*
for param in $ARG
do
        echo "runspec-simpoint.sh $param ref alpha"
        ./runspec-simpoint.sh $param ref alpha 
done
