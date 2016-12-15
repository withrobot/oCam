#!/bin/bash

# build
echo ""
echo "------------------------ ${PWD##*/} build start. -------------------------"
make clean 
res=$?

if [ $res -eq 0 ]; then
make -j8
res=$?
fi

if [ $res -eq 0 ]; then
echo "------------------------ ${PWD##*/} build finished. -------------------------"
echo ""
else
echo "------------------------ ${PWD##*/} build failed. -------------------------"
echo ""
fi

# run
if [ $res -eq 0 ]; then
./build/bin/${PWD##*/} $1
fi
