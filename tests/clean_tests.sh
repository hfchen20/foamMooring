#!/bin/bash

CWD=$(pwd)

for dir in * ; do
    if [ -d $dir ]; then
        cd $dir;
        ./Allclean
        cd $CWD
    fi
done
