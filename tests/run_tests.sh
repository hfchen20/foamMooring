#!/bin/bash

RES_COL=60
MOVE_TO_COL="printf \\033[${RES_COL}G"

RED=$(tput setaf 1)
GREEN=$(tput setaf 2)
NORMAL=$(tput sgr0)

SETCOLOR_SUCCESS=$GREEN
SETCOLOR_FAILURE=$RED
SETCOLOR_NORMAL=$NORMAL

echo_success() {
  $MOVE_TO_COL
  printf "["
  printf $SETCOLOR_SUCCESS
  printf $"  OK  "
  printf $SETCOLOR_NORMAL
  printf "]"
  printf "\r"
  return 0
}

echo_failure() {
  $MOVE_TO_COL
  printf "["
  printf $SETCOLOR_FAILURE
  printf $"FAILED"
  printf $SETCOLOR_NORMAL
  printf "]"
  printf "\r"
  return 1
}

CWD=$(pwd)

for dir in * ; do
    if [ -d $dir ]; then
        cd $dir;
        printf "$dir "
        shift
        ./Allrun > ../$dir.log 2>&1
        retVal=$?
        if [ $retVal -ne 0 ]; then
            echo_failure $"$dir"
        else
            echo_success $"$dir"
        fi
        echo
        cd $CWD
    fi
done
