#!/bin/bash
IN_FILE_PATH=$1
OUT_FILE_PATH=$2
START_COL=$3
END_COL=$4
cat $FILE
cut -d, -f$START_COL-$END_COL $IN_FILE_PATH > "cut.tmp" && mv "cut.tmp" $OUT_FILE_PATH
