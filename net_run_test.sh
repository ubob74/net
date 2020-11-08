#!/bin/bash

MODULE_NAME=net-test
FILE=$MODULE_NAME.ko

FILES="$FILE"

if [ $# -ne 2 ]; then
	printf "Usage: %s <ftp address> <test name>\n" $0
	exit 1;
fi

echo $1 $2

PREFIX=$2

for i in $FILES; do
	test -f $i && rm -f $i
	ftpget $1 $i $PREFIX/$i || exit 1
done

rmmod $MODULE_NAME
test -f $FILE && insmod $FILE || exit 1

