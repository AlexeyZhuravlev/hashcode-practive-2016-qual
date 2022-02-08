#!/bin/bash

if [ $# -ne 2 ]; then
	for filename in ./test/*.in; do
        echo "===================="
		echo "Running on $filename"
		./test/$1 < $filename > ./test/$(basename "$filename" .in).ans
	done
else
	./test/$1 < ./test/$2.in > ./test/$2.ans
fi

mkdir tmp
cp ./src/$1.cpp tmp
cp ./include/common.h tmp
cd tmp
tar -cvf ../test/"$1.tar" $1.cpp common.h
cd ..
rm -r tmp
