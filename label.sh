#!/bin/bash
export LD_LIBRARY_PATH=":./"
echo $1

filepath=$(cd "$(dirname "$0")"; pwd)  
echo "$(basename $0) $(dirname $0) -- $filepath " 

file=$1

test=$filepath/$file
filelist=`ls $test`

#echo $filelist
for file in $filelist
do
	#echo $file
	if [ "${file#*.}" = "mp3" ];
	then
  	echo $file
  	./demo $test/$file
	fi
done


