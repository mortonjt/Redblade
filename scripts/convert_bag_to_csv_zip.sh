#!/bin/bash

bag_path=$1

for file in `ls $bag_path`; do

    if [[ "$file" == *.bag ]]; then

	filename=${file%.bag}

	echo "$file"
	echo "$filename"

	python `rospack find rosbag_parsers`/src/parse_odometry_tests.py --input_rosbag=$bag_path$file --out=$bag_path$filename

	zip $bag_path$filename $bag_path*.csv

	rm $bag_path*.csv

    fi

done

