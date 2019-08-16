#!/bin/bash

detector_set=("HARRIS" "FAST" "BRISK" "ORB" "AKAZE" "SIFT" "SHITOMASI")
descriptor_set=("BRIEF" "ORB" "FREAK" "AKAZE" "SIFT" "BRISK")

cd  build/

for detector in ${detector_set[@]};
do
    for descriptor in ${descriptor_set[@]};
    do
        ./3D_object_tracking $detector $descriptor
    done
done
