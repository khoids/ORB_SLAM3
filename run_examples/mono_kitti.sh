#!bin/bash
if [ $(($1-1)) -lt 2 ]
then
    echo Sequence 00-02
    ./Examples/Monocular/mono_kitti ./Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI00-02.yaml ~/Datasets/KITTI/odometry_gray/dataset/sequences/$1
elif [ $(($1)) -eq 3 ]
then
    echo Sequence 03
    ./Examples/Monocular/mono_kitti ./Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI03.yaml ~/Datasets/KITTI/odometry_gray/dataset/sequences/$1
elif [ $(($1-1)) -lt 12 ]
then
    echo Sequence 04-12
    ./Examples/Monocular/mono_kitti ./Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI04-12.yaml ~/Datasets/KITTI/odometry_gray/dataset/sequences/$1
else
    echo Not found data sequence source
    echo $1
fi