#!/bin/bash

num=6
model="iris"
world="race.world"

if [ "$1" == "prof" ]; then
  arg="--gazebo_ros"
elif [ "$1" == "nonprof" ]; then
  arg=""
else
  echo "$0 (prof | nonprof) [mc | vtol] [num] [world]"
  exit
fi

if [ "$2" == "vtol" ]; then
  model="standard_vtol"
fi

if [ "$3" != "" ]; then
  num=$3
fi

if [ "$4" != "" ]; then
  world=$4
fi

./start.sh $model $num gazebo/worlds/$world --pose_list places.txt $arg $@

if [ "$1" == "prof" ]; then
<<<<<<< HEAD
<<<<<<< HEAD
  ./bin/path_gen.py iris 6 race/centrals.txt race/test_ws/ 20 w1 w2 w3 w4 &
  ./examples/group.py
=======
  ./bin/path_gen.py $model $num race/centrals.txt race/test_ws/ 20 w1 w2 w3 w4 &
>>>>>>> f42e2f4040ad92a964feddd4d376feef905cd393
=======
  suf=""
  c=4

  if [ "$world" == "race_1.world" ]; then
    suf="_1"
    c=8
  fi

  walls=""
  for (( n=1 ; n<=$c; n++ ))
  do
    walls="${walls} w${n}"
  done

  ./bin/path_gen.py $model $num race/centrals${suf}.txt race/test_ws${suf}/ 20 $walls &
>>>>>>> beeae952b388f57516a269bcaed735ca2ff744df
fi
