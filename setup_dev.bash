#!/usr/bin/env bash

PATHPLANNING_REPO_PATH=$(cd -- $(dirname -- $0) && pwd)
export PYTHONPATH=$PYTHONPATH:$PATHPLANNING_REPO_PATH/python
export MAP_DATA=$PATHPLANNING_REPO_PATH/data/highway_map.csv

echo exported PYTHONPATH=$PYTHONPATH
echo exported MAP_DATA=$MAP_DATA
