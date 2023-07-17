#!/bin/bash

# copying the module files there allows VSCode to find the files and supply code completion. 
# If you find a better way to do this (should work for all, not just add to vscode path), please contact me.

MODULE_PATH="$(rospack find influxdb_monitoring)/include/influxdb_monitoring"
DEVEL_PATH="$(rospack find influxdb_monitoring)/../../devel/lib/python3/dist-packages/influxdb_monitoring"
echo $MODULE_PATH
echo $DEVEL_PATH

cd $MODULE_PATH
cp `ls -I __init__.py -I __pycache__` $DEVEL_PATH