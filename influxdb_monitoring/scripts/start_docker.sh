#!/bin/bash

#DOCKER_PATH=$HOME/dockerstuff
DOCKER_PATH=/media/data/monitoring_system
echo $DOCKER_PATH
echo "DOCKER_PATH=$DOCKER_PATH" > $DOCKER_PATH/.env
echo "UID=1000" >> $DOCKER_PATH/.env
echo "GID=1000" >> $DOCKER_PATH/.env


echo "Requires docker compose"
echo "https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository"
source ~/tedusar_ws/devel/setup.bash

mkdir -p $DOCKER_PATH/appdata
roscd influxdb_monitoring/docker
cp telegraf.conf ~/dockerstuff/appdata/

sudo docker compose --env-file $DOCKER_PATH/.env up -d