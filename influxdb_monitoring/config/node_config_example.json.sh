#!/bin/bash

# PARAMETERS
TMUX=$1 #Options: tmux | byobu sudo -H
SESSION_NAME=$2
SOURCE=$3

cd ~
source $SOURCE

# Script
${TMUX} has-session -t ${SESSION_NAME}

if [ $? = 0 ]
then
  echo "Session ${SESSION_NAME} already exists. Killing it..."
  byobu kill-session -t $SESSION_NAME
fi

# Create the session
echo "Creating a new session ${SESSION_NAME}..."
${TMUX} new-session -s ${SESSION_NAME} -n monitoring -d
sleep 1s


WINDOW=0
${TMUX} split-window -h -t ${SESSION_NAME}:${WINDOW}    # Split window horizontally (new window becomes x.1)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.0  # Split window x.0 vertically (new window becomes x.1. - old window x.1 becomes x.2)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.2  # Split window x.2 vertically (new window becomes x.3)


${TMUX} new-window -n nodes1 -t ${SESSION_NAME}
WINDOW=1
${TMUX} split-window -h -t ${SESSION_NAME}:${WINDOW}    # Split window horizontally (new window becomes x.1)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.0  # Split window x.0 vertically (new window becomes x.1. - old window x.1 becomes x.2)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.2  # Split window x.2 vertically (new window becomes x.3)


${TMUX} new-window -n nodes2 -t ${SESSION_NAME}
WINDOW=2
${TMUX} split-window -h -t ${SESSION_NAME}:${WINDOW}    # Split window horizontally (new window becomes x.1)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.0  # Split window x.0 vertically (new window becomes x.1. - old window x.1 becomes x.2)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.2  # Split window x.2 vertically (new window becomes x.3)


${TMUX} new-window -n nodes2 -t ${SESSION_NAME}
WINDOW=3
${TMUX} split-window -h -t ${SESSION_NAME}:${WINDOW}    # Split window horizontally (new window becomes x.1)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.0  # Split window x.0 vertically (new window becomes x.1. - old window x.1 becomes x.2)
${TMUX} split-window -v -t ${SESSION_NAME}:${WINDOW}.2  # Split window x.2 vertically (new window becomes x.3)