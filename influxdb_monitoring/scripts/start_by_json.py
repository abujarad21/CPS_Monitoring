#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import rospkg
import sys
import argparse
import json
import subprocess
import time

def startupByJSON(filename):
    global session_name
    with open(filename) as json_file:
        data = json.load(json_file)
    try:
        session_name = data["session_name"]
        tmux = data["tmux"]
        source = data["source"]
    except:
        print("Could not read session name/tmux/source from file %s" % filename)
        return False

    # start tmux session
    subprocess.run(["%s.sh" % filename, tmux, session_name, source])
    # create windows

    try:
        for window_name in data["windows"]:
            print("%s: %s" % (window_name, data["windows"][window_name]["start_cmd"]))
            start_args = [tmux,  "send-keys", "-t", "%s:%s" % (session_name, window_name), data["windows"][window_name]["start_cmd"], "C-m"]
            #print(start_args)
            subprocess.run(start_args)
            time.sleep(float(data["windows"][window_name]["delay"]))
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        print("failed during subprocess calling")
        return False
    print("All good during start")
    return True



def main():
    """
    Entry point for the demo script.
    """

    parser = argparse.ArgumentParser(
                    prog = 'ProgramName',
                    description = 'What the program does',
                    epilog = 'Text at the bottom of help')
    parser.add_argument("--startConfig", type=str, default="config/node_config_example.json")
    parser.add_argument("--filePkg", type=str, default="influxdb_monitoring")
    parser.add_argument("--isSimulation", type=bool, default=False)
    args, unknown = parser.parse_known_args()
    try:
        rospack = rospkg.RosPack()
        package_dir = rospack.get_path(args.filePkg)
        full_filename = package_dir + "/" + args.startConfig
        print(full_filename)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        print("error while parsing pose of interest file (--startConfig and --filePkg)")
        return

    if (not startupByJSON(full_filename)):
        return

    if (args.isSimulation):
        print("is simulation")

if __name__ == '__main__':
  main()
