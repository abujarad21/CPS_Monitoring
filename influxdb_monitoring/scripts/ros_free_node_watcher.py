#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import os
import psutil
import time
import subprocess
import influxdb_monitoring.NodeMonitor as NodeMonitor
import argparse



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                    prog = 'ROS free node watcher',
                    description = 'Restarts ROS stack, when a monitored process dies.',
                    epilog = 'Text at the bottom of help')
    parser.add_argument("--startScript", type=str, default="start_monitoring_example.sh")
    parser.add_argument("--startConfig", type=str, default="config/node_config_example.json")
    parser.add_argument("--filePkg", type=str, default="influxdb_monitoring")
    parser.add_argument("--sessionName", type=str, default="robot_session")
    args, unknown = parser.parse_known_args()

    session_name = args.sessionName
    file_pkg = args.filePkg
    start_file = args.startScript
    start_config = args.startConfig

    print(session_name)
    print(file_pkg)
    print(start_file)


    pid_to_watch = -1
    last_running = time.time()
    wait_inbetween_checks = 1
    last_pid_to_watch = 0
    reported_death = False
    process_running = False
    successive_restart_count = 0
    started_running = time.time()
    while True:
        try:
            f = open(os.path.expanduser("~/.node_watcher_pid"), "r")
            pid_to_watch = int(f.read())
            if (pid_to_watch != last_pid_to_watch):
                print("Watching new pid: %d" % pid_to_watch)
                last_pid_to_watch = pid_to_watch

        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            print("no pid found..")
        
        while (psutil.pid_exists(pid_to_watch)):
            if (not process_running):
                started_running = time.time()
            if (time.time() - started_running > 15):
                successive_restart_count = 0
            process_running = True
            reported_death = False
            
            last_running = time.time()
            time.sleep(wait_inbetween_checks)

        if (last_running + wait_inbetween_checks + 5 < time.time()):
            if (not reported_death):
                print("Process died!")
                reported_death = True
            unresponsive_for = time.time()-last_running
            if (unresponsive_for > 30):
                print("Node watcher unresponsive for more than 30s, restarting (%d)" % successive_restart_count)
                if (successive_restart_count > 2):
                    print("could not restart after %d tries, assuming roscore failed. Restarting whole stack!" % successive_restart_count)
                    #${TMUX} send-keys -t ${SESSION_NAME}:${WINDOW}.1 "${WINDOW}.1"
                    try:
                        start_args = ["byobu", "kill-session", "-t", session_name]
                        subprocess.run(start_args)
                        time.sleep(10)

                        start_args = ["rosrun", file_pkg, start_file]
                        subprocess.run(start_args)
                    except KeyboardInterrupt:
                        raise KeyboardInterrupt
                    except:
                        print("failed during subprocess calling")
                    
                else:
                    try:
                        NodeMonitor.load_stuff(skip_parse=True, start_file_=start_config, file_pkg_=file_pkg)
                        NodeMonitor.restartNode("/nodeMonitorWatcher")
                    except KeyboardInterrupt:
                        raise KeyboardInterrupt
                    except Exception as e:
                        print(e)
                successive_restart_count += 1
                time.sleep(5)
            time.sleep(5)

        process_running = False
