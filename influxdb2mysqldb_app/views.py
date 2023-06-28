from django.shortcuts import render
from .models import Robots, ScanTable, OdomTable
from django.db.models import Max
import pandas as pd
import pytz


def robots_view(request):
    robots = Robots.objects.all()
    robot_data = []
    for robot in robots:
        odom = OdomTable.objects.filter(robot_id=robot.id).latest('timestamp')
        scan = ScanTable.objects.filter(robot_id=robot.id).latest('timestamp')
        robot_data.append((robot, odom, scan))
    print(robot_data)  # Debug line
    return render(request, 'robots_view.html', {'robot_data': robot_data})
# def view_data(request):
#     robot_name = "my_robot"  # or any robot name you are interested in
#     robot = Robots.objects.get(name=robot_name)
#     robot_id = robot.id
#     scans = ScanTable.objects.filter(robot_id=robot_id).order_by('-timestamp').values()
#     odoms = OdomTable.objects.filter(robot_id=robot_id).order_by('-timestamp').values()

#     # convert to pandas dataframes
#     scans_df = pd.DataFrame.from_records(scans)
#     odoms_df = pd.DataFrame.from_records(odoms)


#     # merge dataframes on timestamp
#     merged_df = pd.merge(scans_df, odoms_df, on='timestamp', how='outer')
#     local_tz = pytz.timezone('Asia/Kuala_Lumpur')  # Replace 'Your_Time_Zone' with the desired time zone
#     merged_df['timestamp'] = merged_df['timestamp'].dt.tz_convert(local_tz)

#     # convert back to list of dicts
#     merged_data = merged_df.to_dict('records')
#     # convert back to list of dicts
#     merged_data = merged_df.to_dict('records')

#     # format timestamp and sensor values
#     for record in merged_data:
#         record['timestamp'] = record['timestamp'].strftime('%B %d, %Y, %I:%M:%S %p')
#         record['a_min'] = round(record['a_min'], 2)
#         record['a_max'] = round(record['a_max'], 2)
#         record['x'] = round(record['x'], 2)
#         record['y'] = round(record['y'], 2)
#         record['z'] = round(record['z'], 2)

#     context = {
#         'robot_id': robot_id,
#         'robot_name': robot_name,
#         'merged_data': merged_data,
#     }
#     return render(request, 'page.html', context)

# def view_data(request):
#     robot_name = "my_robot"  # or any robot name you are interested in
#     robot = Robots.objects.get(name=robot_name)
#     robot_id = 1
#     scans = ScanTable.objects.filter(robot_id=robot_id).order_by('-timestamp').values()
#     odoms = OdomTable.objects.filter(robot_id=robot_id).order_by('-timestamp').values()

#     # convert to pandas dataframes
#     scans_df = pd.DataFrame.from_records(scans)
#     odoms_df = pd.DataFrame.from_records(odoms)

#     # merge dataframes on timestamp
#     merged_df = pd.merge(scans_df, odoms_df, on='timestamp', how='outer')

#     # convert back to list of dicts
#     merged_data = merged_df.to_dict('records')

#     context = {
#         'robot_id': robot.id,
#         'robot_name': robot.name,
#         'merged_data': merged_data,
#     }
#     return render(request, 'page.html', context)