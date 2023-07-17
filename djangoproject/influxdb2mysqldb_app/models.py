from django.db import models

class Robots(models.Model):
    class Meta:
        db_table = 'robots'

    name = models.CharField(max_length=200, unique=True)
    model = models.CharField(max_length=200, null=True)  # new field for the model of the robot
    ip_address = models.CharField(max_length=200, null=True)  # new field for the IP address of the robot


class OdomTable(models.Model):
    class Meta:
        db_table = 'odom_table'
    
    robot = models.ForeignKey(Robots, related_name="odom_entries", on_delete=models.CASCADE, null=True)
    timestamp = models.DateTimeField()
    x = models.FloatField(null=True)
    y = models.FloatField(null=True)
    z = models.FloatField(null=True)

class ScanTable(models.Model):
    class Meta:
        db_table = 'scan_table'
    
    robot = models.ForeignKey(Robots, related_name="scan_entries", on_delete=models.CASCADE, null=True)
    timestamp = models.DateTimeField()
    a_min = models.FloatField(null=True)
    a_max = models.FloatField(null=True)