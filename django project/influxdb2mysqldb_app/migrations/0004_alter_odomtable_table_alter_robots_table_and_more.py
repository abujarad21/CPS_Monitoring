# Generated by Django 4.2.2 on 2023-06-23 07:51

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('influxdb2mysqldb_app', '0003_alter_odomtable_table_alter_robots_table_and_more'),
    ]

    operations = [
        migrations.AlterModelTable(
            name='odomtable',
            table='odom_table',
        ),
        migrations.AlterModelTable(
            name='robots',
            table='robots',
        ),
        migrations.AlterModelTable(
            name='scantable',
            table='scan_table',
        ),
    ]