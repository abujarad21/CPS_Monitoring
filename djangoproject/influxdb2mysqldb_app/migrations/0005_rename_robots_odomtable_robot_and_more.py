# Generated by Django 4.2.2 on 2023-06-23 08:15

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('influxdb2mysqldb_app', '0004_alter_odomtable_table_alter_robots_table_and_more'),
    ]

    operations = [
        migrations.RenameField(
            model_name='odomtable',
            old_name='robots',
            new_name='robot',
        ),
        migrations.RenameField(
            model_name='scantable',
            old_name='robots',
            new_name='robot',
        ),
    ]
