from __future__ import absolute_import, unicode_literals
from celery import shared_task
from django.core.management import call_command
import time
@shared_task
def task_transfer_data():
    call_command('transfer_data')
    time.sleep(10)  # Delay the task execution for 10 seconds
