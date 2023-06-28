from django.urls import path

from .views import robots_view

urlpatterns = [
    #path('view_data/', view_data, name='view_data'),
    path('robots/', robots_view, name='robots_view'),
]