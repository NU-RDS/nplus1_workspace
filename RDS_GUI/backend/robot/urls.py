from django.urls import path
from . import views

urlpatterns = [
    path("finger_angles/", views.get_finger_angles, name="finger_angles"),
    path("move/", views.move_finger, name="move_finger"),
    path("video_feed/", views.video_feed, name="video_feed"),
    path("stop_video_feed/", views.stop_video_feed, name="stop_video_feed"),
]
