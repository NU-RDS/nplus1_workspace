from django.urls import path
from .views import move_arm

urlpatterns = [
    path("move/", move_arm, name="move_arm"),
]
