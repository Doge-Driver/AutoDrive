from . import PurePursuit


def velocity(pathPoints):
    return PurePursuit.velocity_plan(pathPoints) * 1000


def steering(pathPoints):
    return PurePursuit.steering_angle(pathPoints)
