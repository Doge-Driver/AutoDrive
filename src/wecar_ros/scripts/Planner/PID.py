p_gain = 0.91
i_gain = 0.05
d_gain = 0.07
controlTime = 0.033
prev_error = 0
i = 0
angle_list = [0.0, 0.0, 0.0]


def pid(target_vel, current_vel, time_measure):
    error = target_vel - current_vel
    angle_list.append(error)
    del angle_list[0]
    i_error = simpson(angle_list, time_measure)
    p = p_gain * error
    # i+=i_gain*error*controlTime
    i += i_gain * i_error
    d = d_gain * (error - prev_error) / controlTime

    output = p + i + d
    prev_error = error
    return output


def simpson(angles, h):
    return h / 3 * (angles[0] + 4 * angles[1] + angles[2])
