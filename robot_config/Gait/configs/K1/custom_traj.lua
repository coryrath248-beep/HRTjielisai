kp_base = {
    10., 10.,
    150., 150., 150., 150.,
    150., 150., 150., 150.,
    350., 350., 180., 350., 250., 250.,
    350., 350., 180., 350., 250., 250.,
}
kd_base = {
    0.5, 0.5,
    2., 2., 2., 2., 
    2., 2., 2., 2., 
    3., 3., 3., 3., 1.3, 1.3,
    3., 3., 3., 3., 1.3, 1.3,
}
joint_zeros = {
    0., 0., 
    0., 0., 0., 0.,
    0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
}

all_zero_traj = {
    start_from_cur_state = true,
    seq_len = 1,
    frame_1 = {
        p = joint_zeros,
        kp = kp_base,
        kd = kd_base,
        torq = joint_zeros,
        dur = 1.0,
    },
}

commander_conf = {
    n_joint = 22,
    n_traj = 4,
    traj_1 = all_zero_traj,
    traj_2 = all_zero_traj,
    traj_3 = all_zero_traj,
    traj_4 = all_zero_traj,
}

return commander_conf
