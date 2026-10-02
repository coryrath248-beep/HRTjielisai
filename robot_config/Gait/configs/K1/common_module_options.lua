package.path = package.path .. ";../src/tests/?.lua"

controller_base_dt_ms = 2

kp_test = {
    -- on ground
    70., 70.,
    40., 50., 10., 10.,
    40., 50., 10., 10.,
    350., 350., 180., 350., 250., 250.,
    350., 350., 180., 350., 250., 250.,

    -- on air
    -- 5., 5.,
    -- 40., 50., 10., 10.,
    -- 40., 50., 10., 10.,
    -- 250., 250., 180., 250., 100., 100.,
    -- 250., 250., 180., 250., 100., 100.,

    -- 0., 0.,
    -- 0., 0., 0., 0., 
    -- 0., 0., 0., 0., 
    -- 0., 0., 0., 0., 0., 0.,
    -- 0., 0., 0., 0., 0., 0.,
}

kd_test = {
    -- on ground
    1.5, 1.5,
    .5, 1.5, .2, .2,
    .5, 1.5, .2, .2,
    7.5, 7.5, 3., 5.5, 0.5, 0.5,
    7.5, 7.5, 3., 5.5, 0.5, 0.5,

    -- on air
    -- .1, .1,
    -- .5, 1.5, .2, .2,
    -- .5, 1.5, .2, .2,
    -- 5.5, 3., 3., 5.5, .2, .2,
    -- 5.5, 3., 3., 5.5, .2, .2,

    -- 0., 0.,
    -- 0., 0., 0., 0., 
    -- 0., 0., 0., 0., 
    -- 0., 0., 0., 0., 0., 0.,
    -- 0., 0., 0., 0., 0., 0.,
}

ready_pos_test = {
    0.00,  0.00,
    0.2, -1.45, 0.0, -0.5,
    0.2,  1.45, 0.0,  0.5,
    -0.0,  0.0,  0.0,  0.105,  0.08, 0.07,
    -0.0,  0.0,  0.0,  0.105,  0.08, 0.07,
    -- -0.1,  0.0,  0.0,  0.2,  0.07, 0.06,
    -- -0.1,  0.0,  0.0,  0.2,  0.07, 0.06, 

    -- -0.0,  0.0,  0.0,  0.0,  0.0, 0.0,
    -- -0.0,  0.0,  0.0,  0.0,  0.0, 0.0, 
}

local options = {
    simulator_out = {
        is_real_bot_ = true,
        is_imu_rotated_ = false,
        record_data = true,

        proto_index_imu_data_ = 3,
        proto_index_joint_num_ = 7,
        proto_index_joint_val_ = 6,
        proto_index_remote_data_ = 4,
        num_motor_ = 22,        
        
        max_lin_vel_x_ = 0.25,
        max_lin_vel_y_ = 0.07,
        max_rot_vel_z_ = 1.3,
    },

    simulator_in = {
        record_data = true,

        step_len_ = controller_base_dt_ms, 
        proto_index_joint_cmd_ = 5,
        use_pvt_ = true,
        num_motor_ = 22,
        motor_max_torque_value_ = {
            8.0, 8.0,
            14.0, 14.0, 14.0, 14.0, 
            14.0, 14.0, 14.0, 14.0, 
            60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
            60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
        },
        motor_max_position_value_ = {
            3.14159, 3.14159, 
            3.14159, 3.14159, 3.14159, 3.14159,
            3.14159, 3.14159, 3.14159, 3.14159,
            3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159,
            3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159,
        },
    },

    action_hub = {
        -- used by head action 
        motor_cmd_diff_max = 0.4,
        low_cmd_kp_from_high_api = 60.0,
        low_cmd_kd_from_high_api = 1.5,
        head_yaw_cmd_speed = 0.005,
        head_pitch_cmd_speed = 0.005,
        head_yaw_max = 1.0472,
        head_yaw_min = -1.0472,
        head_pitch_max = 0.8552,
        head_pitch_min = -0.3491,
        
        -- disbaled biped planner in K1
        disable_biped_planner = false,

        record_data_ = false,
        ori_offset_left ={0., 0., 1., 1., 0., 0., 0., 1., 0.,},
        ori_offset_right ={0., 0., 1., -1., 0., 0., 0., -1., 0.,},
        kp_joint = {
            70.0, 70.0,
            220., 220., 180., 180.,
            220., 220., 180., 180.,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
        },
        kd_joint = {
            1.5, 1.5,
            5.0, 5.0, 5.0, 5.0,
            5.0, 5.0, 5.0, 5.0,
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
        },
        kp_joint_zero_t_ = {
            0.0, 0.0,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
        },
        kd_joint_zero_t_ = {
            1.05, 1.05,
            0.3, 0.3, 0.2, 0.2,
            0.3, 0.3, 0.2, 0.2,
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
        },
        urdf_path = "urdf_path",
        ee_name = {
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        constraint_type_names = {
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
        },
        constraint_body_names = {
            "left_forearm_pitch_link",
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
            "right_forearm_pitch_link",
        },

        constratint_weight = {1.0, 0.02, 1.0, 0.02},
        ee_point = {
            -0.012,  0.213,  0.000,
            -0.012, -0.213,  0.000,
        },
        q_min = {
        -1.6, -0.4,
        -2.9671, -1.4835, -2.2689, -2.1817,
        -2.9671, -1.9199, -2.2689, -2.1817,
        -3.1416, -0.5236, -1.6472, -0.200, -1.8727, -1.4363,
        -3.1416, -1.5708, -1.6472, -0.200, -1.8727, -1.4363,
        },
        q_max = {
         1.6,  1.6,
         1.2217,  1.9199,  2.2689,  2.1817,
         1.2217,  1.4835,  2.2689,  2.1817,
         3.1416,  1.5708,  1.0472,  2.3387,  1.3491,  1.4363,
         3.1416,  0.5236,  1.0472,  2.3387,  1.3491,  1.4363,
        },
        q_better_guess = {
         0.00,  0.00,
         0.25, -1.40,  0.00, -0.50,
         0.25,  1.40,  0.00,  0.50,
        -0.40, -0.03,  0.05,  0.70, -0.35,  0.03,
        -0.40,  0.03, -0.05,  0.70, -0.35, -0.03,
        },
        arm_start_index = 2,
        arm_num = 4,
        max_steps = 100,
        step_tol = 1.0e-4,
        lambda = 1.0e-3,
        record_traj_data_ = true;
        saved_data_path_ = "",
        saved_data_path_list_ = {"../configs/K1/dance_new_year_k1.txt",
                                 "../configs/K1/dance_boxing_k1.txt",
                                 "../configs/K1/dance_nezha_k1.txt",
                                 "../configs/K1/dance_towards_future_k1.txt",
                                 "../configs/K1/gesture_maneki_neko_k1.txt",
                                 "../configs/K1/gesture_pogba_k1.txt",
                                 "../configs/K1/gesture_ultraman_k1.txt",
                                 "../configs/K1/gesture_chinese_greet_k1.txt",
                                 "../configs/K1/gesture_greet_k1.txt"
                                 },

        -- used by custom upper body control action
        upper_body_num_joint = 10,
        q_min_custom = {
            -1.0472, -0.3491,
            -3.3161, -1.6755, -2.2689, -2.1293,
            -3.3161, -1.5708, -2.2689, -0.1745,
            -3.0020, -0.4014, -1.0472, -0.0000, -1.8727, -1.3491,
            -3.0020, -1.5708, -1.0472, -0.0000, -1.8727, -1.3491,
        },
        q_max_custom = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689, 0.1745,
            1.2217,  1.6755,  2.2689, 2.1293,
            2.2166,  1.5708,  1.0472,  2.2340,  1.3491,  1.3491,
            2.2166,  0.4014,  1.0472,  2.2340,  1.3491,  1.3491,
        },
    },

    robot_state_manager = {
        use_config = true,
        disable_part_commands = false,
        enable_remote_controller = true,
        robocup_enable = true,
        wbc_gait_enable = true,
        command_config_path = "../../booster_config/robot_config/kidsize/K1/configurations/task_instruction.yaml",

        urdf_path = "urdf_path",
        hand_action_enable = true,

        vel_x_max = 1.0, -- 1.2,
        vel_y_max = 1.0, -- 0.6,
        rotvel_max = 1.0, -- 2.0,

        vel_x_max_run = 1.0, -- 1.25, -- 0.5,
        vel_y_max_run = 1.0, -- 0.6, -- 0.3,
        rotvel_max_run = 1.0, -- 2.0,

        vel_x_max_wbc_gait = 1.0, -- 1.25, -- 0.5,
        vel_y_max_wbc_gait = 1.0, -- 0.6, -- 0.3,
        rotvel_max_wbc_gait = 1.0, -- 2.0,

        -- hand_ee_pose_1 = {0.3, 0.2, 0.1, -1.57, -1.57, 0., 
        --                   0.3, -0.2, 0.1, 1.57, -1.57,  0., },
        -- hand_ee_pose_2 = {0.3, 0.14, 0.1, -1.57, -1.57, -0.34, 
        --                   0.3, -0.14, 0.1, 1.57, -1.57, 0.34, },
        hand_ee_pose_1 = {0.3, 0.2, 0.00, -1.57, -1.57, 0., 
                          0.3, -0.2, 0.00, 1.57, -1.57,  0., },
        hand_ee_pose_2 = {0.3, 0.14, 0.00, -1.57, -1.57, -0.34, 
                          0.3, -0.14, 0.00, 1.57, -1.57, 0.34, },
        completion_time = 1200.0,
    },

    command_manager = {
        k1_use_new_state_manager = true,

        vel_x_max = 0.3,
        vel_y_max = 0.3,
        rotvel_max = 0.8,

        vel_x_max_run = 1.0,
        vel_y_max_run = 0.3,
        rotvel_max_run = 1.0,

        roll_max = 0.1,
        pitch_max = 0.1,
        height_bias_max = 0.02,
        height = 0.48,

        -- desc: 当连接多个 joystick 时，表示使用哪个 joystick
        -- scope: 
        -- unit: m
        joystick_index = 0,

        -- desc: 表示是否在motion中开启遥控器服务
        enable_remote_controller = false,

        enable_big_step_locomotion = false,
        enable_robocup_locomotion = true,
        enable_face_down_get_up = false,
        enable_face_up_get_up = false,
        enable_push_up = false,
        enable_lie_down = false,

        use_joystick = true,
        -- lcm_channel = "CHANNEL_DECISION_1",

        default_planner_index = 3,

        motor_cmd_diff_max = 0.4,
        low_cmd_kp_from_high_api = 40.0,
        low_cmd_kd_from_high_api = 0.65,
        head_yaw_cmd_speed = 0.005,
        head_pitch_cmd_speed = 0.005,
        head_pitch_index = 1,
        head_yaw_index = 0,
        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.6755, -2.2689, -2.1293,
            -3.3161, -1.5708, -2.2689, -0.1745,
            -3.0020, -0.4014, -1.0472, -0.0000, -1.8727, -1.3491,
            -3.0020, -1.5708, -1.0472, -0.0000, -1.8727, -1.3491,
        },
        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689, 0.1745,
            1.2217,  1.6755,  2.2689, 2.1293,
            2.2166,  1.5708,  1.0472,  2.2340,  1.3491,  1.3491,
            2.2166,  0.4014,  1.0472,  2.2340,  1.3491,  1.3491,
        },
        hand_ee_pose_1 = {0.3, 0.2, 0.1, 0. , 0. , 0., 
                          0.3, -0.2, 0.1, 0. , 0. , 0., },
        hand_ee_pose_2 = {0.3, 0.1, 0.1, 0. , 0. , -0.57, 
                          0.3, -0.1, 0.1, 0. , 0. , 0.57, },
        completion_time = 2000.0,

        urdf_path = "urdf_path",
        use_config = true,
        command_config_path = "command_config_path",
    },

    dcm_mode_portal_collect = {
        portal_pair_key = "portal_for_dcm_mode",
    },
    dcm_mode_portal_publish = {
        portal_pair_key = "portal_for_dcm_mode",
    },

    publisher = {
        send_over_temp_light_status_threshold = 85.0,

        head_name = "head_pitch_link",
        head_point = {0.0613, 0.0, 0.108},

        robot_name = "",
        is_serial = false,
        left_foot_name = "left_foot_link",
        right_foot_name = "right_foot_link",
        feet_point_pos_local = {0.014, 0.000, -0.024,},
    },
    
    joint_map_output = {
        record_data = true,
        n_joint_original = 22,
        n_joint_mapped = 22,
        mapping = {
            0, 1, 
            2, 3, 4, 5, 
            6, 7, 8, 9, 
            10, 11, 12, 13, 14, 15, 
            16, 17, 18, 19, 20, 21,
        },
        pos_factor = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
        },
        torq_factor = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
        },
    },

    joint_map_input = {
        record_data = true,
        use_pvt_ = true,
        n_joint_original = 22,
        n_joint_mapped = 22,
        mapping = {
            0, 1, 
            2, 3, 4, 5, 
            6, 7, 8, 9, 
            10, 11, 12, 13, 14, 15, 
            16, 17, 18, 19, 20, 21,
        },
        pos_factor = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
        },
        torq_factor = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
        },
        kp_factor = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
        },
        kd_factor = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 1., 1., 1.,
        },
    },

    debugging_mode = {
        n_joint = 22,

        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.6755, -2.2689, -2.1293,
            -3.3161, -1.5708, -2.2689, -0.1745,
            -3.0020, -0.4014, -1.0472, -0.0000, -1.8727, -1.3491,
            -3.0020, -1.5708, -1.0472, -0.0000, -1.8727, -1.3491,
        },
        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689, 0.1745,
            1.2217,  1.6755,  2.2689, 2.1293,
            2.2166,  1.5708,  1.0472,  2.2340,  1.3491,  1.3491,
            2.2166,  0.4014,  1.0472,  2.2340,  1.3491,  1.3491,
        },
    },
    damping_mode = {
        record_data = true,
        n_joint = 22,
        kd = {
            2., 2.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            5., 5., 5., 5., 0.5, 0.5,
            5., 5., 5., 5., 0.5, 0.5,
        }
    },

    parallel_mech_input = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },

    parallel_mech_input_stance = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },

    parallel_mech_input_rl_locomotion = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },

    parallel_mech_input_rl_locomotion_run = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
        use_pd_gain_convert_ = true,
        kp_cross_term_comp_factor_ = 1.,
        kd_cross_term_comp_factor_ = 1.,
        kp_max_ = 100.,
        kp_min_ = 10.,
        tau_ff_max_ = {36., 36.},
    },

    parallel_mech_input_rl_locomotion_old = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
        use_pd_gain_convert_ = true,
        kp_cross_term_comp_factor_ = 1.,
        kd_cross_term_comp_factor_ = 1.,
        kp_max_ = 100.,
        kp_min_ = 10.,
        tau_ff_max_ = {36., 36.},
    },

    parallel_mech_input_amp_locomotion_run = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
        use_pd_gain_convert_ = true,
        kp_cross_term_comp_factor_ = 1.,
        kd_cross_term_comp_factor_ = 1.,
        kp_max_ = 100.,
        kp_min_ = 10.,
        tau_ff_max_ = {36., 36.},
    },

    parallel_mech_input_custom_traj = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },
    parallel_mech_input_rl_traj_fdr = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },
    parallel_mech_input_visual_kick = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },
    parallel_mech_input_visual_kick_v1 = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },
    parallel_mech_input_custom_mode = {
        use_pos2torq_convert_ = true,
        serial_vel_filter_weight_ = 1.0,
        torque_limit_ = 40.,
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },
    parallel_mech_output = {
        record_data = true,
        use_fk_hotstart_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },

    ----------------------------- all configs for traj_track_bmm -------------------------------
    traj_track_bmm = {
        num_dofs = 22,
        num_acts = 22,

        wait_time = 0.,
        fail_grav_threshold = 0.5,

        dof_vel_clip = 100.,
        ang_vel_clip = 100.,
        obs_stack_size = 0,

        num_traj = 16,
        is_serial = false,

        fixed_traj_idx = -1,
        trajectories = {
            traj_1 = { -- harbia-1 hjw [RT+Y]
                model_path = "lib/K1/dance/harbia/k1_g5e-Exp-1.pt",  
                traj_path = "lib/K1/dance/harbia/k1_g5e_traj.pt",                                                
                use_isaaclab = false,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.886075949, 0.886075949, 0.886075949, 0.886075949,
                    0.886075949, 0.886075949, 0.886075949, 0.886075949,
                    0.09375, 0.0625, 0.0625, 0.125, 0.166666667, 0.166666667,
                    0.09375, 0.0625, 0.0625, 0.125, 0.166666667, 0.166666667,
                },
                pd_tracking_joint_ids = {},
            },
            traj_2 = { -- harbia-2 [RT+B]
                model_path = "lib/K1/dance/harbia/k1_h2-Exp-1.pt",  
                traj_path = "lib/K1/dance/harbia/k1_h2_traj.pt",                                               
                use_isaaclab = false,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.886075949, 0.886075949, 0.886075949, 0.886075949,
                    0.886075949, 0.886075949, 0.886075949, 0.886075949,
                    0.09375, 0.0625, 0.0625, 0.125, 0.166666667, 0.166666667,
                    0.09375, 0.0625, 0.0625, 0.125, 0.166666667, 0.166666667,
                },
                pd_tracking_joint_ids = {},
            },            
            traj_3 = { -- MJ1_2 wf [LT+X]
                model_path = "lib/K1/dance/harbia/k1_mj_dance_002_2025-12-30_18-28-06.pt",  
                traj_path = "lib/K1/dance/harbia/k1_mj2_seg1_cpu.pt",                                             
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    4.0, 4.0,
                    4.0, 4.0, 4.0, 4.0, 
                    4.0, 4.0, 4.0, 4.0, 
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    1.0, 1.0,
                    1.0, 1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0, 1.0,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.375, 0.375,
                    0.875, 0.875, 0.875, 0.875,
                    0.875, 0.875, 0.875, 0.875,
                    0.09375, 0.109375, 0.0625, 0.125, 0.166666667, 0.166666667,
                    0.09375, 0.109375, 0.0625, 0.125, 0.166666667, 0.166666667,
                },
                pd_tracking_joint_ids = {},
            },  
            traj_4 = { -- MJ4-seg hjw [RT+X]
                model_path = "lib/K1/dance/mj/k1_MJ4_1021_publish-9.pt",  
                traj_path = "lib/K1/dance/mj/k1_mj4_4_cpu.pt",                               
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    2., 2.,
                    0.6, 0.6, 0.6, 0.6,
                    0.6, 0.6, 0.6, 0.6,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.886075949, 0.886075949, 0.886075949, 0.886075949,
                    0.886075949, 0.886075949, 0.886075949, 0.886075949,
                    0.09375, 0.0625, 0.0625, 0.125, 0.166666667, 0.166666667,
                    0.09375, 0.0625, 0.0625, 0.125, 0.166666667, 0.166666667,
                },
                pd_tracking_joint_ids = {},
            }, 
            traj_5 = { -- moonwalk wf [LT+Y]
                model_path = "lib/K1/dance/mj/2025-10-16-02-13-43-k1_moonwalk_take_2_ro_10-12-04-20_loop20_3_35fps.3000.pt",
                traj_path = "lib/K1/dance/mj/k1_moonwalk_Take_2_rollout_10-12-04-20_loop20_3_clip5_35fps_cpu.pt",
                use_isaaclab = false,
                end_keep = false,
                use_phase = true,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    4., 4., 4., 4.,
                    4., 4., 4., 4.,
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    1., 1.,
                    1., 1., 1., 1.,
                    1., 1., 1., 1.,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },
                action_scale = {
                    0.3750, 0.3750, 0.8750, 0.8750, 0.8750, 0.8750, 0.8750, 0.8750, 0.8750,
                    0.8750, 0.0938, 0.1250, 0.0781, 0.1406, 0.1667, 0.1667, 0.0938, 0.1250,
                    0.0781, 0.1406, 0.1667, 0.1667
                },
                pd_tracking_joint_ids = {0,1},
            },
            traj_6 = {  --kick cwh [LT+B]
                model_path = "lib/K1/kungfu/2025-10-30_14-15-09_fight_001.pt",  
                traj_path = "lib/K1/kungfu/k1_fight_20251022_final_deploy_cpu.pt",                               
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95,
                    3.95, 3.95, 3.95, 3.95,
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.75949, 0.75949, 0.75949, 0.75949,
                    0.75949, 0.75949, 0.75949, 0.75949,
                    0.09375, 0.109375, 0.0625, 0.125, 0.16667, 0.16667,
                    0.09375, 0.109375, 0.0625, 0.125, 0.16667, 0.16667,
                },
                pd_tracking_joint_ids = {0,1},
            },  
            traj_7 = { -- kongfu zhiye [LT+A]      
                model_path = "lib/K1/kungfu/2025-10-23_21-03-27_roundkick-v6.4_finaly.pt",  
                traj_path = "lib/K1/kungfu/roundkick-v6.4_finaly-fps80_cpu.pt",
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0, 
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95,
                    80.0, 80.0, 80.0, 80.0, 30.0, 30.0, 
                    80.0, 80.0, 80.0, 80.0, 30.0, 30.0, 
                },
                kd = {
                    2.0, 2.0, 
                    0.5, 0.5, 0.5, 0.5, 
                    0.5, 0.5, 0.5, 0.5, 
                    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                },                          
                action_scale = {
                    0.1, 0.1,  
                    0.886, 0.886, 0.886, 0.886,  
                    0.886, 0.886, 0.886, 0.886,  
                    0.094, 0.109, 0.078, 0.125, 0.167, 0.167, 
                    0.094, 0.109, 0.078, 0.125, 0.167, 0.167, 
                },
                pd_tracking_joint_ids = {2, 3, 4},
            },
            traj_8 = { -- MJ1_2 wf [LT+X]
                model_path = "lib/K1/dance/mj/2025-10-02-02-04-52-k1_dance_mj_924_MJ1.pt",  
                traj_path = "lib/K1/dance/mj/k1_MJ1_2_cpu.pt",                                                
                use_isaaclab = false,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    4.0, 4.0, 4.0, 4.0, 
                    4.0, 4.0, 4.0, 4.0, 
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    1.0, 1.0,
                    1.0, 1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0, 1.0,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.375, 0.375,
                    0.875, 0.875, 0.875, 0.875,
                    0.875, 0.875, 0.875, 0.875,
                    0.0938, 0.1250, 0.0781, 0.1406, 0.166666667, 0.166666667,
                    0.0938, 0.1250, 0.0781, 0.1406, 0.166666667, 0.166666667,
                },
                pd_tracking_joint_ids = {0,1},
            },  
            traj_9 = { -- Sit Down hjw [LT+LB+B]
                model_path = "lib/K1/sit/k1_sit5A0930.pt",
                traj_path = "lib/K1/sit/k1_sit5A0930_cpu.pt",
                use_isaaclab = true,
                end_keep = true,
                use_phase = false,
                damping_start = -1;
                use_delta_dof_cmd = false,
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=167,                
                kp = {
                    5.0, 5.0,
                    0.1, 0.1, 3.95, 3.95,
                    0.1, 0.1, 3.95, 3.95,
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    2., 2.,
                    0.25, 0.25, 0.25, 0.25,
                    0.25, 0.25, 0.25, 0.25,
                    2., 2., 2., 2., 2.0, 2.0,
                    2., 2., 2., 2., 2.0, 2.0,
                },
                action_scale = {
                    0.1, 0.1, 
                    0.85443, 0.85443, 0.85443, 0.85443, 
                    0.85443, 0.85443, 0.85443, 0.85443,
                    0.09375, 0.0625, 0.0625, 0.125, 0.1667, 0.1667, 
                    0.09375, 0.0625, 0.0625, 0.125, 0.1667, 0.1667
                },
                pd_tracking_joint_ids = {0,1},
            },  
            traj_10 = { -- Sit Up hjw [LT+LB+X]
                model_path = "lib/K1/sit/k1_sit5A0930.pt",
                traj_path = "lib/K1/sit/k1_sit5A0930_cpu.pt",                
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = 10,
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = 150,
                end_frame=-1,                   
                kp = {
                    5.0, 5.0,
                    0.1, 0.5, 3.95, 3.95,
                    0.1, 0.5, 3.95, 3.95,
                    80.,80.,80.,80., 30., 30.,
                    80.,80.,80.,80., 30., 30.,
                },
                kd = {
                    2., 2.,
                    0.25, 0.25, 0.25, 0.25,
                    0.25, 0.25, 0.25, 0.25,
                    2., 2., 2., 2., 2.0, 2.0,
                    2., 2., 2., 2., 2.0, 2.0,
                },
                action_scale = {
                    0.1, 0.1, 
                    0.85443, 0.85443, 0.85443, 0.85443, 
                    0.85443, 0.85443, 0.85443, 0.85443,
                    0.09375, 0.0625, 0.0625, 0.125, 0.1667, 0.1667, 
                    0.09375, 0.0625, 0.0625, 0.125, 0.1667, 0.1667
                },
                pd_tracking_joint_ids = {0,1},
            },      
            traj_11 = { -- LionDance LJC 
                model_path = "lib/K1/wushi/wushi_exp.pt",
                traj_path = "lib/K1/wushi/wushi_traj.pt",                
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                 
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    100.,100.,100.,100., 50., 50.,
                    100.,100.,100.,100., 50., 50.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                },
                pd_tracking_joint_ids = {},
            },      
            traj_12 = { --wushi2 [LT+RB+A]
                model_path = "lib/K1/dance/wushi2_exp.pt",  
                traj_path = "lib/K1/dance/wushi2_traj.pt",                                                
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    100.,100.,100.,100., 50., 50.,
                    100.,100.,100.,100., 50., 50.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                },
                pd_tracking_joint_ids = {},
            },
            traj_13 = { -- 山河故人 [LT+RB+B]
                model_path = "lib/K1/dance/2025-12-18_02-13-35_go_west_k1.pt",  
                traj_path = "lib/K1/dance/gowest_neat_cpu.pt",                                                
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    100.,100.,100.,100., 50., 50.,
                    100.,100.,100.,100., 50., 50.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                },
                pd_tracking_joint_ids = {},
            },    
            traj_14 = { -- 改革春风 [LT+RB+X]
                model_path = "lib/K1/dance/2026-02-03_17-33-55_chunfeng.pt",  
                traj_path = "lib/K1/dance/chunfeng_neat4_cpu.pt",                                                 
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    10.0, 10.0,
                    3.95, 3.95, 3.95, 3.95, 
                    3.95, 3.95, 3.95, 3.95, 
                    100.,100.,100.,100., 50., 50.,
                    100.,100.,100.,100., 50., 50.,
                },
                kd = {
                    2., 2.,
                    0.3, 0.3, 0.3, 0.3,
                    0.3, 0.3, 0.3, 0.3,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },                          
                action_scale = {
                    0.1, 0.1,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.6329, 0.6329, 0.6329, 0.6329,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                    0.075, 0.05, 0.05, 0.1, 0.1, 0.1,
                },
                pd_tracking_joint_ids = {},
            }, 
            traj_15 = { -- readyposehalf: 69-dim obs / 12-dim act (lower body); head+arms PD to traj
                model_path = "lib/K1/wushi/readyposehalf_exp.pt",
                traj_path = "lib/K1/wushi/readypose_traj.pt",
                use_isaaclab = true,
                use_lower_body_obs_69 = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,
                kp = {
                    3.9478417602100686, 3.9478417602100686,
                    3.9478417602100686, 3.9478417602100686, 3.9478417602100686, 3.9478417602100686,
                    3.9478417602100686, 3.9478417602100686, 3.9478417602100686, 3.9478417602100686,
                    30.200989465607023, 21.447961045805584, 17.846013389258083, 60.401978931214046, 35.692026778516166, 35.692026778516166,
                    30.200989465607023, 21.447961045805584, 17.846013389258083, 60.401978931214046, 35.692026778516166, 35.692026778516166,
                },
                kd = {
                    0.25132741228, 0.25132741228,
                    0.25132741228, 0.25132741228, 0.25132741228, 0.25132741228,
                    0.25132741228, 0.25132741228, 0.25132741228, 0.25132741228,
                    3.60497756989125, 2.560161764834957, 2.1302109340993156, 4.806636759855, 4.260421868198631, 4.260421868198631,
                    3.60497756989125, 2.560161764834957, 2.1302109340993156, 4.806636759855, 4.260421868198631, 4.260421868198631,
                },
                -- 12 scales in policy/lb order: sim(3,4,8,9,12,13,16,17,18,19,20,21)
                -- = L/R HipPitch, L/R HipRoll, L/R HipYaw, L/R Knee, L/R AnklePitch, L/R AnkleRoll
                -- values taken from 22-DOF URDF-order table at the corresponding leg URDF index
                action_scale = {
                    0.5628954647118317, 0.5628954647118317,  -- L/R Hip Pitch  (URDF 10,16)
                    0.8858650926968038, 0.8858650926968038,  -- L/R Hip Roll   (URDF 11,17)
                    0.5365343951699267, 0.5365343951699267,  -- L/R Hip Yaw    (URDF 12,18)
                    0.46356097093915555, 0.46356097093915555, -- L/R Knee      (URDF 13,19)
                    0.26826719758496337, 0.26826719758496337, -- L/R Ank Pitch (URDF 14,20)
                    0.26826719758496337, 0.26826719758496337, -- L/R Ank Roll  (URDF 15,21)
                },
                pd_tracking_joint_ids = {},
            }, 
            traj_16 = { -- 改革春风 [LT+RB+X]
                model_path = "lib/K1/wushi/mjnew_exp.pt",  
                traj_path = "lib/K1/wushi/mjfinal_traj.pt",                                                 
                use_isaaclab = true,
                end_keep = false,
                use_phase = false,
                use_delta_dof_cmd = false,
                damping_start = -1;
                smooth_factor = 0.0,
                obs_traj_offsets = {0},
                start_frame = -1,
                end_frame=-1,                
                kp = {
                    3.9478417602100686, 3.9478417602100686,
                    3.9478417602100686, 3.9478417602100686, 3.9478417602100686, 3.9478417602100686,
                    3.9478417602100686, 3.9478417602100686, 3.9478417602100686, 3.9478417602100686,
                    30.200989465607023, 21.447961045805584, 17.846013389258083, 60.401978931214046, 35.692026778516166, 35.692026778516166,
                    30.200989465607023, 21.447961045805584, 17.846013389258083, 60.401978931214046, 35.692026778516166, 35.692026778516166,
                },
                kd = {
                    0.25132741228, 0.25132741228,
                    0.25132741228, 0.25132741228, 0.25132741228, 0.25132741228,
                    0.25132741228, 0.25132741228, 0.25132741228, 0.25132741228,
                    3.60497756989125, 2.560161764834957, 2.1302109340993156, 4.806636759855, 4.260421868198631, 4.260421868198631,
                    3.60497756989125, 2.560161764834957, 2.1302109340993156, 4.806636759855, 4.260421868198631, 4.260421868198631,
                },     
                action_scale = {
                    0.3799544386804864, 0.3799544386804864,
                    0.8865603569211349, 0.8865603569211349, 0.8865603569211349, 0.8865603569211349,
                    0.8865603569211349, 0.8865603569211349, 0.8865603569211349, 0.8865603569211349,
                    0.5628954647118317, 0.8858650926968038, 0.5365343951699267, 0.46356097093915555, 0.26826719758496337, 0.26826719758496337,
                    0.5628954647118317, 0.8858650926968038, 0.5365343951699267, 0.46356097093915555, 0.26826719758496337, 0.26826719758496337,
                },
                pd_tracking_joint_ids = {},
            },                                                                
        },
        dof_velocity_scale = 1.0,
        parallel_joint_ids = {14, 15, 20, 21},
        torque_limit = {
            8.0, 8.0,
            18.0, 18.0, 18.0, 18.0, 
            18.0, 18.0, 18.0, 18.0, 
            60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
            60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
        },
        damping_params={
            5.0, 5.0,
            0.1, 0.1, 0.1, 0.1,
            0.1, 0.1, 0.1, 0.1,
            10.,20.,20.,10., 10., 10.,
            10.,20.,20.,10., 10., 10.,
        },

        reference = {
            0., 0.,
            0., -1.3, 0., 0.,
            0.,  1.3, 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
        },

        urdf_to_isaaclab_index = {
            0, 2, 6, 10, 16, 1, 3, 7, 11, 17, 4, 8, 12, 18, 5, 9, 13, 19, 14, 20, 15, 21
        },
        isaaclab_to_urdf_index = {
            0, 5, 1, 6, 10, 14, 2, 7, 11, 15, 3, 8, 12, 16, 18, 20, 4, 9, 13, 17, 19, 21
        },

        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.7453, -2.2689, -2.4435,
            -3.3161, -1.5708, -2.2689, -0.0000,
            -3.0, -0.2, -1.0472, -0.0000, -0.8727, -0.4363,
            -3.0, -1.5708, -1.0472, -0.0000, -0.8727, -0.4363,
        },

        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689,  0.0000,
            1.2217,  1.7453,  2.2689,  2.4435,
            2.2166,  1.5708,  1.0472,  2.1817,  0.3491,  0.4363,
            2.2166,  0.2,  1.0472,  2.1817,  0.3491,  0.4363,
        },
    },

    parallel_mech_input_traj_track_bmm = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与"两脚踝电机轴线的连接面"的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与"两脚踝电机轴线的连接面"的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与"两脚踝电机轴线的连接面"的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },


    ----------------------------- all configs for single_traj_track -------------------------------

    single_traj_track = {
        num_dofs = 22,

        wait_time = 0.,
        fail_grav_threshold = 0.5,

        dof_vel_clip = 20.,
        ang_vel_clip = 30.,
        obs_stack_size = 0,

        num_traj = 1,
        trajectories = {
            traj_1 = {
                model_path = "lib/K1/dance/k1_dance_2025-05-27-21-10-38.pt",
                traj_path = "lib/K1/dance/traj_2025-05-14-01-57-20.pt",
                obs_traj_offsets = {0, 2, 4},
                kp = {
                    20.0, 20.0,
                    20.0, 20.0, 20.0, 20.0,
                    20.0, 20.0, 20.0, 20.0,
                    80.0, 80.0, 80.0, 80.0, 20.0, 20.0,
                    80.0, 80.0, 80.0, 80.0, 20.0, 20.0,
                },
                kd = {
                    0.5, 0.5,
                    2., 2., 2., 2.,
                    2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                    2., 2., 2., 2., 2., 2.,
                },
            },
        },
        num_acts = 22,

        fixed_joint_ids = {0, 1},
        dof_velocity_scale = 0.1,

        smooth_factor = 0.8,
        parallel_joint_ids = {14, 15, 20, 21},
 
        torque_limit = {
            8.0, 8.0,
            14.0, 14.0, 14.0, 14.0, 
            14.0, 14.0, 14.0, 14.0, 
            60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
            60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
        },

        reference = {
            0., 0.,
            0., -1.4, 0., 0.,
            0.,  1.4, 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
        },

        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.7453, -2.2689, -2.4435,
            -3.3161, -1.5708, -2.2689, -0.0000,
            -3.0, -0.2, -1.0472, -0.0000, -0.8727, -0.4363,
            -3.0, -1.5708, -1.0472, -0.0000, -0.8727, -0.4363,
        },

        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689,  0.0000,
            1.2217,  1.7453,  2.2689,  2.4435,
            2.2166,  1.5708,  1.0472,  2.1817,  0.3491,  0.4363,
            2.2166,  0.2,  1.0472,  2.1817,  0.3491,  0.4363,
        },
    },

    parallel_mech_input_single_traj_track = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {14, 15, 20, 21},
        joint_idx_serial_ = {14, 15, 20, 21},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0590,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1580,                                               -- 长连杆长度
        len_link_R_ = 0.0820,                                               -- 短连杆长度
        r_joint_ = 0.0410,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0244, 0.0000, -0.1617},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, 0.0000},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0290, 0.0000, 0.0175},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0480,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -102.6627,                                  -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 31.4698,                                   -- 初始位置时，上下两曲柄与“两脚踝电机轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.5172, 2.0664, 1.5172, 2.0664,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“两脚踝电机轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.13956, 0.00000, 0.13956, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角弧度（左Pitch、左Roll、右Pitch、右Roll）
        is_mirror_ = {false, true},
    },

    ------------------------------------------------------------------------------------------------------------
    noise = {
        gyro_noise_amp = 0.000,
        lin_vel_noise_amp = 0.000,
        rot_vel_noise_amp = 0.000,
        acc_noise_amp = 0.000,
        q_noise_amp = 0.000,
        dq_noise_amp = 0.00,
        torq_noise_amp = 0.0,
        random_seed = 123,
    },

    contact_probability_portal_collect = {
        portal_pair_key = "portal for contact_probability",
    },
    contact_probability_portal_publish = {
        portal_pair_key = "portal for contact_probability",
    },

    rviz = {
        lcm_channel = "CHANNEL_RVIZ_STATE",
    },

    dcm_planner = {
        record_data_ = true,
        -- stance
        plan_horizon_ = 0,
        plan_dt_ = 0.02,
        mass_ = 30.2,
        prepare_time_ = 2.0,
        stop_time_ = 1.0,
        period_ = 1.0,
        nominal_torso_pos_ = { 0.0, 0.0, 0.50, },
        nominal_com_pos_ = { 0.00, 0.0, 0.47, },
        set_torso_to_center_of_feet_ = false,  
        set_com_to_center_of_feet_ = false,
        pos_amp_ = { 0.0, 0.0, 0.0 },
        rpy_amp_ = { 0.0, 0.0, 0.0 },
        -- pos_amp_ = { -0.045, 0.0, 0.115 },
        -- rpy_amp_ = { 0.0, -0.45, 0.0 },

        -- squat
        tar_squat_com_pos_ = {0.05, 0., 0.30},
        tar_squat_torso_pos_ = {0.05, 0., 0.30},
        tar_squat_torso_rpy_ = {0., 0.8, 0.},
        squat_down_time_ = 4.0,
        squat_up_time_ = 5.0,

        -- dcm
        biped_state_  = 9,
        speed_mode_   = true,
        control_mode_ = true,
        para_update_  = true,

        acc_min_ = {-0.4, -0.4, -0.8},
        acc_max_ = {0.4, 0.4, 0.8},
        jerk_ = 10.0,

        speed_norm_max_ = 1.0,
        speed_norm_p_ = 1.0,
        speed_min_ = {-0.4, -0.2, -2.0},
        speed_max_ = { 0.4,  0.2,  2.0},
        speed_dead_zone_ = { 0.02, 0.01, 0.01, 0.01},

        body_width_ = 0.18,
        first_step_is_left_ = true,

        walking_cycle_    = 0.55,
        step_num_         = 19.0,
        step_length_      = 0.2,
        step_side_walk_   = 0.0,
        step_rotate_      = 0.0,
        m_upbody_offset_    = 0.0,
        m_leftleg_offset_   = 0.0,
        m_rightleg_offset_  = 0.0,
        -- k_footprint_new_step_ = 1.0,
        -- k_footprint_while_wing_ = 1.0,
        k_footprint_new_step_ = 0.0,
        k_footprint_while_wing_ = 0.0,
        dcm_est_filter_alpha_ = 0.7,    -- The closer the number is to 0, the smoother the DCM estimation curve will be
        adjust_dcm_threshold_ = 100.1,

        -- --------------------------------- Parameters about swing foot (forward)---------------------------------
        t_swing_ratio_forward_ = {0.22, 0.14, 0.14, 0.2},
        t_swing_pitch_ratio_forward_ = {0.2, 0.2, 0.24, 0.06},
        x_swing_ratio_forward_ = {0.2, 0.55, 0.85},
        y_swing_ratio_forward_ = {0.2, 0.55, 0.85},
        x_swing_ratio_kickingforward_ = {0.4, 0.7, 1.2},

        swing_height_forward_ = 0.05,
        swing_height_forward_min_ = 0.04,
        z_swing_ratio_forward_ = {0.5, 0.5},
        t_up_ratio_forward_ = 0.05, --0.1,
        t_down_ratio_forward_ = 0.05, --0.08,
        pitch_up_max_ = 0.0, --0.1,
        pitch_down_max_ = 0.0, -- -0.06,
        pitch_swing_ratio_forward_ = {1.0, 0.1, 1.0}, --{1.0, 0.4, 1.0},
        pitch_up_ratio_forward_ = 0.4,
        pitch_down_ratio_forward_ = 0.6,

        -- --------------------------------- Parameters about swing foot (backward) ---------------------------------
        t_swing_ratio_back_ = {0.2, 0.15, 0.15, 0.2},
        x_swing_ratio_back_ = {0.21, 0.55, 0.85},
        y_swing_ratio_back_ = {0.21, 0.55, 0.85},
        swing_height_back_ = 0.05,
        z_swing_ratio_back_ = {0.68, 0.60},

        -- --------------------------------- Parameters about heel-to-toe dcm planning ---------------------------------
        time_scale_ratio_ = 1.0,
        lip_h_ratio_   = 3.0,
        delta_h_heel2toe_ = 0.0,
        zmp_offset_x_forward_                   =  0.020,        -- walk forward
        zmp_offset_x_backward_                  =  0.015,        -- walk backward
        max_com_vel_x_forward_for_zmp_offset_   = 0.3,
        max_com_vel_x_backward_for_zmp_offset_  = 0.3,
   
        zmp_offset_x_forward_offset_ = 0.005,
        zmp_offset_x_backward_offset_ = -0.005,
        zmp_y_outside_forward_walking_offset_ =  0.00,
        zmp_y_outside_backward_walking_offset_ =  0.00,


        zmp_y_outside_left_walking_offset_ =  0.09,
        zmp_y_outside_right_walking_offset_ =  -0.09,

        -- zmp_offset_y_outside_lfoot_ =  0.065,
        zmp_offset_y_outside_lfoot_ =  0.05,
        -- zmp_offset_y_outside_lfoot_ =  0.045,
        -- zmp_offset_y_outside_lfoot_ =  0.05,
        -- zmp_offset_y_outside_lfoot_ =  0.02,
        -- zmp_offset_y_outside_lfoot_ =  0.01,

        -- zmp_offset_y_outside_rfoot_ =  0.065,
        zmp_offset_y_outside_rfoot_ =  0.05,
        -- zmp_offset_y_outside_rfoot_ =  0.035,
        -- zmp_offset_y_outside_rfoot_ =  0.05,
        -- zmp_offset_y_outside_rfoot_ =  0.02,
        -- zmp_offset_y_outside_rfoot_ =  0.01,
        zmp_offset_x_toe_ = 0.01,
        zmp_offset_x_heel_ = 0.01,
        zmp_x_range_ssp_ = 0.02,
        zmp_offset_y_heel2toe_lfoot_ = 0.0,
        zmp_offset_y_heel2toe_rfoot_ = -0.0,
        time_ratio_heel2toe_ = 0.3,
        ht_dsp_percentage_ = 0.3,
        th_dsp_percentage_ = 0.3,

        time_step_ = 0.001 * controller_base_dt_ms,

        -- para_Step_Length_Frwd_Min_ = 0.02,
        -- para_Step_Length_Frwd_Max_ = 0.1,
        -- para_Step_Length_Back_Max_ = 0.07,
        -- para_Step_Side_Min_ = 0.02,
        -- para_Step_Side_Max_ = 0.1,
        -- para_Step_Rotate_Stay_Min_ = 0.01,
        -- para_Step_Rotate_Side_Min_ = 0.01,
        -- para_Step_Rotate_Side_ = 0.01,
        -- para_Step_Rotate_Stay_Max_ = 0.25,
        -- para_Step_Rotate_Frwd_Max_ = 0.1,
        -- para_Dist_Length_Min_ = 0.02,
        -- para_Dist_Side_Min_ = 0.01,
        -- para_Dist_Rotate_Min_ = 0.01,
        -- para_Speed_Up_Ratio_ = 0.5,

        left_foot_name_ = "left_foot_link",
        right_foot_name_ = "right_foot_link",

        -- -------------------------- demo 相关参数 -------------
        -- demo_ = "demo2",
         demo_ = "",

        demo_step_ = {1.0, 12.0, 2.0, 12.0, 2.0, 7.0, 1.0,
                        9.0, 1.0, 4.5, 2.0, 3.5, 2.0, 1.0, 
                        2.0, 1.0, 1.5, 1.0, 2.0, 1.0, 1.0, 
                        1.0, 14.0, 2.0, 9.5, 2.0, 14.0, 2.0, 
                        10.0, 1.0, 12.0},

        demo_gait_index_ = {0, 1, 1, 1, 1, 1, 1,
                            1, 1, 1, 1, 1, 1, 1,
                            1, 1, 1, 1, 1, 1, 1,
                            1, 1, 1, 1, 1, 1, 1,
                            1, 1, 1},
        demo_vel_x_ = {0.0, 0.2, 0.0, -0.1, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
                     0.0, -0.2, 0.0, -0.2, 0.0, 0.2, 0.0,
                     0.0, 0.16, 0.0, 0.0, 0.0, 0.155, 0.0,
                     0.0, 0.0, 0.0},
    
        demo_vel_y_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.07, 0.0,
                    -0.07, 0.0, 0.0, 0.0, 0.0, 0.0, -0.07,
                    0.0, -0.07, 0.0, 0.07, 0.0, 0.07, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    -0.07, 0.0, -0.07},
        demo_vel_z_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.7, 0.0, -0.7, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.19, 0.0, 0.7, 0.0, -0.185, 0.0,
                    0.3, 0.0, -0.3},
    
        -- 0.5 米每秒速度行走
        -- demo_step_ = {1.0, 600.0, 7.0},
        -- demo_gait_index_ = {0, 1, 1},
    
        -- demo_vel_x_ = {0.0, 0.024, 0.0},
                            
        -- demo_vel_y_ = {0.0, -0.01688, 0.0},
        -- demo_vel_z_ = {0.0, 0.0, 0.0},
          
    },

    hand_planner = {
        record_data_ = false,
        
        action_list_ = {"wave_L", "pick_ball", "wave_R",},

        action_wave_L = {
            param_names = {"period", "wy_1", "wy_2"},
            L = {
                enable = true,
                init_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.2, 0.2, 0.0},
                        lin_vel = {0.2, 0.0, 0.1},
                        rpy = {0., 0., 0.},
                        T = 0.5,
                    },
                    WP_2 = {
                        pos = {0.3, 0.25, 0.45},
                        lin_vel = {0., 0., 0.},
                        rpy = {-0.2, -1.3, 0.1},
                        T = 1.,
                    },
                },
                repeat_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.3, 0.1, 0.45},
                        _pos = {"", "wy_1", ""},
                        lin_vel = {0., 0., 0.},
                        rpy = {0.5, -1.3, -0.1},
                        T = 0.5,
                        _T = "period",
                    },
                    WP_2 = {
                        pos = {0.3, 0.3, 0.45},
                        _pos = {"", "wy_2", ""},
                        lin_vel = {0., 0., 0.},
                        rpy = {-0.5, -1.3, 0.1},
                        T = 0.5,
                        _T = "period",
                    },
                },
                exit_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.2, 0.2, 0.0},
                        lin_vel = {-0.2, 0.0, 0.1},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.1, 0.2, -0.1},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., 0.},
                        T = 0.5,
                    },
                },
            },
            R = {
                enable = false,
            },
        },
        action_wave_R = {
            L = {
                enable = false,
            },
            R = {
                enable = true,
                init_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.2, -0.2, 0.0},
                        lin_vel = {0.2, 0.0, 0.1},
                        rpy = {0., 0., 0.},
                        T = 0.5,
                    },
                    WP_2 = {
                        pos = {0.3, -0.25, 0.45},
                        lin_vel = {0., 0., 0.},
                        rpy = {0.2, -1.3, -0.1},
                        T = 1.,
                    },
                },
                repeat_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.3, -0.1, 0.45},
                        lin_vel = {0., 0., 0.},
                        rpy = {-0.5, -1.3, 0.1},
                        T = 0.5,
                    },
                    WP_2 = {
                        pos = {0.3, -0.3, 0.45},
                        lin_vel = {0., 0., 0.},
                        rpy = {0.5, -1.3, -0.1},
                        T = 0.5,
                    },
                },
                exit_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.2, -0.2, 0.0},
                        lin_vel = {-0.2, 0.0, 0.1},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.1, -0.2, -0.1},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., 0.},
                        T = 0.5,
                    },
                },
            },
        },

        action_pick_ball = {
            param_names = {
                "ready_L_x", "ready_L_y", "ready_L_z", 
                "ready_R_x", "ready_R_y", "ready_R_z",
                "catch_L_x", "catch_L_y", "catch_L_z",
                "catch_R_x", "catch_R_y", "catch_R_z",

            },
            L = {
                enable = true,
                init_seq = {
                    seq_len = 3,
                    WP_1 = {
                        pos = {0.15, 0.3, -0.0},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., -0.4},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.3, 0.25, 0.05},
                        lin_vel = {0.2, 0.0, 0.1},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                    WP_3 = {
                        pos = {0.45, 0.25, 0.2},
                        _pos = {"ready_L_x", "ready_L_y", "ready_L_z"},
                        lin_vel = {0.0, 0.0, 0.0},
                        rpy = {0., -0.03, 0.},
                        T = 1.,
                    },
                },
                repeat_seq = {
                    seq_len = 1,
                    WP_1 = {
                        pos = {0.48, 0.11, 0.18},
                        _pos = {"catch_L_x", "catch_L_y", "catch_L_z"},
                        lin_vel = {0.0, 0.0, 0.0},
                        rpy = {0., -0.15, -0.0},
                        T = 1.,
                    },
                },
                exit_seq = {
                    seq_len = 5,
                    WP_1 = {
                        pos = {0.23, 0.11, 0.5},
                        lin_vel = {0., 0.0, 0.},
                        rpy = {0., -1.45, -0.0},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.23, 0.11, 0.50},
                        lin_vel = {0., 0.0, 0.},
                        rpy = {0., -1.45, -0.0},
                        T = 0.5,
                    },
                    WP_3 = {
                        pos = {0.40, 0.11, 0.45},
                        lin_vel = {-0.2, 0.0, -0.0},
                        rpy = {0., -0.8, -0.0},
                        T = 0.3,
                    },
                    WP_4 = {
                        pos = {0.45, 0.2, 0.20},
                        lin_vel = {0.00, 0.0, -0.2},
                        rpy = {0., -0.3, 0.},
                        T = 1.,
                    },
                    WP_5 = {
                        pos = {0.1, 0.2, -0.1},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                },
            },
            R = {
                enable = true,
                init_seq = {
                    seq_len = 3,
                    WP_1 = {
                        pos = {0.15, -0.3, -0.0},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., 0.4},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.3, -0.25, 0.05},
                        lin_vel = {0.2, 0.0, 0.1},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                    WP_3 = {
                        pos = {0.45, -0.25, 0.2},
                        _pos = {"ready_R_x", "ready_R_y", "ready_R_z"},
                        lin_vel = {0.0, 0.0, 0.0},
                        rpy = {0., -0.03, 0.},
                        T = 1.,
                    },
                },
                repeat_seq = {
                    seq_len = 1,
                    WP_1 = {
                        pos = {0.48, -0.11, 0.18},
                        _pos = {"catch_R_x", "catch_R_y", "catch_R_z"},
                        lin_vel = {0.0, 0.0, 0.0},
                        rpy = {0., -0.15, -0.0},
                        T = 1.,
                    },
                },
                exit_seq = {
                    seq_len = 5,
                    WP_1 = {
                        pos = {0.23, -0.11, 0.5},
                        lin_vel = {0., 0.0, 0.},
                        rpy = {0., -1.45, -0.0},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.23, -0.11, 0.50},
                        lin_vel = {0., 0.0, 0.},
                        rpy = {0., -1.45, -0.0},
                        T = 0.5,
                    },
                    WP_3 = {
                        pos = {0.40, -0.11, 0.45},
                        lin_vel = {-0.2, 0.0, -0.0},
                        rpy = {0., -0.8, -0.0},
                        T = 0.3,
                    },
                    WP_4 = {
                        pos = {0.45, -0.2, 0.20},
                        lin_vel = {0.00, 0.0, -0.2},
                        rpy = {0., -0.3, 0.},
                        T = 1.,
                    },
                    WP_5 = {
                        pos = {0.1, -0.2, -0.1},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                },
            },
        },
        action_level_arm = {
            L = {
                enable = true,
                init_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.1, 0.4, -0.1},
                        lin_vel = {0.0, 0.2, 0.2},
                        rpy = {0., 0., 1.3},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.1, 0.55, 0.1},
                        lin_vel = {0.0, 0.0, 0.0},
                        rpy = {0., 0.1, 1.5},
                        T = 1.,
                    },
                    
                },
                repeat_seq = {
                    seq_len = 1,
                    WP_1 = {
                        pos = {0.1, 0.55, 0.1},
                        lin_vel = {0.0, 0.0, 0.0},
                        rpy = {0., 0.1, 1.5},
                        T = 0.2,
                    },
                },
                exit_seq = {
                    seq_len = 2,
                    WP_1 = {
                        pos = {0.1, 0.4, -0.1},
                        lin_vel = {0.0, -0.2, -0.2},
                        rpy = {0., 0., 1.3},
                        T = 1.,
                    },
                    WP_2 = {
                        pos = {0.1, 0.2, -0.1},
                        lin_vel = {0., 0., 0.},
                        rpy = {0., 0., 0.},
                        T = 1.,
                    },
                },
            },
            R = {
                enable = false,
            },
        },
    },

    hand_direct_command_planner = {
        record_data_ = false,
        ori_offset_left ={0., 0., 1., 1., 0., 0., 0., 1., 0.,},
        ori_offset_right ={0., 0., 1., -1., 0., 0., 0., -1., 0.,},
        kp_joint = {
            40.0, 40.0,
            70., 70., 70., 70., 
            70., 70., 70., 70., 
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
        },
        kd_joint = {
            0.65, 0.65,
            .5, 1.5, .2, .2, 
            .5, 1.5, .2, .2, 
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
        },
        kp_joint_zero_t_ = {
            40.0, 40.0,
            70., 70., 70., 70.,
            70., 70., 70., 70.,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
            150.0, 120.0, 120.0, 150.0,  100.0,  100.0,
        },
        kd_joint_zero_t_ = {
            0.65, 0.65,
            .5, 1.5, .2, .2, 
            .5, 1.5, .2, .2, 
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
            20.0,  12.0,  12.0,  20.0,  0.1,  0.1,
        },
        urdf_path = "urdf_path",
        ee_name = {
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        constraint_type_names = {
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
        },
        constraint_body_names = {
            "left_forearm_pitch_link",
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        constratint_weight = {50.0, 1.0, 50.0, 1.0},
        ee_point = {
            -0.012,  0.213,  0.000,
            -0.012, -0.213,  0.000,
        },
        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.6755, -2.2689, -2.1293,
            -3.3161, -1.5708, -2.2689, -0.1745,
            -3.0020, -0.4014, -1.0472, -0.0000, -1.8727, -1.3491,
            -3.0020, -1.5708, -1.0472, -0.0000, -1.8727, -1.3491,
        },
        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689, 0.1745,
            1.2217,  1.6755,  2.2689, 2.1293,
            2.2166,  1.5708,  1.0472,  2.2340,  1.3491,  1.3491,
            2.2166,  0.4014,  1.0472,  2.2340,  1.3491,  1.3491,
        },
        q_better_guess = {
            0.00,  0.00,
            0.25, -1.40, 0.00, -0.50,
            0.25,  1.40, 0.00,  0.50,
            -0.40, -0.03,  0.05,  0.70, -0.35,  0.03,
            -0.40,  0.03, -0.05,  0.70, -0.35, -0.03,
        },
        arm_start_index = 2,
        arm_num = 4,
        record_traj_data_ = false;
        saved_data_path_ = "",
        saved_data_path_list_ = {},
    },
    
    hand_direct_command_portal_collect = {
        portal_pair_key = "portal_for_hand_direct_command_manager",
    },
    hand_direct_command_portal_publish = {
        portal_pair_key = "portal_for_hand_direct_command_manager",
    },

    stance_mode_portal_collect = {
        portal_pair_key = "portal_for_stance_mode",
    },
    stance_mode_portal_publish = {
        portal_pair_key = "portal_for_stance_mode",
    },

    stance_planner = {
        record_data_ = true,
        left_foot_name_ = "left_foot_link",
        right_foot_name_ = "right_foot_link",

        use_waist_joint_planning = true,

        list_action = {"yewen_squat", "deep_squat", "dance"},
        action_yewen_squat = {
            init_segments = {"yewen_init","yewen_stretch_out"},
            repeat_segments = {"yewen_hold",},
            exit_segments = {"yewen_retract","yewen_exit"},
        },
        action_deep_squat = {
            init_segments = {"deep_squat_init",},
            repeat_segments = {"deep_squat_hold",},
            exit_segments = {"deep_squat_exit",},
        },

        action_dance = {
            init_segments = {"dance_init",},
            repeat_segments = {"dance_repeat",},
            exit_segments = {"dance_exit",},
        },

        

        action_seg_yewen_init = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    pos = {0., 0.08, 0.42},
                    lin_vel = {0., 0., 0.},
                    duration = 1.5,
                },

            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 2.,
                    pos = {0., 0., 0.5},
                    lin_vel = {0., 0., 0.},
                    rpy = {0., 0., -0.},
                },
            },
            foot_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 1.,
                    pos = {0., -0.2, 0.05},
                    lin_vel = {0., 0., 0.},
                    rpy = {0., 0., 0.1},
                },
            },
            delay_time = 1.,
        },

        action_seg_yewen_stretch_out = {
            ref_foot = "left",
            move_foot_contact_state = false,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    pos = {0.00, -0.02, 0.42},
                    lin_vel = {0., 0., 0.},
                    duration = 2.,
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 2.,
                    pos = {0., 0., 0.5},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0.0, -0.},
                },
            },
            foot_seq = {
                seq_len = 3,
                WP_1 = {
                    start_tic = 0.,
                    duration = 1.,
                    pos = {0., -0.2, 0.1},
                    lin_vel = {0., 0.0, 0.},
                    rpy = {0., 0., -0.0},
                },
                WP_2 = {
                    start_tic = 1.,
                    duration = 2.,
                    pos = {0.40, -0.1, 0.15},
                    lin_vel = {0.1, 0.0, 0.},
                    rpy = {0., -0.9, -0.0},
                },
                WP_3 = {
                    start_tic = 3.,
                    duration = 1.,
                    pos = {0.47, -0.05, 0.3},
                    lin_vel = {0., 0.0, 0.},
                    rpy = {0., -1.45, -0.0},
                },
            },
        },

        action_seg_yewen_hold = {
            ref_foot = "left",
            move_foot_contact_state = false,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    pos = {0.00, -0.02, 0.42},
                    lin_vel = {0., 0., 0.},
                    duration = 0.2,
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.2,
                    pos = {0., 0., 0.5},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0.0, -0.},
                },
            },
            foot_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.2,
                    pos = {0.47, -0.05, 0.3},
                    lin_vel = {0., 0.0, 0.},
                    rpy = {0., -1.45, -0.0},
                },
            },
        },

        action_seg_yewen_retract = {
            ref_foot = "left",
            move_foot_contact_state = false,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    pos = {0.00, -0.02, 0.42},
                    lin_vel = {0., 0., 0.},
                    duration = 2.,
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 2.,
                    pos = {0., 0., 0.5},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0.0, -0.},
                },
            },
            foot_seq = {
                seq_len = 3,
                WP_1 = {
                    start_tic = 0.,
                    duration = 1.,
                    pos = {0.40, -0.1, 0.15},
                    lin_vel = {-0.1, 0.0, 0.},
                    rpy = {0., -0.9, -0.0},
                },
                WP_2 = {
                    start_tic = 1.,
                    duration = 1.,
                    pos = {0., -0.2, 0.1},
                    lin_vel = {0., 0.0, 0.},
                    rpy = {0., 0., -0.0},
                },
                WP_3 = {
                    start_tic = 2.,
                    duration = 1.,
                    pos = {-0., -0.2, 0.0},
                    lin_vel = {0., 0.0, 0.},
                    rpy = {0., 0., -0.},
                },
                
            },
        },

        action_seg_yewen_exit = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    pos = {0.0, -0.0, 0.45},
                    lin_vel = {0., 0., 0.},
                    duration = 2.5,
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.5,
                    pos = {0., 0., 0.5},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., -0.0, -0.0},
                },

            },
            foot_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.5,
                    pos = {-0.18, -0.25, 0.248},
                    lin_vel = {0., 0.0, 0.},
                    rpy = {0., 0., -0.7},
                },
            },
        },

        action_seg_deep_squat_init = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 3.,
                    pos = {0.05, 0., 0.33},
                    lin_vel = {0., 0., 0.},
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 3.,
                    pos = {0.05, 0., 0.33},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0.8, 0.},
                },

            },
            foot_seq = {
                seq_len = 0,
            },
        },

        action_seg_deep_squat_hold = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.1,
                    pos = {0.05, 0., 0.33},
                    lin_vel = {0., 0., 0.},
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.1,
                    pos = {0.05, 0., 0.33},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0.8, 0.},
                },

            },
            foot_seq = {
                seq_len = 0,
            },
        },

        action_seg_deep_squat_exit = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 3.,
                    pos = {0.0, 0., 0.50},
                    lin_vel = {0., 0., 0.},
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 3.,
                    pos = {0.0, 0., 0.50},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0., 0.},
                },

            },
            foot_seq = {
                seq_len = 0,
            },
        },

        action_seg_dance_init = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.2,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.},
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.2,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0., 0.},
                },

            },
            foot_seq = {
                seq_len = 0,
            },
        },

        action_seg_dance_repeat = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 2,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.5,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.},
                },
                WP_2 = {
                    start_tic = 0.5,
                    duration = 0.5,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.},
                },
            },
            torso_seq = {
                seq_len = 2,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.5,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., -0.2, 0.2},
                },
                WP_2 = {
                    start_tic = 0.5,
                    duration = 0.5,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., -0.2, -0.2},
                },
            },
            foot_seq = {
                seq_len = 0,
            },
            waist_joint_seq = {
                seq_len = 2,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.5,
                    pos = -0.5,
                    vel = 0.,
                },
                WP_2 = {
                    start_tic = 0.5,
                    duration = 0.5,
                    pos = 0.5,
                    vel = 0.,
                },
            },
        },
        
        action_seg_dance_exit = {
            ref_foot = "center",
            move_foot_contact_state = true,
            com_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.7,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.},
                },
            },
            torso_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.7,
                    pos = {0.0, 0.0, 0.45},
                    lin_vel = {0., 0., 0.0},
                    rpy = {0., 0., 0.},
                },

            },
            foot_seq = {
                seq_len = 0,
            },
            waist_joint_seq = {
                seq_len = 1,
                WP_1 = {
                    start_tic = 0.,
                    duration = 0.7,
                    pos = 0.,
                    vel = 0.,
                },

            },
        },
    },

    commander1 = {
        record_data = true,
        n_joint = 22, 
        kp = kp_test,
        kd = kd_test,
        p = {
            0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
        },
        v = {
            0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
        },
        torq = {
            0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
        },
        wave_type = "slope",
        peak = ready_pos_test,
        period = 2.0,
        start_from_cur_state = true,
    },

    stance_wbc_common = {
        wbc_config_path = "common_wbc_path",
        -- ======================================= PVT模式相关 =======================================
    },
    stance_pvt = {
        urdf_path = "urdf_path",
        limit_pst = 0.98,
        damping_joint_kd = 2.0,
        ee_name = {
            "left_foot_link",
            "right_foot_link",
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        ee_point = {
             0.014,  0.000, -0.024,
             0.014,  0.000, -0.024,
            -0.012,  0.213,  0.000,
            -0.012, -0.213,  0.000,
        },
        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.6755, -2.2689, -2.1293,
            -3.3161, -1.5708, -2.2689, -0.1745,
            -3.0020, -0.4014, -1.0472, -0.0000, -1.8727, -1.3491,
            -3.0020, -1.5708, -1.0472, -0.0000, -1.8727, -1.3491,
        },
        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689, 0.1745,
            1.2217,  1.6755,  2.2689, 2.1293,
            2.2166,  1.5708,  1.0472,  2.2340,  1.3491,  1.3491,
            2.2166,  0.4014,  1.0472,  2.2340,  1.3491,  1.3491,
        },
        q_better_guess = {
            0.00,  0.00,
            0.25, -1.40, 0.00, -0.50,
            0.25,  1.40, 0.00,  0.50,
            -0.40, -0.03,  0.05,  0.70, -0.35,  0.03,
            -0.40,  0.03, -0.05,  0.70, -0.35, -0.03,
        },
        qdot_max = {
            13.0, 13.0,
            13.0, 13.0, 13.0, 13.0,
            13.0, 13.0, 13.0, 13.0,
            19.0, 19.0, 19.0, 19.0, 19.0, 19.0,
            19.0, 19.0, 19.0, 19.0, 19.0, 19.0,
        },
        constraint_type_names = {
            "ConstraintTypeFull",
            "ConstraintTypeFull",
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
        },
        constraint_body_names = {
            "left_foot_link",
            "right_foot_link",
            "left_forearm_pitch_link",
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        constratint_weight = {1.0, 1.0, 1.0, 0.05, 1.0, 0.05},
        lambda = 0.001,
        max_steps = 100,
        step_tol = 1e-7,
        vel_level_ik_weight_R = 1.0e-4,
        kp_joint = {
            5.0,  5.0,
            10.0, 10.0, 10.0, 10.0, 
            10.0, 10.0, 10.0, 10.0, 
            260., 160., 160., 260., 150.0, 150.0,
            260., 160., 160., 260., 150.0, 150.0,
        },
        kd_joint = {
            0.1, 0.1,
            0.1, 0.1, 0.1, 0.1, 
            0.1, 0.1, 0.1, 0.1, 
            4.0, 3.0, 3.0, 4.0, 1.5, 1.5,
            4.0, 3.0, 3.0, 4.0, 1.5, 1.5,
        },
        kp_joint_stand = {
            0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 150.0, 150.0,
            0.0, 0.0, 0.0, 0.0, 150.0, 150.0,
        },
        kd_joint_stand = {
            0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.5, 1.5,
            0.0, 0.0, 0.0, 0.0, 1.5, 1.5,
        },
        increase_pd_time = 0.03,
        decrease_pd_time = 0.02,
        leg_dof = 6,
        leg_joint_start_idx = {10, 16},
        use_feet_level_ctrl = true,
        use_feet_level_ctrl_support_foot_only = true,
        ankle_joint_types = {"pitch", "roll", "pitch", "roll"},
        ankle_joint_indexes = {14, 15, 20, 21},
        set_head = true,
        head_yaw_ref = 0.0,
        head_pitch_ref = 0.0,
        head_yaw_joint_idx = 0,
        head_pitch_joint_idx = 1,
    },
    stance_planner_wbc_convert = {
        feet_point_pos_local = {0.014,  0.000, -0.024,},
        left_hand_pos_local = {-0.012,  0.213,  0.000,},
        right_hand_pos_local = {-0.012, -0.213,  0.000,},
        left_foot_name = "left_foot_link",
        right_foot_name = "right_foot_link",
        left_hand_name = "left_forearm_pitch_link",
        right_hand_name = "right_forearm_pitch_link",
        joint_cnt = 22,

        use_waist_joint_planning = false,
        waist_joint_idx = 0,
        waist_name = "Waist",
        waist_pos_local = {0. ,0., 0.},
        urdf_path = "urdf_path",
    },

    dcm_wbc_common = {
        wbc_config_path = "common_wbc_path",
        -- ======================================= PVT模式相关 =======================================
    },
    dcm_pvt = {
        urdf_path = "urdf_path",
        limit_pst = 0.98,
        damping_joint_kd = 2.0,
        ee_name = {
            "left_foot_link",
            "right_foot_link",
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        ee_point = {
             0.014,  0.000, -0.024,
             0.014,  0.000, -0.024,
            -0.012,  0.213,  0.000,
            -0.012, -0.213,  0.000,
        },
        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.6755, -2.2689, -2.1293,
            -3.3161, -1.5708, -2.2689, -0.1745,
            -3.0020, -0.4014, -1.0472, -0.0000, -1.8727, -1.3491,
            -3.0020, -1.5708, -1.0472, -0.0000, -1.8727, -1.3491,
        },
        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689, 0.1745,
            1.2217,  1.6755,  2.2689, 2.1293,
            2.2166,  1.5708,  1.0472,  2.2340,  1.3491,  1.3491,
            2.2166,  0.4014,  1.0472,  2.2340,  1.3491,  1.3491,
        },
        q_better_guess = {
            0.00,  0.00,
            0.25, -1.40, 0.00, -0.50,
            0.25,  1.40, 0.00,  0.50,
            -0.40, -0.03,  0.05,  0.70, -0.35,  0.03,
            -0.40,  0.03, -0.05,  0.70, -0.35, -0.03,
        },
        qdot_max = {
            13.0, 13.0,
            13.0, 13.0, 13.0, 13.0,
            13.0, 13.0, 13.0, 13.0,
            19.0, 19.0, 19.0, 19.0, 19.0, 19.0,
            19.0, 19.0, 19.0, 19.0, 19.0, 19.0,
        },
        constraint_type_names = {
            "ConstraintTypeFull",
            "ConstraintTypeFull",
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
            "ConstraintTypePosition",
            "ConstraintTypeOrientation",
        },
        constraint_body_names = {
            "left_foot_link",
            "right_foot_link",
            "left_forearm_pitch_link",
            "left_forearm_pitch_link",
            "right_forearm_pitch_link",
            "right_forearm_pitch_link",
        },
        constratint_weight = {1.0, 1.0, 1.0, 0.05, 1.0, 0.05},
        lambda = 0.001,
        max_steps = 100,
        step_tol = 1e-7,
        vel_level_ik_weight_R = 1.0e-4,
        kp_joint = {
            5.0,  5.0,
            10.0, 10.0, 10.0, 10.0, 
            10.0, 10.0, 10.0, 10.0, 
            260., 160., 160., 260., 150.0, 150.0,
            260., 160., 160., 260., 150.0, 150.0,
        },
        kd_joint = {
            0.1, 0.1,
            0.1, 0.1, 0.1, 0.1, 
            0.1, 0.1, 0.1, 0.1, 
            4.0, 3.0, 3.0, 4.0, 1.5, 1.5,
            4.0, 3.0, 3.0, 4.0, 1.5, 1.5,
        },
        kp_joint_stand = {
            0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 150.0, 150.0,
            0.0, 0.0, 0.0, 0.0, 150.0, 150.0,
        },
        kd_joint_stand = {
            0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.5, 1.5,
            0.0, 0.0, 0.0, 0.0, 1.5, 1.5,
        },
        increase_pd_time = 0.03,
        decrease_pd_time = 0.02,
        leg_dof = 6,
        leg_joint_start_idx = {10, 16},
        use_feet_level_ctrl = true,
        use_feet_level_ctrl_support_foot_only = true,
        ankle_joint_types = {"pitch", "roll", "pitch", "roll"},
        ankle_joint_indexes = {14, 15, 20, 21},
        set_head = true,
        head_yaw_ref = 0.0,
        head_pitch_ref = 0.0,
        head_yaw_joint_idx = 0,
        head_pitch_joint_idx = 1,
    },
    dcm_planner_wbc_convert = {
        feet_point_pos_local = {0.014,  0.000, -0.024,},
        left_hand_pos_local = {-0.012,  0.213,  0.000,},
        right_hand_pos_local = {-0.012, -0.213,  0.000,},
        left_foot_name = "left_foot_link",
        right_foot_name = "right_foot_link",
        left_hand_name = "left_forearm_pitch_link",
        right_hand_name = "right_forearm_pitch_link",
        joint_cnt = 20,

        use_waist_joint_planning = false,
        waist_joint_idx = 0,
        waist_name = "Waist",
        waist_pos_local = {0. ,0., 0.},
        urdf_path = "urdf_path",
    },
    common_legged_ekf = {
        config_path = "common_wbc_path",
    },

    legged_estimate_convert = {
        config_path = "common_wbc_path",
        feet_point_pos_local = {0.014,  0.000, -0.024,},
    },

    planner_pvt_switch = {
        disabled_behavior = "disable",
    },

    intercept_motor_cmd = {
        intercepted_motor_indexes_ = {0,1,
                                    2,3,4,5,
                                    6,7,8,9},
        joint_cnt = 22,
        -- reduce torque peak
        enable_torque_peak_reduce = false,
        reduce_motors = {14, 15, 20, 21},
        reduce_planner_indices = {6, 8},
        torque_peak_thresh = 20.,
        reduce_periods = 10,
        reduce_strength = 0.5,
        max_cmd_torque = 12.,

        -- head kp kd intercept in rl mode
        head_intercept_indices = {1, 6, 8},
        -- head_kp_rl_mode = {40., 40.},
        -- head_kd_rl_mode = {0.65, 0.65},
        head_kp_rl_mode = {10., 10.},
        head_kd_rl_mode = {1., 1.},
    },

    rl_locomotion = {
        model_file = "lib/rma_locomotion_model",
        g_offset_x = 0.05,
    },

    rl_locomotion_old = {
        model_file = "lib/K1_run_tcn",
        model_file = "lib/K1_run_tcn_fast",
        push_recovery_model_file = "lib/K1_push_recovery_tcn",
        -- model_file = "lib/K1_run_tcn_push",
        -- model_file = "lib/K1_run_tcn_test",
        odom_model_file = "lib/rma_run_odom",
        urdf_file = "urdf_file",
        robot_fall_angle = 0.4,
        robot_adjust_angle = 0.3,
        g_offset_x = -0.04,
        vel_offset_x = 0.0,
        vel_offset_x_low_speed = 0.0,
        g_offset_x_low_speed = 0.0,
        low_vel_cnt_ = 60,
        vel_offset_y = 0.0,
        ankle_pitch_kp = 40.,
        ankle_pitch_kd = 1.,
        ankle_roll_kp = 40.,
        ankle_roll_kd = 1.,
        forward_vel_max = 1.2,
        backward_vel_max = 0.7,
        cmd_x_dead_zone = 0.1,
        cmd_y_dead_zone = 0.1,
        phase_rate = 1.9,
        action_scale = 1.0,

        stop_cnt_threshold = 60,

        is_serial = false,

        -- Fixed arm position feature
        fix_arm_position = false,  -- Initially disabled, can be toggled at runtime
        fixed_arm_position = {
            -0.71114896, -1.17624166, 0.13659583, -2.32278097,  -- 左臂 (4个关节)
            -0.81616835, 1.37621103, 0.40890744, 2.21507698,    -- 右臂 (4个关节)
        },
        arm_interpolation_duration = 1.0,  -- Interpolation duration in seconds
    },

    rl_locomotion_run = {
        is_serial = false,
        rt_thread = false,

        max_vel_cmd = {0.65, 0.45, 1.6}, -- k1 leg amp
        min_vel_cmd = {-0.7, -0.45, -1.6},

        max_vel_cmd_incre = {1.3, 1.0, 2.0},
        max_vel_cmd_decre = {1.3, 1.0, 2.0},
        vel_cmd_dead_zone = {0.2, 0.2, 0.4},

        vel_cmd_dead_zone_start = {0.25, 0.25, 0.25},
        vel_cmd_dead_zone_end = {0.9, 0.9, 0.9},

        vel_cmd_map_start_pos = {0.55, 0.2, 0.6},
        vel_cmd_map_end_pos = {0.65, 0.45, 1.5},
        vel_cmd_map_start_neg = {0.4, 0.2, 0.6},
        vel_cmd_map_end_neg = {0.7, 0.45, 1.5},

        -- turn_threshold = 0.3;
        dead_threshold = 0.8,
        stop_cnt_threshold = 50,
        side_threshold = 0.15,
        arm_swing_gain = 0.5,

        -- 脚部调整检测阈值：越大越不易触发
        foot_adjust_distance = 0.25,       -- y 方向脚间距超过此值触发（脚过远）
        foot_adjust_x_threshold = 0.10,    -- x 方向脚差值超过此值触发
        foot_adjust_y_min = 0.05,          -- y 方向脚间距小于此值触发（脚过近）

        adjust_cooldown_ticks = 100,
        adjust_min_zero_vel_ticks = 10,

        action_filter = 0.5,
        g_offset_x = 0.015,
        g_offset_y = 0.0,
        vel_offset_x = 0.0,
        vel_offset_y = 0.0,
        clip_action = 100.0,
        clip_observation = 100.0,

        -- 双臂肩 pitch 足够前伸且双肘「向前屈」时，对观察中 proj_g.x 叠加（与 g_offset_x 可叠加）；0 关闭
        g_offset_x_arm_front_delta = 0.02,
        arm_front_shoulder_pitch_min = 0.3,
        arm_front_left_elbow_max = -0.55,
        arm_front_right_elbow_min = 0.55,

        vel_c_xy_pos = 0.08,
        vel_c_xy_neg = 0.01,
        vel_c_x_yaw_pos = 0.9,
        vel_c_x_yaw_neg = 0.3,
        vel_c_y_yaw_pos = 0.3,
        vel_c_y_yaw_neg = 0.3,

        -- 2026-01-21_17-15-28, 2026-01-07_15-29-45, 2025-12-20_16-13-16
        mlp_model_file = "lib/exported_policy_k1_leg_front.bin",
        -- mlp_model_file = "lib/exported_policy_k1_leg.bin",
        mlp_action_filter = 0.8,
        mlp_clip_action = 100.0,

        -- 2026-01-21_17-15-28, 2026-01-06_16-57-23
        mlp_left_model_file = "lib/exported_policy_k1_leg_left.bin",
        mlp_left_action_filter = 0.8,
        mlp_left_clip_action = 100.0,

        -- 2026-01-21_17-15-28, 2026-01-06_16-16-02
        mlp_right_model_file = "lib/exported_policy_k1_leg_right.bin",
        mlp_right_action_filter = 0.8,
        mlp_right_clip_action = 100.0,

        -- 2025-12-20_16-13-16, 2026-01-14_14-58-20, 2026-01-08_17-04-01, 2026-01-21_17-15-28
        mlp_back_model_file = "lib/exported_policy_k1_leg_back.bin",
        mlp_back_action_filter = 0.8,
        mlp_back_clip_action = 100.0,

        -- 2026-03-03_17-58-45, 2026-03-03_17-58-11
        mlp_turn_left_model_file = "lib/exported_policy_k1_leg_turn_left.bin",
        mlp_turn_left_action_filter = 0.8,
        mlp_turn_left_clip_action = 100.0,

        -- 2026-03-03_17-58-59
        mlp_turn_right_model_file = "lib/exported_policy_k1_leg_turn_right.bin",
        mlp_turn_right_action_filter = 0.8,
        mlp_turn_right_clip_action = 100.0,

        kp = {
            20., 20.,
            15., 15., 15., 15.,
            15., 15., 15., 15.,
            100., 100., 100., 100., 50., 50.,
            100., 100., 100., 100., 50., 50.,
        },
        kd = {
            2.5, 2.5,
            1.5, 1.5, 1.5, 1.5,
            1.5, 1.5, 1.5, 1.5,
            2.5, 2.0, 2.0, 2.0, 1.5, 1.5, --real
            2.5, 2.0, 2.0, 2.0, 1.5, 1.5,
            -- 2., 2., 2., 2., 0.1, 0.1, --sim
            -- 2., 2., 2., 2., 0.1, 0.1,
        },
        max_torque = {      -- for serial joints
            7., 7.,
            10., 10., 10., 10.,
            10., 10., 10., 10.,
            35., 25., 25., 50., 25., 25.,
            35., 25., 25., 50., 25., 25.,
        },
        max_torque_ankle_motors = {50.0, 50.0, 50.0, 50.0},
        
        -- Fixed arm position feature
        fix_arm_position = false,  -- Initially disabled, can be toggled at runtime
        arm_start_index = 2,
        arm_num = 4,
        fixed_arm_position = {
            -0.81616835, -1.37621103, 0.40890744, -2.21507698,  -- 左臂 (4个关节)
            -0.81616835, 1.37621103, 0.40890744, 2.21507698,    -- 右臂 (4个关节)
        },
        arm_interpolation_duration = 1.0,  -- Interpolation duration in seconds
    },

    amp_locomotion_run = {
        is_serial = false,
        rt_thread = false,

        urdf_file = "urdf_file",
        foot_adjust_distance = 0.23,

        adjust_cooldown_ticks = 100,
        adjust_min_zero_vel_ticks = 15,

        max_vel_cmd = {1.25, 0.4, 1.8}, -- k1 amp
        min_vel_cmd = {-0.75, -0.5, -1.8},

        -- max_vel_cmd = {1.15, 0.55, 1.4}, -- k1 amp
        -- min_vel_cmd = {-0.75, -0.34, -1.4},

        max_vel_cmd_incre = {1.3, 1.2, 2.0},
        max_vel_cmd_decre = {2.0, 1.2, 2.0},
        vel_cmd_dead_zone = {0.2, 0.2, 0.1},

        vel_cmd_dead_zone_start = {0.1, 0.1, 0.1},
        vel_cmd_dead_zone_end = {0.9, 0.9, 0.9},

        vel_cmd_map_start_pos = {0.4, 0.2, 0.2},
        vel_cmd_map_end_pos = {1.25, 0.4, 1.8},
        vel_cmd_map_start_neg = {0.2, 0.2, 0.2},
        vel_cmd_map_end_neg = {0.75, 0.5, 1.8},

        turn_threshold = 0.3;
        dead_threshold = 0.8,
        stop_cnt_threshold = 60,

        mlp_model_file = "lib/exported_policy_k1.bin", -- 2025-12-03_11-23-39

        mlp_g_offset_x = 0.0,
        mlp_g_offset_y = 0.0,
        mlp_vel_offset_x = 0.0,
        mlp_vel_offset_y = 0.0,
        mlp_clip_action = 100.0,
        mlp_arm_action_fix_offset = 0.2,

        mlp_action_filter = 0.8,
        mlp_clip_observation = 100.0,

        mlp_side_model_file = "lib/exported_policy_k1_side.bin", -- 2026-01-09_15-39-41

        mlp_side_action_filter = 0.8,
        mlp_side_clip_action = 100.0,

        mlp_turn_model_file = "lib/exported_policy_k1_turn.bin", -- 2026-01-08_17-30-51

        mlp_turn_action_filter = 0.8,
        mlp_turn_clip_action = 100.0,

        kp = {
            20., 20.,
            20., 20., 20., 20.,
            20., 20., 20., 20.,
            100., 100., 100., 100., 50., 50.,
            100., 100., 100., 100., 50., 50.,
        },
        kd = {
            2., 2.,
            2.5, 2.5, 2.5, 2.5,
            2.5, 2.5, 2.5, 2.5,
            2.0, 2.0, 2.0, 2.0, 1., 1., --real
            2.0, 2.0, 2.0, 2.0, 1., 1.,
            -- 2., 2., 2., 2., 0.1, 0.1, --sim
            -- 2., 2., 2., 2., 0.1, 0.1,
        },
        max_torque = {      -- for serial joints
            7., 7.,
            10., 10., 10., 10.,
            10., 10., 10., 10.,
            35., 25., 25., 50., 25., 25.,
            35., 25., 25., 50., 25., 25.,
        },
        max_torque_ankle_motors = {50.0, 50.0, 50.0, 50.0},
    },

    rl_traj_fdr = {
        model_path = "lib/K1/k1_fdr.pt",
        faceup_joint_traj_path = "lib/K1/faceup_joint.pt";
        faceup_grav_traj_path = "lib/K1/faceup_grav.pt";
        faceup_height_traj_path = "lib/K1/faceup_height.pt";
        facedown_joint_traj_path = "lib/K1/facedown_joint.pt";
        facedown_grav_traj_path = "lib/K1/facedown_grav.pt";
        facedown_height_traj_path = "lib/K1/facedown_height.pt";
        NUM_DOFS= 22,
        NUM_ACTS= 22,
        NUM_OBS= 100,
        l_hip = 0.096,
        leg_length = 0.435,
        smooth_factor = 0.,
        zero_joint_ids = {0, 1,12, 15, 18, 21},
        left_hip_roll_idx = 11,
        right_hip_roll_idx = 17,
        possible_intersection_z_min = -0.415,
        possible_intersection_z_max = -0.16,
        fallen_pitch_threshold = 1.0,
        fallen_ang_vel_threshold = 0.25,
        fail_grav_threshold = 0.5,
        wait_time = 0.5,
        prep_v_limit = 10.,
        
        dof_velocity_scale = 0.1,

        torque_limit = {
            70., 70.,
            14., 14., 14., 14.,
            14., 14., 14., 14.,
            60., 25., 30., 60., 24., 15.,
            60., 25., 30., 60., 24., 15.,
        },

        kp_0 = {
            70.0, 70.0,
            53.0, 40.0, 40.0, 55.0,
            53.0, 40.0, 40.0, 55.0,
            90., 70., 70., 90., 30., 30.,
            90., 70., 70., 90., 30., 30.,
        },

        kp_1 = {
            70.0, 70.0,
            53.0, 40.0, 40.0, 55.0,
            53.0, 40.0, 40.0, 55.0,
            90., 70., 70., 90., 30., 30.,
            90., 70., 70., 90., 30., 30.,
        },

        kd = {
            1.5, 1.5, 
            2.5, 3., 2.5, 2.5, 
            2.5, 3., 2.5, 2.5, 
            5., 5., 5., 5., 5., 5.,
            5., 5., 5., 5., 5., 5.,
            -- 2., 2., 2., 2., 
            -- 2., 2., 2., 2., 
            -- 3., 3., 3., 3., 0.5, 0.5,
            -- 3., 3., 3., 3., 0.5, 0.5
            1.5, 3., 2.5, 2.5, 
            1.5, 3., 2.5, 2.5, 
            5., 5., 5., 5., 5., 5.,
            5., 5., 5., 5., 5., 5.,
            -- 2., 2., 2., 2., 
            -- 2., 2., 2., 2., 
            -- 3., 3., 3., 3., 0.5, 0.5,
            -- 3., 3., 3., 3., 0.5, 0.5
        },

        reference = {
            0., 0.,
            0., -1.45, 0., 0.,
            0.,  1.45, 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,},

        q_min = {
            -1.0472, -0.3491,
            -3.3161, -1.7453, -2.2689, -2.4435,
            -3.3161, -1.5708, -2.2689, -0.0000,
            -3.0, -0.2, -1.0472, -0.0000, -0.8727, -0.4363,
            -3.0, -1.5708, -1.0472, -0.0000, -0.8727, -0.4363,
        },
        q_max = {
            1.0472,  0.8552,
            1.2217,  1.5708,  2.2689,  0.0000,
            1.2217,  1.7453,  2.2689,  2.4435,
            2.2166,  1.5708,  1.0472,  2.1817,  0.3491,  0.4363,
            2.2166,  0.2,  1.0472,  2.1817,  0.3491,  0.4363,
        },
    },

    rl_traj_fdr_state_portal_collect = {
        portal_pair_key = "portal_for_rl_traj_fdr_state",
    },
    rl_traj_fdr_state_portal_publish = {
        portal_pair_key = "portal_for_rl_traj_fdr_state",
    },

    rl_locomotion_fallen_portal_collect = {
        portal_pair_key = "portal_for_rl_locomotion_fallen",
    },
    rl_locomotion_fallen_portal_publish = {
        portal_pair_key = "portal_for_rl_locomotion_fallen",
    },

    rl_locomotion_run_fallen_portal_collect = {
        portal_pair_key = "portal_for_rl_locomotion_run_fallen",
    },
    rl_locomotion_run_fallen_portal_publish = {
        portal_pair_key = "portal_for_rl_locomotion_run_fallen",
    },

    rl_locomotion_old_fallen_portal_collect = {
        portal_pair_key = "portal_for_rl_locomotion_run_fallen",
    },
    rl_locomotion_old_fallen_portal_publish = {
        portal_pair_key = "portal_for_rl_locomotion_run_fallen",
    },

    visual_kick_old = {
         model_file = "lib/K1_whole_body_loco_kick",
         chase_kick_model_file = "lib/K1_whole_body_chase_kick",
         ankle_kd = 1.,
    },

    visual_kick_v1 = {
        is_serial = false,
        model_file = "lib/exported_policy_k1.bin",
        kick_model_files = {
            "lib/k1_kick_policy_2_v1.bin",
            "lib/k1_kick_policy_264_v1.bin",
            "lib/k1_kick_policy_192_v1.bin",
            "lib/k1_kick_policy_138_v1.bin",
        },

        g_offset_x = 0.0,
        max_vel_cmd = {2.0, 0.6, 2.0},
        min_vel_cmd = {-1.0, -0.6, -2.0},
        max_vel_cmd_incre = {2.0, 1.0, 2.0},
        vel_cmd_dead_zone = {0.1, 0.2, 0.2},
        dead_threshold = 0.8,
        ball_pos_offset = {
            {0., 0.,},
            {0., 0.,},
            {0., 0.,},
            {0., 0.,},
        },
        kick_yaw_offset = {
            0.3,
            -0.3,
            -0.3,
            0.5,
        },
        use_certain_kick_index = -1,
        use_certain_kick_power = -1.,

        kp = {
            20., 20.,
            20., 20., 20., 20.,
            20., 20., 20., 20.,
            100., 100., 100., 100., 50., 50.,
            100., 100., 100., 100., 50., 50.,
        },
        kd = {
            2., 2.,
            2., 2., 2., 2.,
            2., 2., 2., 2.,
            2., 2., 2., 2., 1., 1., --real
            2., 2., 2., 2., 1., 1.,
            -- 2., 2., 2., 2., 0.5, 0.5, --sim
            -- 2., 2., 2., 2., 0.5, 0.5,
        },
        max_torque = {      -- for serial joints
            7., 7.,
            10., 10., 10., 10.,
            10., 10., 10., 10.,
            35., 25., 25., 50., 25., 25.,
            35., 25., 25., 50., 25., 25.,
        },
        max_torque_ankle_motors = {50.0, 50.0, 50.0, 50.0},
        rt_thread = false,
        clip_action = 100.0,
        clip_observation = 100.0,
        
        -- Fixed arm position feature
        fix_arm_position = false,  -- Initially disabled, can be toggled at runtime
        fixed_arm_position = {
            -0.71114896, -1.17624166, 0.13659583, -2.32278097,  -- 左臂 (4个关节)
            -0.81616835, 1.37621103, 0.40890744, 2.21507698,    -- 右臂 (4个关节)
        },
        arm_interpolation_duration = 1.0,  -- Interpolation duration in seconds
    },

    visual_kick = {
        is_serial = false,
        pub_traj_for_sysid = false,
        kp = {
            20., 20.,
            20., 20., 20., 20.,
            20., 20., 20., 20.,
            100., 100., 100., 100., 50., 50.,
            100., 100., 100., 100., 50., 50.,
        },
        kd = {
            2., 2.,
            2., 2., 2., 2.,
            2., 2., 2., 2.,
            2., 2., 2., 2., 1., 1., --real
            2., 2., 2., 2., 1., 1.,
        },
        max_torque = {      -- for serial joints
            7., 7.,
            10., 10., 10., 10.,
            10., 10., 10., 10.,
            35., 25., 25., 50., 25., 25.,
            35., 25., 25., 50., 25., 25.,
        },
        max_torque_ankle_motors = {50.0, 50.0, 50.0, 50.0},

        dead_threshold = 0.8,
        stop_cnt_threshold = 60,
        foot_adjust_distance = 0.25,
        kick_timeout_dur = 0.8,
        urdf_file = "urdf_file",

        num_obs_stacking = 10,
        action_scale = 0.25,

        locomotion = {
            clip_observation = 100.,
            clip_action = 100.,
            model_files = {
                "lib/exported_policy_k1.bin", -- 2025-12-03_11-23-39
                "lib/exported_policy_k1_side.bin", -- 2026-01-09_15-39-41
                "lib/exported_policy_k1_turn.bin", -- 2026-01-08_17-30-51
            },

            max_vel_cmd = {1.5, 0.4, 1.8},
            min_vel_cmd = {-0.7, -0.4, -1.8},
            vel_cmd_dead_zone = {0.2, 0.2, 0.1},

            max_vel_cmd_incre = {1.3, 1.2, 2.0},
            max_vel_cmd_decre = {1.6, 1.2, 2.0},
            update_interval = 0.02,
        },

        ball_shooting = {
            clip_observation = 100.,
            clip_action = 10.,
            model_files = {
                "lib/k1_shoot_policy_2.bin",
                "lib/k1_shoot_policy_264.bin",
                "lib/k1_shoot_policy_192.bin",
                "lib/k1_shoot_policy_0109_0.bin",
                "lib/k1_shoot_policy_0109_2.bin",
            },
            ball_pos_offset = {
                {0., -0.05,},
                {0., 0.05,},
                {0., 0.08,},
                {0.05, -0.05,},
                {0.05, 0.02,},
            },
            kick_yaw_offset = {
                0.1,
                -0.1,
                0.,
                0.1,
                0.2,
            },
            use_certain_kick_index = -1,
        },

        ball_passing = {
            clip_observation = 100.,
            clip_action = 10.,
            model_files = {
                "lib/k1_passing_policy_0.bin",
                "lib/k1_passing_policy_1.bin",
                "lib/k1_passing_policy_2.bin",
            },
            ball_pos_offset = {
                {0., 0.,},
                {0., 0.,},
                {0., 0.,},
            },
            kick_yaw_offset = {
                0.,
                0.,
                0.,
            },
            use_certain_kick_index = -1,
            use_certain_kick_power = -1.,
        },

        model_file = "lib/exported_policy_k1.bin",
        kick_model_files = {
            "lib/k1_shoot_policy_2.bin",
            "lib/k1_shoot_policy_264.bin",
            "lib/k1_shoot_policy_192.bin",
            "lib/k1_shoot_policy_0109_0.bin",
            "lib/k1_shoot_policy_0109_2.bin",
            "lib/k1_passing_policy_0.bin",
            "lib/k1_passing_policy_1.bin",
            "lib/k1_passing_policy_2.bin",
        },
    },

    amp_locomotion_run_fallen_portal_collect = {
        portal_pair_key = "portal_for_rl_locomotion_run_fallen",
    },
    amp_locomotion_run_fallen_portal_publish = {
        portal_pair_key = "portal_for_rl_locomotion_run_fallen",
    },

    custom_traj = require('custom_traj'),

    custom_traj_finished_portal_collect = {
        portal_pair_key = "portal_for_custom_traj_finished",
    },
    custom_traj_finished_portal_publish = {
        portal_pair_key = "portal_for_custom_traj_finished",
    },

    fall_down_recovery = {
        left_hip_roll_idx = 11,
        right_hip_roll_idx = 17,
        face_down_fallen_pitch_threshold = 1.0,
        face_up_fallen_pitch_threshold = -1.0,
        is_ready_ori_abs_threshold = 0.2,
        fallen_ang_vel_threshold = 0.25,
        falling_cumulative_time_threshold = 0.4,
        is_ready_cumulative_time_threshold = 0.7,
        l_hip = 0.096,
        possible_intersection_z_min = -0.415,
        possible_intersection_z_max = -0.16,
        leg_length = 0.435,
    },

    stand_up_type_idx_portal_collect = {
        portal_pair_key = "portal_for_stand_up_type_idx",
    },
    stand_up_type_idx_portal_publish = {
        portal_pair_key = "portal_for_stand_up_type_idx",
    },

    fdr_state_portal_collect = {
        portal_pair_key = "portal_for_fdr_state",
    },
    fdr_state_portal_publish = {
        portal_pair_key = "portal_for_fdr_state",
    },

    is_recovery_avalible_portal_collect = {
        portal_pair_key = "portal_for_is_recovery_avalible",
    },
    is_recovery_avalible_portal_publish = {
        portal_pair_key = "portal_for_is_recovery_avalible",
    },

    odom_estimator = {
        odom_model_file = "lib/K1_odom_model_move",

        odom_model_files = {
            -- K1_leg_odom_model_20260331_131642 K1_leg_odom_model_20260403_230740 K1_leg_odom_model_20260409_174936
            ["3"] = "lib/K1_leg_odom_model_20260413_201443",
            -- K1_odom_model_20260128_152713 K1_odom_model_20260403_230729 K1_odom_model_20260408_175440 K1_odom_model_20260413_201427
            ["20"] = "lib/K1_odom_model_20260128_152713",
            ["4"] = "lib/K1_odom_model",
            ["12"] = "lib/K1_odom_model_visionkick",
        },

        num_obs = 39,
        num_stack = 50,
        start_dof_idx = 10,
        num_dof = 12,
        input_mean = {
            9.8197e-01, -4.6688e-03,  1.1839e-04,  4.6722e-03,  9.8197e-01,
            -7.6649e-05, -6.6154e-05,  1.0093e-04,  9.9953e-01,  3.6994e-04,
            1.1922e-03,  9.5861e-03, -1.0216e-01,  5.1952e-02,  9.7849e+00,
            -2.8482e-01,  7.0854e-03,  1.3864e-02,  3.9954e-01, -1.6429e-01,
            1.8119e-02, -2.7976e-01, -2.8276e-03, -1.7625e-02,  4.0313e-01,
            -1.4074e-01, -1.4925e-02, -4.6144e-03, -5.3214e-03,  4.5190e-03,
            -4.5163e-03,  2.1489e-03, -4.6973e-03,  4.5580e-03, -3.6131e-03,
            3.8598e-03,  3.9056e-03,  2.9058e-03,  2.1452e-02
        },
        input_std = {
            6.0042e-02, 1.7787e-01, 2.1745e-02, 1.7788e-01, 6.0029e-02, 2.1570e-02,
            2.1687e-02, 2.1628e-02, 8.4860e-04, 3.2136e-01, 2.7714e-01, 6.7120e-01,
            2.6908e+00, 2.6318e+00, 4.9752e+00, 2.4326e-01, 6.9868e-02, 8.5098e-02,
            3.3803e-01, 1.3634e-01, 6.8686e-02, 2.3849e-01, 7.2647e-02, 8.4776e-02,
            3.3425e-01, 1.4620e-01, 6.6263e-02, 1.6190e+00, 7.0481e-01, 7.2105e-01,
            3.0032e+00, 1.4236e+00, 1.2506e+00, 1.5869e+00, 7.5354e-01, 7.1201e-01,
            2.9700e+00, 1.5010e+00, 1.2543e+00
        },
        output_mean = {1.3219e-03,  2.3894e-04},
        -- output_mean = {-3.86008760e-04, 6.54618416e-05},
        output_std = {0.0070, 0.0052},
        -- output_std = {6.93930592e-03, 5.00399433e-03},
        output_dead_zone = {2.0e-3, 6.0e-4},
        -- output_dead_zone = {5.0e-3, 1.0e-3},

        input_means = {
            ["3"] = {
                -- 9.78279471e-01, 6.50645839e-03, -2.50402343e-04, -6.55229529e-03, 9.78299677e-01, -1.84831224e-04, 2.76737352e-04, 5.34349878e-04,
                -- 9.98906016e-01, -2.11347015e-05, -5.14916005e-03, -1.41573297e-02, 4.46738452e-01, 2.43610218e-02, 9.80376816e+00, -3.02063465e-01,
                -- 3.34200189e-02, 8.83366819e-03, 6.59265995e-01, -1.96295142e-01, 1.71624869e-03, -3.27854156e-01, -6.37909174e-02, -1.56079987e-02,
                -- 6.78453267e-01, -1.59827396e-01, -2.48714015e-02, -5.20731229e-03, -3.26375663e-03, 3.66677716e-03, -5.25123440e-03, -1.74982718e-03,
                -- 5.94755774e-03, 4.23451141e-03, -6.62685931e-03, 5.66180842e-03, 4.15904401e-03, 3.24356981e-04, 1.27277225e-02

                9.79415357e-01, 6.26477599e-03, -2.00733950e-04, -6.31304877e-03, 9.79436219e-01, -1.69365347e-04, 2.34989420e-04, 4.97353903e-04,
                9.98921454e-01, -9.57513330e-05, -4.59951628e-03, -1.35878548e-02, 4.52997088e-01, 2.33504586e-02, 9.80313778e+00, -3.03142369e-01,
                3.10873594e-02, 1.13054588e-02, 6.64343715e-01, -1.96826875e-01, 3.16174561e-03, -3.31073821e-01, -6.21938184e-02, -1.85044874e-02,
                6.83680236e-01, -1.60410240e-01, -2.53461991e-02, -4.97180782e-03, -2.92627560e-03, 3.83111672e-03, -6.15902292e-03, -1.50854327e-03,
                5.84055949e-03, 4.83914278e-03, -6.56535057e-03, 5.70646022e-03, 3.32708005e-03, 5.09927166e-04, 1.27351722e-02
            },
            ["20"] = {
                -- 9.85285640e-01, -8.90479307e-04, 4.59404509e-05, 8.27998272e-04, 9.85366762e-01, 9.38099765e-05, 8.82093082e-05, -7.22590266e-05,
                -- 9.98908341e-01, -5.03931777e-04, 1.60859933e-03, 1.15300796e-03, -2.43149310e-01, 1.46284536e-01, 9.82568550e+00, -2.99048215e-01,
                -- -7.49119325e-04, -3.82888615e-02, 5.11910021e-01, -1.44259080e-01, -9.65378527e-03, -3.54528159e-01, -4.56111580e-02, -3.66725847e-02,
                -- 5.96284389e-01, -1.48581073e-01, 3.03599937e-03, 8.31935497e-04, -4.50637052e-03, 7.11545302e-03, -3.84819554e-03, -1.11572454e-02,
                -- 8.82087182e-03, 3.62427090e-03, -6.67536445e-03, 3.55607970e-03, 1.46512529e-02, -6.58324081e-03, 1.19672632e-02

                -- 9.85952616e-01, -7.71300169e-04, 6.36727200e-05, 7.08347245e-04, 9.86032844e-01, 1.17557771e-04, 6.19785569e-05, -9.50737158e-05,
                -- 9.98931885e-01, -4.47545579e-04, 1.32601301e-03, 9.17395810e-04, -2.26926759e-01, 1.50122866e-01, 9.82481766e+00, -2.99268872e-01,
                -- -9.95494309e-04, -3.72676440e-02, 5.12924194e-01, -1.43319309e-01, -8.94477125e-03, -3.54041994e-01, -4.53962870e-02, -3.73556130e-02,
                -- 5.95697224e-01, -1.49345845e-01, 2.83756456e-03, 1.15357654e-03, -4.48156474e-03, 6.88454974e-03, -4.43161000e-03, -1.09752193e-02,
                -- 8.75962898e-03, 3.64700169e-03, -6.75836485e-03, 3.45201907e-03, 1.52845401e-02, -6.57716207e-03, 1.18756285e-02

                9.8197e-01, -4.6688e-03,  1.1839e-04,  4.6722e-03,  9.8197e-01,
                -7.6649e-05, -6.6154e-05,  1.0093e-04,  9.9953e-01,  3.6994e-04,
                1.1922e-03,  9.5861e-03, -1.0216e-01,  5.1952e-02,  9.7849e+00,
                -2.8482e-01,  7.0854e-03,  1.3864e-02,  3.9954e-01, -1.6429e-01,
                1.8119e-02, -2.7976e-01, -2.8276e-03, -1.7625e-02,  4.0313e-01,
                -1.4074e-01, -1.4925e-02, -4.6144e-03, -5.3214e-03,  4.5190e-03,
                -4.5163e-03,  2.1489e-03, -4.6973e-03,  4.5580e-03, -3.6131e-03,
                3.8598e-03,  3.9056e-03,  2.9058e-03,  2.1452e-02
            },
        },

        input_stds = {
            ["3"] = {
                -- 6.90585747e-02, 1.92461520e-01, 3.34162675e-02, 1.92488611e-01, 6.90509006e-02, 3.26649360e-02, 3.32506448e-02, 3.28294709e-02,
                -- 1.77835743e-03, 2.62257457e-01, 2.05636889e-01, 4.86646891e-01, 2.44066072e+00, 2.29302359e+00, 3.42182469e+00, 2.17631802e-01,
                -- 1.14678629e-01, 1.02205440e-01, 3.57209265e-01, 1.27534300e-01, 5.61906844e-02, 2.37343475e-01, 1.12461545e-01, 1.19910099e-01,
                -- 3.79954934e-01, 1.29132301e-01, 5.54729179e-02, 1.67221689e+00, 8.18853498e-01, 6.76628470e-01, 3.26863003e+00, 1.39742231e+00,
                -- 5.29354870e-01, 1.76356876e+00, 8.63903761e-01, 6.82512939e-01, 3.27928734e+00, 1.35252428e+00, 6.19255185e-01

                6.72571436e-02, 1.87300503e-01, 3.31866965e-02, 1.87326714e-01, 6.72496781e-02, 3.24285738e-02, 3.30289826e-02, 3.25856097e-02,
                1.75941782e-03, 2.60515302e-01, 2.06486538e-01, 4.78751808e-01, 2.44428945e+00, 2.27446413e+00, 3.40500355e+00, 2.18898550e-01,
                1.12553492e-01, 1.00842521e-01, 3.59605610e-01, 1.27425104e-01, 5.58350794e-02, 2.38842219e-01, 1.10710785e-01, 1.18295342e-01,
                3.82159799e-01, 1.29707098e-01, 5.47677316e-02, 1.68481886e+00, 8.02563727e-01, 6.69579387e-01, 3.30160999e+00, 1.40088630e+00,
                5.23427427e-01, 1.77683628e+00, 8.50794494e-01, 6.75171554e-01, 3.30991054e+00, 1.35783780e+00, 6.11487508e-01
            },
            ["20"] = {
                -- 4.89042625e-02, 1.60160944e-01, 3.41782570e-02, 1.60147741e-01, 4.89149615e-02, 3.18041071e-02, 3.42415608e-02, 3.17359120e-02,
                -- 1.68950798e-03, 3.05184066e-01, 2.31306255e-01, 4.12735134e-01, 2.55182242e+00, 2.53684163e+00, 4.57672071e+00, 2.08985597e-01,
                -- 6.19063228e-02, 6.73155263e-02, 3.26533914e-01, 7.72818327e-02, 4.84547839e-02, 2.38298327e-01, 6.05884716e-02, 5.86651042e-02,
                -- 3.76312792e-01, 8.76366347e-02, 4.76540551e-02, 1.93035758e+00, 5.91916919e-01, 5.64509869e-01, 3.25210714e+00, 8.26705337e-01,
                -- 5.46737671e-01, 2.14275908e+00, 6.38585389e-01, 5.35371006e-01, 3.58597636e+00, 9.77739096e-01, 6.24015331e-01

                -- 4.77841645e-02, 1.56427175e-01, 3.38253193e-02, 1.56414956e-01, 4.77943644e-02, 3.14404294e-02, 3.38830985e-02, 3.13782319e-02,
                -- 1.66718487e-03, 3.03049207e-01, 2.31644049e-01, 4.07754332e-01, 2.56382418e+00, 2.53805661e+00, 4.59482431e+00, 2.08059147e-01,
                -- 6.08081669e-02, 6.65775388e-02, 3.25077236e-01, 7.79518262e-02, 4.77379933e-02, 2.35836238e-01, 5.98032698e-02, 5.79317436e-02,
                -- 3.72798264e-01, 8.78654420e-02, 4.71328162e-02, 1.92702293e+00, 5.84042370e-01, 5.58311224e-01, 3.25650072e+00, 8.32930207e-01,
                -- 5.41386247e-01, 2.12452435e+00, 6.30300820e-01, 5.30833960e-01, 3.57134628e+00, 9.86114323e-01, 6.17751122e-01

                6.0042e-02, 1.7787e-01, 2.1745e-02, 1.7788e-01, 6.0029e-02, 2.1570e-02,
                2.1687e-02, 2.1628e-02, 8.4860e-04, 3.2136e-01, 2.7714e-01, 6.7120e-01,
                2.6908e+00, 2.6318e+00, 4.9752e+00, 2.4326e-01, 6.9868e-02, 8.5098e-02,
                3.3803e-01, 1.3634e-01, 6.8686e-02, 2.3849e-01, 7.2647e-02, 8.4776e-02,
                3.3425e-01, 1.4620e-01, 6.6263e-02, 1.6190e+00, 7.0481e-01, 7.2105e-01,
                3.0032e+00, 1.4236e+00, 1.2506e+00, 1.5869e+00, 7.5354e-01, 7.1201e-01,
                2.9700e+00, 1.5010e+00, 1.2543e+00
            },
        },

        output_means = {
            ["3"] = {
                -- 1.37450616e-03, 1.34192189e-04

                1.30639458e-03, 2.63700174e-04
            },
            ["20"] = {
                -- 1.42884429e-03, 2.31907325e-04
                -- 1.35393464e-03, 1.91885658e-04
                1.3219e-03,  2.3894e-04
            },
        },

        output_stds = {
            ["3"] = {
                -- 6.78905705e-03, 5.04144561e-03

                6.69166865e-03, 4.91134496e-03
            },
            ["20"] = {
                -- 6.81900512e-03, 4.80240211e-03
                -- 6.58016093e-03, 4.78790700e-03
                0.0070, 0.0052
            },
        },

        output_dead_zones = {
            ["3"] = {
                7.0e-4, 5.0e-5
            },
            ["20"] = {
                -- 5.0e-4, 5.0e-4
                2.0e-3, 6.0e-4
            },
        },
    },

    scheduler = {
        record_interval = 10
    },
    drawer = {
        drawer_backend = "NoBackend",
    },
    dds = {
        target_ip =  "192.168.10.101, 127.0.0.1",
    },

    record_manager = {
        record_backends = {
            "LCM",
            -- "LCM2DevSuite",
        },
        publish_channel_name = "CHANNEL_RECORD_TO_DEV_SUITE_WHITE",
        real_robot = true,
        server_url = "192.168.1.199:8080",
        robot_name = "test_joints",
        sampling_rate = 0.5,
        chart_keys = {
            -- global
            "global/t_call_ms",
            "global/t_exec_ms",
            "global/t_exec_ms_module_motion_state_publisher",
            "global/t_exec_ms_module_commander1",
            "global/t_exec_ms_module_parallel_mech_input",
            "global/t_exec_ms_module_parallel_mech_output",
            "global/t_exec_ms_module_joint_map_output",
            "global/t_exec_ms_module_joint_map_input",
            "global/t_exec_ms_module_simulator_out",
            "global/t_exec_ms_module_simulator_in",
            "global/t_exec_ms_module_common_legged_ekf",
            "global/t_exec_ms_module_planner_pvt_switch",
            "global/t_exec_ms_module_command_manager",
            "global/t_exec_ms_module_rl_locomotion",
            "global/t_exec_ms_module_rl_locomotion_run",
            "global/t_exec_ms_record",
            "global/t_exec_ms_draw",

            -- simulator_in
            "simulator_in/joint_cmd_pos",
            "simulator_in/joint_cmd_vel",
            "simulator_in/joint_cmd_tor",
            "simulator_in/joint_cmd_kp",
            "simulator_in/joint_cmd_kd",
            "simulator_in/torque_act",      -- only on webots
            "simulator_in/t",
            
            -- simulator_out
            "simulator_out/joint_fb_pos",
            "simulator_out/joint_fb_vel",
            "simulator_out/joint_fb_tor",
            "simulator_out/imu_eul",
            "simulator_out/imu_quaternion",
            "simulator_out/imu_acc",
            "simulator_out/imu_rotvel",
            "simulator_out/torso_gps",
            "simulator_out/motor_temp",
            "simulator_out/motor_err_code",
            "simulator_out/t",
            
            -- -- rl_locomotion
            -- "rl_locomotion/obs",
            -- "rl_locomotion/t",
            -- "rl_locomotion/denoised",

            -- rl_locomotion_run
            "rl_locomotion_run/obs",
            "rl_locomotion_run/t",
            "rl_locomotion_run/denoised",

            "rl_locomotion_old/obs",
            "rl_locomotion_old/t",
            "rl_locomotion_old/denoised",

            -- amp_locomotion_run
            "amp_locomotion_run/obs",
            "amp_locomotion_run/t",
            "amp_locomotion_run/denoised",

            -- command_manager
            "command_manager/last_rl_type",
            "command_manager/fdr_state",
            "command_manager/is_doing_stand_up",
            "command_manager/planner_index",
            "command_manager/t",

            -- rsm
            "robot_state_manager/planner_index",
            "robot_state_manager/t",            

            -- -- fall_down_recovery
            -- "fall_down_recovery/torso_ang_vel_norm",
            -- "fall_down_recovery/torso_rpy",
            -- "fall_down_recovery/falling_timer_",
            -- "fall_down_recovery/is_ready_timer_",
            -- "fall_down_recovery/state_record_",
            -- "fall_down_recovery/state_",
            -- "fall_down_recovery/t",
            
            -- parallel_mech_input_custom_traj
            -- "parallel_mech_input_custom_traj/motor_pvt_serial_p_des",
            -- "parallel_mech_input_custom_traj/motor_pvt_serial_v_des",
            -- "parallel_mech_input_custom_traj/t",
            -- parallel_mech_output
            -- "parallel_mech_output/joint_serial_pos_fb",
            -- "parallel_mech_output/joint_serial_vel_fb",
            -- "parallel_mech_output/t",           
            -- "parallel_mech_output/t",
            -- "parallel_mech_output/joint_serial_pos_fb",
            -- "parallel_mech_output/joint_serial_vel_fb",
            -- "parallel_mech_output/joint_parallel_pos_fb",
            -- "parallel_mech_output/joint_parallel_vel_fb",


            -- "parallel_mech_input_custom_mode/t",
            -- -- "parallel_mech_input_custom_mode/motor_pvt_parallel_torq_ff",
            -- "parallel_mech_input_custom_mode/motor_pvt_parallel_p_des",
            -- "parallel_mech_input_custom_mode/motor_pvt_parallel_v_des",

            -- odom_estimator
            -- "odom_estimator/t",
            -- "odom_estimator/pos_est",
            -- "odom_estimator/rpy_est",
            -- "odom_estimator/output_anti",
        },
    },
}


function options.init(arg1)
    is_real_bot = arg1

    if  is_real_bot then
        urdf_path = "/opt/booster/Gait/configs/K1/robot.urdf"
        common_wbc_path = "/opt/booster/Gait/configs/K1/config.toml"
        stance_wbc_path = "/opt/booster/Gait/configs/K1/config.toml"
        command_config_path = "/opt/booster/Gait/configs/K1/command_config.toml"
        jostick_command_config_path = "/opt/booster/Gait/configs/K1/task_instruction.yaml"
        
        options.dcm_pvt.urdf_path = urdf_path
        options.stance_pvt.urdf_path = urdf_path
        options.dcm_planner_wbc_convert.urdf_path = urdf_path
        options.stance_planner_wbc_convert.urdf_path = urdf_path
        options.hand_direct_command_planner.urdf_path = urdf_path
        options.command_manager.urdf_path = urdf_path
        options.command_manager.command_config_path = command_config_path
        options.robot_state_manager.urdf_path = urdf_path
        options.robot_state_manager.command_config_path = jostick_command_config_path
        options.robot_state_manager.enable_remote_controller = false
        options.robot_state_manager.disable_part_commands = true
        options.rl_locomotion_run.urdf_file = urdf_path
        options.rl_locomotion_old.urdf_file = urdf_path
        options.amp_locomotion_run.urdf_file = urdf_path
        options.visual_kick.urdf_file = urdf_path
        options.action_hub.urdf_path = urdf_path

        options.common_legged_ekf.config_path = common_wbc_path
        options.legged_estimate_convert.config_path = common_wbc_path
        options.dcm_wbc_common.wbc_config_path = common_wbc_path
        options.stance_wbc_common.wbc_config_path = stance_wbc_path
        -- options.command_manager.enable_remote_controller = true
        options.command_manager.default_planner_index = 3
    else
        urdf_path_sim = "../../booster_config/robot_config/kidsize/K1/robot.urdf"
        common_wbc_path_sim = "../../booster_config/robot_config/kidsize/K1/configurations/config.toml"
        stance_wbc_path_sim = "../../booster_config/robot_config/kidsize/K1/configurations/config.toml"
        command_config_path_sim = "../../booster_config/robot_config/kidsize/K1/configurations/command_config.toml"
        
        -- options.dcm_planner.urdf_path = urdf_path_sim
        -- options.dcm_planner.urdf_path_ = urdf_path_sim
        options.stance_pvt.urdf_path = urdf_path_sim
        options.dcm_pvt.urdf_path = urdf_path_sim
        options.dcm_planner_wbc_convert.urdf_path = urdf_path_sim
        options.stance_planner_wbc_convert.urdf_path = urdf_path_sim
        options.hand_direct_command_planner.urdf_path = urdf_path_sim
        options.command_manager.urdf_path = urdf_path_sim
        options.command_manager.command_config_path = command_config_path_sim
        options.robot_state_manager.urdf_path = urdf_path_sim
        options.rl_locomotion_run.urdf_file = urdf_path_sim
        options.rl_locomotion_old.urdf_file = urdf_path_sim
        options.amp_locomotion_run.urdf_file = urdf_path_sim
        options.visual_kick.urdf_file = urdf_path_sim
        options.action_hub.urdf_path = urdf_path_sim

        options.common_legged_ekf.config_path = common_wbc_path_sim
        options.legged_estimate_convert.config_path = common_wbc_path_sim
        options.dcm_wbc_common.wbc_config_path = common_wbc_path_sim
        options.stance_wbc_common.wbc_config_path = stance_wbc_path_sim
        options.drawer.drawer_backend = "WebotsDrawerBackend"
        -- options.command_manager.enable_remote_controller = true
        options.command_manager.default_planner_index = 1

        options.action_hub.saved_data_path_list_ = {"../../booster_config/robot_config/kidsize/K1/configurations/dance_new_year_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/dance_boxing_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/dance_nezha_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/dance_towards_future_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/gesture_maneki_neko_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/gesture_pogba_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/gesture_ultraman_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/gesture_chinese_greet_k1.txt",
                                                    "../../booster_config/robot_config/kidsize/K1/configurations/gesture_greet_k1.txt"
                                                    }
        -- options.dcm_planner.demo_ = "demo2"
        options.dcm_planner.demo_ = ""

        options.amp_locomotion_run.kd = {
            2., 2.,
            2.5, 2.5, 2.5, 2.5,
            2.5, 2.5, 2.5, 2.5,
            2., 2., 2., 2., 0.1, 0.1, --sim
            2., 2., 2., 2., 0.1, 0.1,
        }

        options.rl_locomotion_old.ankle_pitch_kp = 40.
        options.rl_locomotion_old.ankle_pitch_kd = 0.1
        options.rl_locomotion_old.ankle_roll_kp = 40.
        options.rl_locomotion_old.ankle_roll_kd = 0.1

        options.traj_track_bmm.trajectories.traj_1.kd = {
            2., 2.,
            0.3, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }  
        options.traj_track_bmm.trajectories.traj_2.kd = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }  
        options.traj_track_bmm.trajectories.traj_3.kd = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }  
        options.traj_track_bmm.trajectories.traj_4.kd = {
            2., 2.,
            0.3, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }  
        options.traj_track_bmm.trajectories.traj_5.kd = {
            1., 1.,
            1., 1., 1., 1.,
            1., 1., 1., 1.,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }  
        options.traj_track_bmm.trajectories.traj_6.kd = {
            2., 2.,
            0.3, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }  
        options.traj_track_bmm.trajectories.traj_7.kd = {
            2.0, 2.0, 
            0.5, 0.5, 0.5, 0.5, 
            0.5, 0.5, 0.5, 0.5, 
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }          
        options.traj_track_bmm.trajectories.traj_8.kd = {
            2., 2.,
            0.3, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3,
            2., 2., 2., 2., 0.5, 0.5,
            2., 2., 2., 2., 0.5, 0.5,
        }                                                  

        options.simulator_in = {
            -- desc: 表示是否记录数据
            -- scope: 
            -- unit: 
            record_data = true,

            -- desc: 与仿真软件交互时，控制算法的控制周期。应为正整数
            -- scope: 
            -- unit: ms
            step_len_ = controller_base_dt_ms,

            -- desc: 表示Webots中各个关节执行器（电机）的名字。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: 
            webots_motor_names_ = {              
                "AAHead_Yaw", "Head_Pitch",
                "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
                "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",

                "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw",
                "Left_Knee_Pitch", "Left_Crank_Up", "Left_Crank_Down", 
                
                "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw",
                "Right_Knee_Pitch", "Right_Crank_Up", "Right_Crank_Down", 
            },

            -- desc: 表示各个关节执行器（电机）的正方向。1表示正方向不变，-1表示正方向反向。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: 
            motor_direction_ = {
                1, 1, 
                1, 1, 1, 1, 
                1, 1, 1, 1, 
                1, 1, 1, 1, 1, 1, 
                1, 1, 1, 1, 1, 1,
            },        

            -- desc: 表示发送给各个关节执行器（电机）的力矩指令的最大值。应为正数。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: Nm
            motor_max_torque_value_ = {
                8.0, 8.0,
                18.0, 18.0, 18.0, 18.0, 
                18.0, 18.0, 18.0, 18.0, 
                60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
                60.0, 50.0, 36.0, 60.0, 36.0, 36.0,
            },         

            -- desc: 表示各个关节执行器（电机）的位置指令的最大值。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: [0, 3.14159]
            -- unit: Nm
            motor_max_position_value_ = {
                3.14159, 3.14159, 
                3.14159, 3.14159, 3.14159, 3.14159,
                3.14159, 3.14159, 3.14159, 3.14159,
                3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 
                3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 
            },

            -- desc: 表示是否添加关节执行器（电机）力矩变化率限制
            -- scope: 
            -- unit: 
            add_motor_torque_dot_limit_ = false,

            -- desc: 表示关节执行器（电机）力矩变化率上限。应为正数。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: Nm*ms^(-1)
            motor_max_torque_dot_value_ = {
                20.0, 20.0,
                20.0, 20.0, 20.0, 20.0, 
                20.0, 20.0, 20.0, 20.0, 
                20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
                20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
            },
            
            -- desc：表示是否要在机器人上自动施加推力
            -- scope: 
            -- unit:
            enable_push_test_ = false,

            -- desc：表示箭头模型的.wbo文件的路径。可以是绝对路径，也可以是相对于supervisor controller的路径
            -- scope: 
            -- unit:
            arrow_file_path_ = "../robotics_control/src/tests/biped_test/worlds/Arrow.wbo",

            -- desc：表示施加的推力是否是随机的。若是，则推力大小范围不超过max_push_force_；若不是，则推力设为max_push_force_
            -- scope: 
            -- unit:
            apply_random_push_force_ = false,
            
            -- desc：表示施加推力大小的随机种子
            -- scope: 
            -- unit:
            push_force_random_seed_ = 1.0,

            -- desc: 表示施加推力的各个分量的最大绝对值，应为正数
            -- scope: 
            -- unit: N
            max_push_force_ = { -56.0, 37.0, 0.0 }, -- adjust footprint

            -- desc: 表示施加推力的作用位置范围，应为正数
            -- scope: 
            -- unit: m
            range_force_pos_ = { 0.0, 0.0, 0.0 },

            -- desc: 表示施加推力的作用位置偏置
            -- scope: 
            -- unit: m
            force_pos_offset_ = { 0.0, 0.0, 0.1 },

            -- desc: 表示绘制推力可视化模型时，力的模型长度与力的大小的比值
            -- scope: 
            -- unit: m/N
            force_length_scale_ = 0.01,

            -- desc: 表示施加推力的持续时间，应为正数
            -- scope: 
            -- unit: s
            -- force_action_time_ = 0.1,
            force_action_time_ = 0.2, -- for default
            -- force_action_time_ = 4.0,

            -- desc: 表示施加推力的时间间隔，应为正数，且大于force_action_time_
            -- scope: 
            -- unit: s
            force_time_interval_ = 5.0,
            -- force_time_interval_ = 4.0,
            -- force_time_interval_ = 3.0,
        }
        options.simulator_out = {
            -- desc: 表示是否记录数据
            -- scope: 
            -- unit: 
            record_data = true,

            -- desc: 表示Webots中各个关节的位置传感器的名字。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: 
            webots_motor_pos_sensor_names_ = {    
                "AAHead_Yaw_sensor", "Head_Pitch_sensor",
                "ALeft_Shoulder_Pitch_sensor", "Left_Shoulder_Roll_sensor", "Left_Elbow_Pitch_sensor", "Left_Elbow_Yaw_sensor",
                "ARight_Shoulder_Pitch_sensor", "Right_Shoulder_Roll_sensor", "Right_Elbow_Pitch_sensor", "Right_Elbow_Yaw_sensor",

                "Left_Hip_Pitch_sensor", "Left_Hip_Roll_sensor", "Left_Hip_Yaw_sensor",
                "Left_Knee_Pitch_sensor", "Left_Crank_Up_sensor", "Left_Crank_Down_sensor", 
                
                "Right_Hip_Pitch_sensor", "Right_Hip_Roll_sensor", "Right_Hip_Yaw_sensor",
                "Right_Knee_Pitch_sensor", "Right_Crank_Up_sensor", "Right_Crank_Down_sensor", 
            },

            -- desc: 表示各个关节的位置传感器的正方向。1表示正方向不变，-1表示正方向反向。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: 
            motor_direction_ = {
                1, 1,
                1, 1, 1, 1,
                1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 
                1, 1, 1, 1, 1, 1,
            },

            -- desc: 表示Webots中各个IMU传感器的名字。顺序依次为：躯干、左脚板、右脚板
            -- scope: 
            -- unit: 
            webots_imu_sensor_names_ = {
                "torso inertial unit", 
            },

            -- desc: 表示Webots中各个陀螺仪传感器的名字。顺序依次为：躯干、左脚板、右脚板
            -- scope: 
            -- unit: 
            webots_gyro_sensor_names_ = {
                "torso gyro", 
            },

            -- desc: 表示Webots中各个加速度传感器的名字。顺序依次为：躯干、左脚板、右脚板
            -- scope: 
            -- unit: 
            webots_acc_sensor_names_ = {
                "torso accelerometer",
            },

            -- desc: 表示Webots中各个GPS传感器的名字。顺序依次为：躯干、左脚板、右脚板
            -- scope: 
            -- unit: 
            webots_gps_sensor_names_ = {
                "torso gps", 
            },

            -- desc: 表示Webots中各个力传感器的名字。顺序依次为：左脚板、右脚板
            -- scope: 
            -- unit: 
            webots_force_sensor_names_ = {
                -- "left touch sensor", "right touch sensor",
            },

            -- desc: 表示Webots中各个力矩传感器的名字。置空表示没有传感器
            -- scope: 
            -- unit: 
            webots_torque_sensor_names_ = {
                -- "", "", "", "", "", "",
            },
        }

        options.rl_locomotion_run.ankle_pitch_kd = 0.2
        options.rl_locomotion_run.ankle_roll_kd = 0.2

        options.amp_locomotion_run.kd[21] = 0.1
        options.amp_locomotion_run.kd[22] = 0.1

        options.visual_kick.kd[15] = 0.1
        options.visual_kick.kd[16] = 0.1
        options.visual_kick.kd[21] = 0.1
        options.visual_kick.kd[22] = 0.1
    end
end

-- Lion Dance Prepare Body Control Module Configuration
options.lion_dance_commander1 = {
    record_data = true,
    n_joint = 22, 
    kp = kp_test,
    kd = kd_test,
    p = {
        0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
    },
    v = {
        0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
    },
    torq = {
        0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
    },
    wave_type = "slope",
    peak = ready_pos_test,  -- Will be overridden by hardcoded value in module
    period = 2.0,
    start_from_cur_state = true,
}

-- Lion Dance ready position (arms raised for lion dance)
ready_pos_test_lion = {
    -0.00984106, -0.05354192,  -- neck (2)
    -0.81616835, -1.37621103, 0.40890744, -2.21507698,  -- left arm (4)
    -0.81616835, 1.37621103, 0.40890744, 2.21507698,  -- right arm (4)
    -0.0, 0.0, 0.0, 0.105, 0.08, 0.03,  -- left leg (6)
    -0.0, 0.0, 0.0, 0.105, 0.08, 0.03,  -- right leg (6)
}

return options
