package.path = package.path .. ";../src/tests/?.lua"

controller_base_dt_ms = 2

kp_test = {
    -- on ground
    -- 5., 5.,
    -- 40., 50., 20., 10.,
    -- 40., 50., 20., 10.,
    -- 100., 
    -- 350., 350., 180., 350., 550., 550.,
    -- 350., 350., 180., 350., 550., 550.,

    -- on air
    5., 5.,
    40., 40., 20., 10.,
    40., 40., 20., 10.,
    50., 
    250., 250., 180., 250., 100., 100., 0.0, 0.0, 
    250., 250., 180., 250., 100., 100.,

    -- 0., 0.,
    -- 0., 0., 0., 0.,
    -- 0., 0., 0., 0.,
    -- 0., 
    -- 0., 0., 0., 0., 0., 0.,
    -- 0., 0., 0., 0., 0., 0.,
}

kd_test = {
    -- on ground
    -- .1, .1,
    -- .5, 1.5, .2, .2,
    -- .5, 1.5, .2, .2,
    -- 5.0,
    -- 7.5, 7.5, 3., 5.5, 7.5, 7.5,
    -- 7.5, 7.5, 3., 5.5, 7.5, 7.5,

    -- on air
    .1, .1,
    .5, 2.5, .2, .2,
    .5, 2.5, .2, .2,
    3.,
    5.5, 3., 3., 5.5, .2, .2,  0.0, 0.0, 
    5.5, 3., 3., 5.5, .2, .2,

    -- 0., 0.,
    -- 0., 0., 0., 0.,
    -- 0., 0., 0., 0.,
    -- 0., 
    -- 0., 0., 0., 0., 0., 0.,
    -- 0., 0., 0., 0., 0., 0.,
}

ready_pos_test = {
    0.00,  0.00,
    0.25, -1.40,  0.00, -0.50,
    0.25,  1.40,  0.00,  0.50,
    0.0,
    -- -0.720, -0.0,  0.0,  1.262,  0.436,  0.390,
    -- -0.720,  0.0, -0.0,  1.262,  0.436,  0.390,

    -- -0.3, 0.0, 0.0, 0.6, 0.25, 0.20,
    -- -0.3, 0.0, 0.0, 0.6, 0.25, 0.20,

    0.0, 0.0,  0.0, 0.0,  -0.3, -0.0,  0.0, 0.0, 
    0.0, 0.0,  0.0, 0.0,  -0.3, -0.0, 
}

local options = {
    parallel_mech_input_commander1 = {
        record_data = true,
        use_pvt_ = true,
        joint_idx_parallel_ = {15, 16, 23, 24},
        joint_idx_serial_ = {15, 16, 23, 24},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0600,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1800,                                               -- 长连杆长度
        len_link_R_ = 0.1200,                                               -- 短连杆长度
        r_joint_ = 0.0430,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0140, 0.0000, -0.1840},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, -0.0120},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0415, 0.0000, 0.0327},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0550,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -92.37855,                                  -- 初始位置时，上下两曲柄与“膝关节轴线与脚踝Pitch轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 11.0135,                                   -- 初始位置时，上下两曲柄与“膝关节轴线与脚踝Pitch轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.51620, 1.70842, 1.51620, 1.70842,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“膝关节轴线与脚踝Pitch轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.00000, 0.00000, 0.00000, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角
        is_mirror_ = {false, true},
    },
    parallel_mech_output = {
        record_data = true,
        use_fk_hotstart_ = true,
        joint_idx_parallel_ = {15, 16, 23, 24},
        joint_idx_serial_ = {15, 16, 23, 24},
        ik_clockwise_ = false,
        fk_same_dir_ = false,
        dist_joint_LR_ = 0.0600,                                            -- 两个踝关节电机的距离
        len_link_L_ = 0.1800,                                               -- 长连杆长度
        len_link_R_ = 0.1200,                                               -- 短连杆长度
        r_joint_ = 0.0430,                                                  -- 曲柄摇臂长度
        pos_ankle_in_limb_ = {0.0140, 0.0000, -0.1840},                     -- 脚踝Pitch坐标原点与上脚踝电机位置偏置
        pos_ankle_shift_ = {0.0000, 0.0000, -0.0120},                       -- 脚踝Roll与Pitch偏置
        pos_heel_in_foot_ = {-0.0415, 0.0000, 0.0327},                      -- 脚后跟两个连杆坐标连线中点与脚踝Roll坐标的偏置
        width_heel_ = 0.0550,                                               -- 脚后跟左右连杆距离
        mech_pitch_offset_deg_ = -92.37855,                                  -- 初始位置时，上下两曲柄与“膝关节轴线与脚踝Pitch轴线的连接面”的上夹角和的平均值，单位：deg
        mech_roll_offset_deg_ = 11.0135,                                   -- 初始位置时，上下两曲柄与“膝关节轴线与脚踝Pitch轴线的连接面”的上夹角之差，单位：deg
        joint_parallel_zero_pos_ = {1.51620, 1.70842, 1.51620, 1.70842,},   -- 四个脚踝曲柄（左腿上、左腿下、右腿上、右腿下）与“膝关节轴线与脚踝Pitch轴线的连接面”的弧度
        joint_serial_zero_pos_ = {0.00000, 0.00000, 0.00000, 0.00000,},     -- 两个脚踝关节连线的垂线与零位脚板的夹角
        is_mirror_ = {false, true},
    },

    commander1 = {
        record_data = true,
        n_joint = 25, 
        kp = kp_test,
        kd = kd_test,
        p = {
            0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            0., 
            0., 0., 0., 0., 0., 0., 0.0, 0.0,
            0., 0., 0., 0., 0., 0.,
        },
        v = {
            0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            0., 
            0., 0., 0., 0., 0., 0., 0.0, 0.0,
            0., 0., 0., 0., 0., 0.,
        },
        torq = {
            0., 0.,
            0., 0., 0., 0.,
            0., 0., 0., 0.,
            0., 
            0., 0., 0., 0., 0., 0., 0.0, 0.0,
            0., 0., 0., 0., 0., 0.,
        },
        wave_type = "slope",
        peak = ready_pos_test,
        period = 2.0,
        start_from_cur_state = true,
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
        real_robot = false,
        server_url = "192.168.1.199:8080",
        robot_name = "test_joints",
        sampling_rate = 0.5,
        chart_keys = {
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
            "simulator_out/imu_acc",
            "simulator_out/imu_rotvel",
            "simulator_out/motor_temp",
            "simulator_out/motor_err_code",
            "simulator_out/t",
        
            -- parallel_mech_output
            "parallel_mech_output/t",
            "parallel_mech_output/joint_parallel_pos_fb",
            "parallel_mech_output/joint_parallel_vel_fb",
            "parallel_mech_output/joint_serial_pos_fb",
            "parallel_mech_output/joint_serial_vel_fb",

            -- parallel_mech_input
            "parallel_mech_input/t",
            "parallel_mech_input/motor_pvt_parallel_p_des",
        },
    },
}

function options.init(is_real_bot)
    if  not (is_real_bot) then
        options.drawer.drawer_backend = "WebotsDrawerBackend"
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
                "AAHead_yaw", "Head_pitch",
                "Left_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
                "Right_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
                "Waist",

                "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw",
                "Left_Knee_Pitch", "Crank_Up_Left", "Crank_Down_Left", "Left_Ankle_Pitch", "Left_Ankle_Roll", 
                
                "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw",
                "Right_Knee_Pitch", "Crank_Up_Right", "Crank_Down_Right",
            },

            -- desc: 表示各个关节执行器（电机）的正方向。1表示正方向不变，-1表示正方向反向。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: 
            motor_direction_ = {
                1, 1, 
                1, 1, 1, 1, 
                1, 1, 1, 1, 
                1,
                1, 1, 1, 1, 1, 1, 1, 1, 
                1, 1, 1, 1, 1, 1,
            },        

            -- desc: 表示发送给各个关节执行器（电机）的力矩指令的最大值。应为正数。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: Nm
            motor_max_torque_value_ = {
                7.0, 7.0,
                36.0, 36.0, 36.0, 36.0,
                36.0, 36.0, 36.0, 36.0,
                60.0,
                90.0, 60.0, 60.0, 130.0, 36.0, 50.0, 0.0, 0.0,
                90.0, 60.0, 60.0, 130.0, 36.0, 50.0,
            },

            -- desc: 表示各个关节执行器（电机）的位置指令的最大值。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: [0, 3.14159]
            -- unit: Nm
            motor_max_position_value_ = {
                3.14159, 3.14159, 
                3.14159, 3.14159, 3.14159, 3.14159, 
                3.14159, 3.14159, 3.14159, 3.14159, 
                3.14159,
                3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 
                3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 
            },

            -- desc: 表示是否添加关节执行器（电机）力矩变化率限制
            -- scope: 
            -- unit: 
            add_motor_torque_dot_limit_ = true,

            -- desc: 表示关节执行器（电机）力矩变化率上限。应为正数。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: Nm*ms^(-1)
            motor_max_torque_dot_value_ = {
                20.0, 20.0,
                20.0, 20.0, 20.0, 20.0,
                20.0, 20.0, 20.0, 20.0,
                20.0,
                20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 0.0, 0.0,
                20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
            },
            
            -- desc：表示是否要在机器人上自动施加推力
            -- scope: 
            -- unit:
            enable_push_test_ = false,
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
                "AAHead_yaw_sensor", "Head_pitch_sensor",
                "Left_Shoulder_Pitch_sensor", "Left_Shoulder_Roll_sensor", "Left_Elbow_Pitch_sensor", "Left_Elbow_Yaw_sensor",
                "Right_Shoulder_Pitch_sensor", "Right_Shoulder_Roll_sensor", "Right_Elbow_Pitch_sensor", "Right_Elbow_Yaw_sensor",
                "Waist_sensor",

                "Left_Hip_Pitch_sensor", "Left_Hip_Roll_sensor", "Left_Hip_Yaw_sensor",
                "Left_Knee_Pitch_sensor", "Crank_Up_Left_sensor", "Crank_Down_Left_sensor", "Left_Ankle_Pitch_sensor", "Left_Ankle_Roll_sensor", 
                
                "Right_Hip_Pitch_sensor", "Right_Hip_Roll_sensor", "Right_Hip_Yaw_sensor",
                "Right_Knee_Pitch_sensor", "Crank_Up_Right_sensor", "Crank_Down_Right_sensor",
            },

            -- desc: 表示各个关节的位置传感器的正方向。1表示正方向不变，-1表示正方向反向。各个分量对应各个关节。关节的顺序依次为：胸腔旋转、左肩的前摆、侧摆和肘关节、右肩的前摆、侧摆和肘关节、左腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角、右腿的前摆、侧摆、大腿旋转、膝盖、脚踝仰俯角、脚踝横滚角。
            -- scope: 
            -- unit: 
            motor_direction_ = {
                1, 1,
                1, 1, 1, 1,
                1, 1, 1, 1,
                1,
                1, 1, 1, 1, 1, 1, 1, 1, 
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
    end
end

return options
