
local graph = {
    import = {
        commander1 = "lib/libmodule_source.so",
        parallel_mech_input = "lib/libparallel_mech.so",
        parallel_mech_output = "lib/libparallel_mech.so",
    },
    modules = {
        commander1 = "source::MitVariedTarget",
        parallel_mech_input_commander1 = "parallel_mech::BipedParallelFeetInput",
        parallel_mech_output = "parallel_mech::BipedParallelFeetOutput",
    },
    connections = {
        -- set connections later
    },
}

function graph.init(is_real_bot)
    if not (is_real_bot) then
        -------------------------- import modules
        graph.import.webots_simulator = "lib/libwebots_interface.so"
        graph.import.pid = "lib/libmodule_pid.so"
	    graph.import.drawer_backend = "lib/libutils_drawings_webots_backend.so"

        -------------------------- set modules
        graph.modules.pid = "pid::MITGroupController"
        graph.modules.simulator_out = "webots_interface::WebotsOutput"
        graph.modules.simulator_in = "webots_interface::WebotsInput"

        -------------------------- set connections
        -------------------------- connect to parallel_mech_output --------------------------
        graph.connections.con1_to_pmo = {
            output_module = "simulator_out",
            output_message_name = "joint_states",
            input_module = "parallel_mech_output",
            input_message_name = "joint_states_parallel",
        }
        graph.connections.con2_to_pmo = {
            output_module = "simulator_out",
            output_message_name = "joint_forces",
            input_module = "parallel_mech_output",
            input_message_name = "joint_forces_parallel",
        }

        -------------------------- connect to commander1 --------------------------
        graph.connections.con_to_commander1 = {
            output_module = "parallel_mech_output",
            output_message_name = "joint_states_serial",
            input_module = "commander1",
            input_message_name = "motor_state_feedback",
        }

        -------------------------- connect to parallel_mech_input_commander1 --------------------------
        graph.connections.con1_to_pmic = {
            output_module = "commander1",
            output_message_name = "mit_motor_command",
            input_module = "parallel_mech_input_commander1",
            input_message_name = "motor_pvt_commands_serial",
        }
        graph.connections.con2_to_pmic = {
            output_module = "simulator_out",
            output_message_name = "joint_states",
            input_module = "parallel_mech_input_commander1",
            input_message_name = "joint_states_parallel",
        }
        graph.connections.con3_to_pmic = {
            output_module = "parallel_mech_output",
            output_message_name = "joint_states_serial",
            input_module = "parallel_mech_input_commander1",
            input_message_name = "joint_states_serial",
        }

        -------------------------- connect to pid --------------------------
        graph.connections.con1_to_pid = {
            output_module = "parallel_mech_input_commander1",
            output_message_name = "motor_pvt_commands_parallel",
            input_module = "pid",
            input_message_name = "motor_commands",
        }
        graph.connections.con2_to_pid = {
            output_module = "simulator_out",
            output_message_name = "joint_states",
            input_module = "pid",
            input_message_name = "motor_states",
        }

        -------------------------- connect to simulator_in --------------------------
        graph.connections.con1_to_si = {
            output_module = "pid",
            output_message_name = "force_commands",
            input_module = "simulator_in",
            input_message_name = "motor_force_commands",
        }
    end
end

return graph
