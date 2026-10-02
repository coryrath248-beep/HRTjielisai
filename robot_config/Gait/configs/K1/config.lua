package.path = package.path .. ";../../booster_config/robot_config/utils/?.lua"
require('config_utils')

is_real_bot = true         

local graph = require('common_graph_define')
-- local graph = require('common_graph_define_test_para_mech')

graph.init(is_real_bot)

local options = require('common_module_options')
-- local options = require('common_module_options_test_para_mech')


options.init(is_real_bot)

values = {
    options = options,
    graph = graph,
}


-- 版本迭代

-- if !is_real_bot or /opt/booster/robot_info.txt not found, default_version working
local default_version = "1.0.0"

if VersionGreaterOrEquals(default_version, "2.3.2", is_real_bot) then
    -- options.custom_traj.traj_4 = stand_up_face_down_traj
    print("versionGreaterOrEquals 2.3.2")
end


return values
