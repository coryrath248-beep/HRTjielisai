#pragma once

#include <tuple>
#include <deque>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();

    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value;
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    /**
     * 闂備礁鎲＄敮妤冩崲閸岀儑缍栭柟鐗堟緲缁€?blackboard 闂傚倷鐒﹁ぐ鍐儔閻撳簶鏋?entry闂備焦瀵х粙鎴﹀嫉椤掑嫬鏋侀柣鎰惈缁犳盯鐓崶銊︹拹闁绘挴鍋撻梻浣告啞濮婄粯鎱ㄩ幘顔藉仱鐟滃秹鍩㈤幘璺烘瀳濠㈣泛鐬奸ˇ顕€姊洪棃鈺勭闁告柨绻掗幑銏ゅ冀椤撶喓鍘掗悗鍏夊亾闁告劖鍎抽弲顓犵磽閸屾艾鏋ら柛鐘虫礋閺屽苯顭ㄩ崟鍨粯濠碘槅鍨遍娆撳触閸ヮ剚鐓?     */
    void initEntry();
};


class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("cross_threshold", 0.2, "cross threshold"),
            InputPort<double>("shoot_goal_post_margin", 0.2, "shoot goal post margin")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "chase threshold"),
            InputPort<string>("decision_in", "", "previous decision"),
            InputPort<string>("position", "offense", "kick position mode"),
            InputPort<double>("kick_dir_tolerance", 0.20, "kick direction tolerance"),
            InputPort<int>("kick_stable_msecs", 0, "unused"),
            InputPort<double>("kick_max_ball_yaw", 0.5, "max ball yaw before kick"),
            InputPort<double>("kick_point_dist", 0.6, "kick point distance behind ball"),
            InputPort<double>("position_tolerance", 0.18, "kick point position tolerance"),
            InputPort<double>("orient_tolerance", 0.12, "kick direction body tolerance"),
            InputPort<double>("kick_ball_x_tolerance", 0.14, "ball forward tolerance before kick"),
            InputPort<double>("kick_ball_y_tolerance", 0.10, "ball lateral tolerance before kick"),
            InputPort<int>("force_kick_after_msecs", 1500, "force kick after good enough setup msecs"),
            InputPort<double>("force_kick_tolerance_scale", 1.8, "force kick tolerance scale"),
            InputPort<double>("force_kick_max_orient_error", 0.30, "force kick max body orientation error"),
            InputPort<double>("force_kick_max_ball_yaw", 0.32, "force kick max ball yaw"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0;
    rclcpp::Time timeLastTick;
    rclcpp::Time timeGoodEnoughKick;
    bool goodEnoughKickTimerActive = false;
};


class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 4.0, "chase threshold"),
            InputPort<double>("setup_range", 1.0, "range to switch from chase to kick setup"),
            InputPort<double>("kick_point_dist", 0.6, "kick point distance behind ball"),
            InputPort<double>("position_tolerance", 0.18, "kick point position tolerance"),
            InputPort<double>("orient_tolerance", 0.15, "kick direction body tolerance"),
            InputPort<int>("ball_velocity_sample_count", 10, "number of ball velocity samples to average"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "adjust angle tolerance"),
            InputPort<double>("adjust_y_tolerance", 0.1, "adjust y tolerance"),
            InputPort<string>("decision_in", "", "previous decision"),
            InputPort<double>("auto_visual_kick_enable_dist_min", 2.0, "auto visual kick min distance"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 3.0, "auto visual kick max distance"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "auto visual kick angle"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 3.0, "auto visual kick obstacle distance"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "auto visual kick obstacle angle"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
    bool hasLastBallForVelocity = false;
    Point lastBallPosForVelocity;
    rclcpp::Time lastBallVelocityTime;
    double ballVxToField = 0.0;
    double ballVyToField = 0.0;
    deque<double> ballVxSamplesForVelocity;
    deque<double> ballVySamplesForVelocity;
};


class GoalieBlock : public SyncActionNode
{
public:
    GoalieBlock(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.20, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, within which considered arrived."),
            InputPort<double>("vx_limit", 0.4, "x speed limit"),
            InputPort<double>("vy_limit", 0.8, "y speed limit"),
            InputPort<double>("vtheta_limit", 1.5, "turn speed limit"),
            InputPort<double>("vtheta_gain", 3.0, "turn speed gain"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override;

private:
    Brain *brain;
};


class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    
    rclcpp::Time _timeLastCmd;   
    int _cmdIndex;                
    long _cmdIntervalMSec;        
    long _cmdRestartIntervalMSec; 

    Brain *brain;

};


class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "yaw speed limit"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; 
    Brain *brain;
};

class SmartFindBall : public StatefulActionNode
{
public:
    SmartFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("reacquire_msecs", 1200, "short memory based reacquire time"),
            InputPort<double>("local_scan_msecs", 3500, "local sector scan time"),
            InputPort<double>("global_scan_msecs", 6500, "global head scan time before body search"),
            InputPort<double>("head_interval_msecs", 220, "head scan interval"),
            InputPort<double>("body_turn_speed", 0.9, "body turn speed while searching"),
            InputPort<double>("body_turn_msecs", 650, "body turn pulse duration"),
            InputPort<double>("body_pause_msecs", 260, "pause duration between body turn pulses"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    bool getBallSearchGuess(double &pitch, double &yaw);
    void commandGuessHead(double yawOffset);
    void commandLocalScan();
    void commandGlobalScan();

    rclcpp::Time _startTime;
    rclcpp::Time _timeLastHeadCmd;
    int _scanIndex = 0;
    double _turnDir = 1.0;
    Brain *brain;
};


class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 300, "scan interval msecs"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _cmdSequence[7][2] = {
        {0.45, 1.1},
        {0.45, 0.0},
        {0.45, -1.1},
        {1.0, -1.1},
        {1.0, 0.0},
        {1.0, 1.1},
        {0.45, 0.0},
    };    
    rclcpp::Time _timeLastCmd;    
    int _cmdIndex = 0;               
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "turn radians"),
            InputPort<bool>("towards_ball", false, "濠?true 闂? 濠电偞鍨堕幐鍝ョ矓閻戣棄鐒垫い鎺嗗亾闁哥喎鍟块‖?rad 闂備焦鐪归崝宀€鈧凹鍙冮、妤€顭ㄩ崨顔碱伕闂佺粯鏌ㄩ崲鍙夌箾? 闂備礁鍚嬪姗€寮甸鈧嵄闁告稑锕ユ慨婊勩亜閹哄棗浜鹃梺璇″枟缁秶绮欐径瀣瘈濞达絼璀︽导鍥р攽閻愬弶锛旂紒杈ㄦ礀閿曘垽宕ㄩ弶鎴狀槴婵☆偊顣﹂懗鍫曟偡濠靛鐓熼柣鎰絻閺嗙喐绻涢崼鐔风伌鐎殿噮鍓熼幃鐑芥偋閸績鍋?")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _lastAngle; 
    double _angle;
    double _cumAngle; 
    double _msecLimit = 5000;  
    rclcpp::Time _timeStart;
    Brain *brain;
};


class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.6, "chase x speed limit"),
            InputPort<double>("vy_limit", 0.4, "chase y speed limit"),
            InputPort<double>("vtheta_limit", 1.0, "chase turn speed limit"),
            InputPort<double>("dist", 0.1, "target distance behind ball"),
            InputPort<double>("safe_dist", 4.0, "circle back safe distance"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};


class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 3.25, "adjust turn threshold"),
            InputPort<double>("vx_limit", 0.05, "闂佽崵濮撮鍛村疮椤栫偛姹查柨婵嗘娴溿倝鏌涢妷顔荤敖缂佺姵鐗滅槐鎺斺偓锝庝憾閸庡酣鏌?vx 闂備焦鐪归崝宀€鈧碍婢橀‖濠偯洪鍕槴?[-limit, limit]"),
            InputPort<double>("vy_limit", 0.05, "闂佽崵濮撮鍛村疮椤栫偛姹查柨婵嗘娴溿倝鏌涢妷顔荤敖缂佺姵鐗滅槐鎺斺偓锝庝憾閸庡酣鏌?vy 闂備焦鐪归崝宀€鈧碍婢橀‖濠偯洪鍕槴?[-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.1, "闂佽崵濮撮鍛村疮椤栫偛姹查柨婵嗘娴溿倝鏌涢妷顔荤敖缂佺姵鐗滅槐鎺斺偓锝庝憾閸庡酣鏌?vtheta 闂備焦鐪归崝宀€鈧碍婢橀‖濠偯洪鍕槴?[-limit, limit]"),
            InputPort<double>("range", 2.25, "target ball range"),
            InputPort<double>("vtheta_factor", 3.0, "adjust vtheta factor"),
            InputPort<double>("tangential_speed_far", 0.2, "far tangential speed"),
            InputPort<double>("tangential_speed_near", 0.15, "near tangential speed"),
            InputPort<double>("near_threshold", 0.8, "闂佽崵濮烽崕銈囨崲閸曨垪鈧牠宕堕浣稿壆濡炪倖姊婚弲顐﹀垂婵傚憡鍊堕煫鍥ㄦ尵缁狅絿绱掗…鎴濆婵炶壈顕ч埢搴㈡償閵娿儲鐤傞梻浣侯焾閿曘倝宕ョ€ｎ亶娓? 濠电偠鎻紞鈧繛澶嬫礋瀵?near speed"),
            InputPort<double>("no_turn_threshold", 0.02, "no turn threshold"),
            InputPort<double>("turn_first_threshold", 0.8, "turn first threshold"),

        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class PrepareKickPose : public SyncActionNode
{
public:
    PrepareKickPose(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("kick_point_dist", 0.6, "distance from ball to kick point"),
            InputPort<double>("vx_limit", 0.5, "position x speed limit"),
            InputPort<double>("vy_limit", 0.3, "position y speed limit"),
            InputPort<double>("vtheta_limit", 1.5, "position turn speed limit"),
            InputPort<double>("vtheta_gain", 2.5, "position turn speed gain"),
            InputPort<double>("dist_tolerance", 0.18, "position distance tolerance"),
            InputPort<double>("ball_path_clearance", 0.3, "minimum clearance from ball on direct path"),
            InputPort<double>("ball_detour_lateral_offset", 0.7, "lateral offset when detouring around ball"),
            InputPort<double>("ball_x_tolerance", 0.14, "ball forward tolerance for final setup"),
            InputPort<double>("ball_y_tolerance", 0.10, "ball lateral tolerance for final setup"),
            InputPort<double>("ball_yaw_tolerance", 0.16, "ball yaw tolerance for final setup"),
            InputPort<double>("orient_tolerance", 0.12, "body orientation tolerance for final setup"),
            InputPort<double>("precise_adjust_range", 1.0, "range to use ball-relative final setup"),
            InputPort<double>("precise_start_dist", 0.35, "distance to kick point before final precise setup"),
            InputPort<double>("final_x_gain", 1.2, "final setup x gain"),
            InputPort<double>("final_y_gain", 1.5, "final setup y gain"),
            InputPort<double>("final_orient_gain", 0.35, "final setup body orientation gain"),
            InputPort<bool>("strict_behind_first", false, "stop at kick point before final orientation and adjustment"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    bool _ballDetourActive = false;
    double _ballDetourSide = 1.0;
};


class FaceKickTarget : public SyncActionNode
{
public:
    FaceKickTarget(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("angle_tolerance", 0.12, "kick direction angle tolerance"),
            InputPort<double>("vtheta_limit", 1.5, "turn speed limit"),
            InputPort<double>("vtheta_gain", 3.0, "turn speed gain"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


// 闂備礁婀遍悷鎶藉幢閳哄倹鏉搁梻鍌氼煬娴滄宕规繝姘畺闁糕剝绋戠粈澶愭煏婵炲灝鈧绮?
class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "minimum kick msecs"),
            InputPort<double>("msecs_stablize", 1000, "stabilize msecs"),
            InputPort<double>("speed_limit", 0.8, "kick speed limit"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    
    double _speed; 
    double _minRange; 
    tuple<double, double, double> _calcSpeed();
};


class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("msecs", 1000, "stand still msecs"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
};


class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.6, "闂備礁鎲＄喊宥夊垂瀹曞洨绠旈柛娑樼摠閸庢垿鎮楅敐搴′簼妞は佸洦鐓熼柕濞垮劚椤忣亝绻涢幘鐟扮厫鐎?pitch"),
            InputPort<double>("high_pitch", 0.45, "闂備礁鎲＄喊宥夊垂瀹曞洨绠旈柛宀€鍋為崕鎴︽倵閿濆骸浜濇い蟻鍥ㄧ厽闁靛鍎遍顏呯箾閹惧啿顣冲ù?pitch"),
            InputPort<double>("left_yaw", 0.8, "闂備礁鎲＄喊宥夊垂閸偆鈻曢煫鍥ㄧ⊕閸庢垿鎮楅敐搴′簼妞は佸洦鐓熼柕濞垮劚椤忣亝绻涢幘鐟扮厫鐎?yaw"),
            InputPort<double>("right_yaw", -0.8, "闂備礁鎲＄喊宥夊垂娴ｅ啨浜归柟闂寸劍閸庢垿鎮楅敐搴′簼妞は佸洦鐓熼柕濞垮劚椤忣亝绻涢幘鍐差暢濞?yaw"),
            InputPort<int>("msec_cycle", 4000, "scan cycle msecs"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "target x on field"),
            InputPort<double>("y", 0, "target y on field"),
            InputPort<double>("theta", 0, "target theta on field"),
            InputPort<double>("long_range_threshold", 1.5, "long range threshold"),
            InputPort<double>("turn_threshold", 0.4, "move turn threshold"),
            InputPort<double>("vx_limit", 0.8, "x speed limit"),
            InputPort<double>("vy_limit", 0.5, "y speed limit"),
            InputPort<double>("vtheta_limit", 0.2, "theta speed limit"),
            InputPort<double>("x_tolerance", 0.5, "x tolerance"),
            InputPort<double>("y_tolerance", 0.5, "y tolerance"),
            InputPort<double>("theta_tolerance", 0.5, "theta tolerance"),
            InputPort<bool>("avoid_obstacle", false, "avoid obstacle")
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "x tolerance"),
            InputPort<double>("theta_tolerance", 0.5, "theta tolerance"),
            InputPort<double>("vx_limit", 0.8, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.1, "x speed limit"),
            InputPort<double>("vy_limit", 0.1, "y speed limit"),
            InputPort<double>("dist_to_goalline", 1.8, "distance to goal line"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.1, "x speed limit"),
            InputPort<double>("vy_limit", 0.1, "y speed limit"),
            InputPort<double>("dist_to_goalline", 2.5, "distance to goal line"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


/**
 * @brief 闂佽崵濮崇粈浣规櫠娴犲鍋柛鈩冪☉鐎氬鏌涘┑鍡楊仾闁绘縿鍊曢湁婵犲﹤妫欓鐘崇箾閸喎鐏撮柡灞芥嚇閹垹鐣￠柇锕€鏁?
 *
 * @param x,y,theta double, 闂備礁鎼悧鍡涘箹椤愨懇鏋嶉柟鐑樻尵椤╂煡鏌涘┑鍡楊仾闂?x闂備焦瀵х粙鎴︽儊?闂備礁鎼崐濠氬箠閹捐绠栨繝濠傚閳绘柨鈹戦悩鎻掝仾闁伙綁浜跺娲敃閵堝懏鐎婚柣搴ゎ潐婵炲﹪寮澶婇敜?s闂備焦瀵х粙鎴λ囬鐐茬濞寸厧鐡ㄩ悞濠氭煕閳╁叐鎴λ囪濮婃椽鎸婃径灞解叢濠电偞娼欏Λ婵嗙暦濠靛惟鐟滃酣鎮鹃柆宥嗗仭婵炲棙鍔曞ù顕€鏌嶈閸撴盯宕版惔锝傚亾鐟欏嫬鈻曢柡浣哥Ч瀹曟悂鎮介。?s), 濠殿喗甯楃粙鎺椻€﹂崼銉晣濠电姵鑹剧壕褰掓⒒閸屾粠妫庣紒鈧?0. 闂備胶顭堢换鍫ュ礉鐎ｎ剚瀚?0 闂備礁鎼崯鎶筋敊閹邦喗顫曟繝闈涱儏绾偓閻庤娲栧ú锔剧矈閼碱剚鍠愰柣妤€鐗婄亸顐ょ磼椤垵澧扮紒杈ㄥ笒閳诲氦绠涢幘瀵告缂傚倷鐒﹀褰掓偡瑜旈幃闈涒槈濞嗘垟鏋栭悗骞垮劚閹冲氦顤勯梻浣告贡椤牆煤閿濆绠?
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

// 闂備礁鎲￠…鍥窗鎼粹槄鑰块柟缁㈠枤閸欐帡鐓崶銊︻棓闁?
class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {};
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};


class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("side", "attack", "attack | defense"),
            InputPort<double>("attack_dist", 0.7, "attack side target dist to ball"),
            InputPort<double>("defense_dist", 1.9, "defense side target dist to ball"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),

        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    bool _isInFinalAdjust = false; 
};

class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("valve", 0.5, "border return threshold"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "simple chase stop distance"),
            InputPort<double>("stop_angle", 0.1, "simple chase stop angle"),
            InputPort<double>("vy_limit", 0.2, "闂傚倸鍊哥€氼參宕濋弴銏犳槬?Y 闂備礁鎼崐濠氬箠閹捐绠栨繝濠傜墛閻掑鏌ｉ弬鍨Щ妞? 濠电偛顕慨浼村磿闂堟党锝囨嫚瀹割喗妗ㄩ梺闈涢獜缂嶁偓婵炲鐓￠幃瑙勭瑹椤栨氨浠х紓浣诡殔椤﹁京鍒掑▎鎾崇闁肩⒈鍓氬В? 闂佽崵鍠愬ú鎴澝归崶顒€绠查柨婵嗘婵瓨绻濇繝鍌涘櫤闁哄棗绻樺濠氬炊閿濆懍澹曢梺鑽ゅ枑濞叉垹绮堟笟鈧幆鍐敆娴ｄ警娲搁柟鍏肩暘閸ㄨ櫣鑺卞鑸电厱闁挎繂鍟俊鑺ョ節閳ь剚瀵肩€电褰嗗銈嗗姂閸婃牠鎮鹃柆宥嗏拻闁告洦鍓欐慨宥夋煃?0.4"),
            InputPort<double>("vx_limit", 0.6, "闂傚倸鍊哥€氼參宕濋弴銏犳槬?X 闂備礁鎼崐濠氬箠閹捐绠栨繝濠傜墛閻掑鏌ｉ弬鍨Щ妞? 濠电偛顕慨浼村磿闂堟党锝囨嫚瀹割喗妗ㄩ梺闈涢獜缂嶁偓婵炲鐓￠幃瑙勭瑹椤栨氨浠х紓浣诡殔椤﹁京鍒掑▎鎾崇闁肩⒈鍓氬В? 闂佽崵鍠愬ú鎴澝归崶顒€绠查柨婵嗘婵瓨绻濇繝鍌涘櫤闁哄棗绻樺濠氬炊閿濆懍澹曢梺鑽ゅ枑濞叉垹绮堟笟鈧幆鍐敆娴ｄ警娲搁柟鍏肩暘閸ㄨ櫣鑺卞鑸电厱闁挎繂鍟俊鑺ョ節閳ь剚瀵肩€电褰嗗銈嗗姂閸婃牠鎮鹃柆宥嗏拻闁告洦鍓欐慨宥夋煃?1.2"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "x"),
            InputPort<double>("y", 0, "y"),
            InputPort<double>("theta", 0, "theta"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};

class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("sound", "cheerful", "sound name"),
            InputPort<bool>("allow_repeat", false, "allow repeated sound"),
        };
    }

private:
    Brain *brain;
};

class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("text", "", "text to speak"),
        };
    }

private:
    Brain *brain;
};
