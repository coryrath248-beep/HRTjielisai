#include <cmath>
#include <cstdlib>
#include "brain_tree.h"
#include "locator.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "locator.h"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>

/**
 * 这里使用宏定义来缩减 RegisterBuilder 的代码量
 * REGISTER_BUILDER(Test) 展开后的效果是
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *     [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

namespace
{
Pose2D calcKickSetupPose(const GameObject &ball, double kickDir, double kickPointDist)
{
    Pose2D target;
    target.x = ball.posToField.x - kickPointDist * cos(kickDir);
    target.y = ball.posToField.y - kickPointDist * sin(kickDir);
    target.theta = kickDir;
    return target;
}

bool isBallCenteredInFront(
    const GameObject &ball,
    double targetForwardDist,
    double xTolerance,
    double yTolerance,
    double yawTolerance)
{
    return ball.posToRobot.x > 0.0
        && fabs(ball.posToRobot.x - targetForwardDist) < xTolerance
        && fabs(ball.posToRobot.y) < yTolerance
        && fabs(ball.yawToRobot) < yawTolerance;
}

bool isPointInField(const Point2D &point, const FieldDimensions &fd, double margin)
{
    return point.x >= -fd.length / 2.0 + margin
        && point.x <= fd.length / 2.0 - margin
        && point.y >= -fd.width / 2.0 + margin
        && point.y <= fd.width / 2.0 - margin;
}

Point2D clampPointToField(const Point2D &point, const FieldDimensions &fd, double margin)
{
    return {
        cap(point.x, fd.length / 2.0 - margin, -fd.length / 2.0 + margin),
        cap(point.y, fd.width / 2.0 - margin, -fd.width / 2.0 + margin)
    };
}

Point2D calcBallDetourPoint(const Point &ballPos, double kickDir, double side, double lateralOffset)
{
    return {
        ballPos.x - side * lateralOffset * sin(kickDir),
        ballPos.y + side * lateralOffset * cos(kickDir)
    };
}

bool isBallInSetupPath(
    const Pose2D &robotPose,
    const Pose2D &kickPoint,
    const Point &ballPos,
    double clearance)
{
    Line path = {robotPose.x, robotPose.y, kickPoint.x, kickPoint.y};
    return pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < clearance;
}

double calcSetupLateralOffset(const Pose2D &pose, const Point &ballPos, double kickDir)
{
    const double dx = pose.x - ballPos.x;
    const double dy = pose.y - ballPos.y;
    return -sin(kickDir) * dx + cos(kickDir) * dy;
}

Pose2D calcSetupDetourPose(
    const GameObject &ball,
    double kickDir,
    double side,
    double lateralOffset,
    const FieldDimensions &fd)
{
    const double fieldMargin = 0.2;
    Point2D detourPoint = calcBallDetourPoint(ball.posToField, kickDir, side, lateralOffset);
    if (!isPointInField(detourPoint, fd, fieldMargin)) {
        detourPoint = clampPointToField(detourPoint, fd, fieldMargin);
    }

    return Pose2D({detourPoint.x, detourPoint.y, kickDir});
}

Pose2D calcSetupLanePose(
    const GameObject &ball,
    double kickDir,
    double longitudinalOffset,
    double lateralOffset,
    const FieldDimensions &fd)
{
    const double fieldMargin = 0.2;
    Point2D lanePoint = {
        ball.posToField.x + longitudinalOffset * cos(kickDir) - lateralOffset * sin(kickDir),
        ball.posToField.y + longitudinalOffset * sin(kickDir) + lateralOffset * cos(kickDir)
    };
    lanePoint = clampPointToField(lanePoint, fd, fieldMargin);
    return Pose2D({lanePoint.x, lanePoint.y, kickDir});
}

Pose2D calcSetupTangentPose(
    const Pose2D &robotPose,
    const GameObject &ball,
    double kickDir,
    double side,
    double safetyRadius,
    const FieldDimensions &fd)
{
    const double fieldMargin = 0.2;
    double dx = robotPose.x - ball.posToField.x;
    double dy = robotPose.y - ball.posToField.y;
    double dist = norm(dx, dy);
    if (dist <= safetyRadius + 0.05) {
        return calcSetupDetourPose(ball, kickDir, side, safetyRadius, fd);
    }

    double thetaBallToRobot = atan2(dy, dx);
    double tangentAngle = acos(min(1.0, safetyRadius / dist));
    Point2D tangentPoint = {
        ball.posToField.x + safetyRadius * cos(thetaBallToRobot - side * tangentAngle),
        ball.posToField.y + safetyRadius * sin(thetaBallToRobot - side * tangentAngle)
    };
    tangentPoint = clampPointToField(tangentPoint, fd, fieldMargin);
    return Pose2D({tangentPoint.x, tangentPoint.y, kickDir});
}

double dist2D(const Pose2D &a, const Pose2D &b)
{
    return norm(a.x - b.x, a.y - b.y);
}

double segmentClearancePenalty(
    const Pose2D &from,
    const Pose2D &to,
    const Point &point,
    double safeClearance,
    double weight)
{
    Line path = {from.x, from.y, to.x, to.y};
    double clearance = pointMinDistToLine(Point2D({point.x, point.y}), path);
    if (clearance >= safeClearance) return 0.0;

    double deficit = safeClearance - clearance;
    return weight * deficit * deficit;
}

double fieldBorderPenalty(const Pose2D &pose, const FieldDimensions &fd)
{
    const double softMargin = 0.45;
    double dx = min(pose.x + fd.length / 2.0, fd.length / 2.0 - pose.x);
    double dy = min(pose.y + fd.width / 2.0, fd.width / 2.0 - pose.y);
    double margin = min(dx, dy);
    if (margin >= softMargin) return 0.0;
    return (softMargin - margin) * 2.5;
}

double obstaclePathPenalty(
    const Pose2D &from,
    const Pose2D &mid,
    const Pose2D &to,
    const vector<GameObject> &obstacles,
    double safeClearance)
{
    double penalty = 0.0;
    for (auto &obs : obstacles) {
        if (obs.label == "Ball") continue;
        penalty += segmentClearancePenalty(from, mid, obs.posToField, safeClearance, 8.0);
        penalty += segmentClearancePenalty(mid, to, obs.posToField, safeClearance, 5.0);
    }
    return penalty;
}

struct KickLaneScore
{
    double score = 0.0;
    bool blocked = false;
};

KickLaneScore scoreKickLane(
    const GameObject &ball,
    double kickDir,
    const vector<GameObject> &obstacles,
    const FieldDimensions &fd)
{
    const double nearForward = 0.2;
    const double lookahead = 2.6;
    const double laneWidth = 0.45;
    KickLaneScore result;

    for (auto &obs : obstacles) {
        if (obs.label == "Ball") continue;

        const double dx = obs.posToField.x - ball.posToField.x;
        const double dy = obs.posToField.y - ball.posToField.y;
        const double forward = dx * cos(kickDir) + dy * sin(kickDir);
        if (forward < nearForward || forward > lookahead) continue;

        const double lateral = fabs(-dx * sin(kickDir) + dy * cos(kickDir));
        const double rangeWeight = (lookahead - forward) / lookahead;
        if (lateral < laneWidth) {
            result.blocked = true;
            result.score += 100.0 + (laneWidth - lateral) * 30.0 + rangeWeight * 10.0;
        } else {
            result.score += rangeWeight / max(0.1, lateral - laneWidth);
        }
    }

    Point2D endpoint = {
        ball.posToField.x + 1.8 * cos(kickDir),
        ball.posToField.y + 1.8 * sin(kickDir)
    };
    if (!isPointInField(endpoint, fd, 0.25)) {
        result.score += 25.0;
    }

    return result;
}

bool chooseSideFrontKickDir(
    const GameObject &ball,
    double baseKickDir,
    const vector<GameObject> &obstacles,
    const FieldDimensions &fd,
    int &preferredSide,
    double &selectedKickDir)
{
    const double sideFrontAngle = 0.55;
    auto baseScore = scoreKickLane(ball, baseKickDir, obstacles, fd);
    if (!baseScore.blocked) {
        preferredSide = 0;
        return false;
    }

    auto leftScore = scoreKickLane(ball, toPInPI(baseKickDir + sideFrontAngle), obstacles, fd);
    auto rightScore = scoreKickLane(ball, toPInPI(baseKickDir - sideFrontAngle), obstacles, fd);

    int bestSide = leftScore.score <= rightScore.score ? 1 : -1;
    if (preferredSide == 1 && leftScore.score <= rightScore.score + 1.0) bestSide = 1;
    if (preferredSide == -1 && rightScore.score <= leftScore.score + 1.0) bestSide = -1;

    preferredSide = bestSide;
    selectedKickDir = toPInPI(baseKickDir + bestSide * sideFrontAngle);
    return true;
}

bool isOwnBackfieldGoalThreat(const Point &ballPos, const FieldDimensions &fd)
{
    const double threatFrontX = -fd.length / 2.0 + fd.penaltyAreaLength + 2.0;
    const double threatHalfWidth = fd.width / 2.0;
    return ballPos.x < threatFrontX && fabs(ballPos.y) < threatHalfWidth;
}

double calcClearanceMaxAngle(const Point &ballPos, const FieldDimensions &fd)
{
    const double ownGoalX = -fd.length / 2.0;
    const double farClearX = ownGoalX + fd.penaltyAreaLength + 2.0;
    const double progress = cap((ballPos.x - ownGoalX) / max(0.1, farClearX - ownGoalX), 1.0, 0.0);
    const double maxAngleDeg = 50.0 - progress * 20.0;
    return deg2rad(maxAngleDeg);
}

bool isClearanceAngleGood(
    double clearanceAngleToField,
    const Point &ballPos,
    const FieldDimensions &fd)
{
    const double maxClearanceAngle = calcClearanceMaxAngle(ballPos, fd);
    return fabs(toPInPI(clearanceAngleToField)) < maxClearanceAngle;
}

double calcClearKickDir(
    const GameObject &ball,
    const Pose2D &robotPose,
    const FieldDimensions &fd)
{
    const double maxClearanceAngle = calcClearanceMaxAngle(ball.posToField, fd);
    const double kickPointDist = 0.5;
    const double robotToBallDir = atan2(
        ball.posToField.y - robotPose.y,
        ball.posToField.x - robotPose.x);
    const double lateralToBall = robotPose.y - ball.posToField.y;
    const int preferredSign = lateralToBall > 0.08 ? -1 : (lateralToBall < -0.08 ? 1 : 0);

    vector<double> candidateDirs = {0.0};
    for (double deg : {10.0, 20.0, 30.0, 40.0, 50.0}) {
        if (preferredSign != 0) {
            candidateDirs.push_back(preferredSign * deg2rad(deg));
            candidateDirs.push_back(-preferredSign * deg2rad(deg));
        } else {
            candidateDirs.push_back(deg2rad(deg));
            candidateDirs.push_back(-deg2rad(deg));
        }
    }

    double bestDir = 0.0;
    double bestScore = 1e9;
    for (double dir : candidateDirs) {
        if (fabs(dir) > maxClearanceAngle + 1e-6) continue;

        Pose2D kickPoint = calcKickSetupPose(ball, dir, kickPointDist);
        double score = norm(kickPoint.x - robotPose.x, kickPoint.y - robotPose.y);
        score += 0.35 * fabs(toPInPI(dir - robotToBallDir));
        if (preferredSign != 0 && dir * preferredSign < -1e-3) {
            score += 0.5;
        }
        if (isBallInSetupPath(robotPose, kickPoint, ball.posToField, 0.25)) {
            score += 3.0;
        }
        if (!isPointInField(Point2D({kickPoint.x, kickPoint.y}), fd, 0.2)) {
            score += 5.0;
        }

        if (score < bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }

    return bestDir;
}

struct SetupApproachCandidate
{
    Pose2D target;
    double side = 0.0;
    double score = 1e9;
    string phase = "direct";
};

struct AssistCandidate
{
    Pose2D target;
    double score = 1e9;
    string name = "fallback";
};

Pose2D clampAssistTarget(Pose2D target, const FieldDimensions &fd, double distToGoalline)
{
    target.x = cap(target.x, fd.length / 2.0 - 0.7, -fd.length / 2.0 + distToGoalline);
    target.y = cap(target.y, fd.width / 2.0 - 0.5, -fd.width / 2.0 + 0.5);
    return target;
}

Pose2D makeLegacyAssistTarget(
    const Point &ballPos,
    bool has2Assists,
    bool isSecondary,
    const FieldDimensions &fd,
    double distToGoalline)
{
    Pose2D target;
    target.x = isSecondary ? ballPos.x - 6.0 : ballPos.x - 2.0;
    double assistSide = ballPos.y > 0.0 ? -1.0 : 1.0;
    double lateralOffset = has2Assists ? (isSecondary ? 2.5 : 1.2) : 1.5;
    target.y = ballPos.y + assistSide * lateralOffset;
    if (target.y * ballPos.y > 0.0) target.y = assistSide * 0.5;
    target.theta = atan2(ballPos.y - target.y, ballPos.x - target.x);
    return clampAssistTarget(target, fd, distToGoalline);
}

double assistLinePenalty(
    const Pose2D &from,
    const Pose2D &to,
    const vector<GameObject> &obstacles,
    double safeClearance,
    double weight)
{
    double penalty = 0.0;
    for (auto &obs : obstacles) {
        if (obs.label == "Ball") continue;
        penalty += segmentClearancePenalty(from, to, obs.posToField, safeClearance, weight);
    }
    return penalty;
}

double assistTeammatePenalty(
    const Pose2D &target,
    const TMStatus tmStatus[HL_MAX_NUM_PLAYERS],
    int selfIdx)
{
    double penalty = 0.0;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;
        auto status = tmStatus[i];
        if (!status.isAlive) continue;

        double d = norm(target.x - status.robotPoseToField.x, target.y - status.robotPoseToField.y);
        if (d < 0.8) {
            penalty += (0.8 - d) * 5.0;
        } else if (d < 1.4) {
            penalty += (1.4 - d) * 0.8;
        }
    }
    return penalty;
}

double scoreAssistTarget(
    const Pose2D &target,
    const Pose2D &robotPose,
    const GameObject &ball,
    const vector<GameObject> &obstacles,
    const FieldDimensions &fd,
    const TMStatus tmStatus[HL_MAX_NUM_PLAYERS],
    int selfIdx,
    bool hasLastTarget,
    const Pose2D &lastTarget)
{
    Pose2D ballPose({ball.posToField.x, ball.posToField.y, 0.0});
    Pose2D goalPose({fd.length / 2.0, 0.0, 0.0});
    double passDist = dist2D(ballPose, target);
    double reachCost = dist2D(robotPose, target);
    double turnCost = fabs(toPInPI(atan2(ball.posToField.y - target.y, ball.posToField.x - target.x) - robotPose.theta)) * 0.15;

    double passDistancePenalty = 0.0;
    if (passDist < 1.0) passDistancePenalty += (1.0 - passDist) * 4.0;
    if (passDist > 4.5) passDistancePenalty += (passDist - 4.5) * 0.9;

    double passLinePenalty = assistLinePenalty(ballPose, target, obstacles, 0.42, 7.0);
    double directShotBlockPenalty = segmentClearancePenalty(ballPose, goalPose, Point({target.x, target.y, 0.0}), 0.55, 4.0);

    GameObject virtualBall = ball;
    virtualBall.posToField.x = target.x;
    virtualBall.posToField.y = target.y;
    double shotDir = atan2(-target.y, fd.length / 2.0 - target.x);
    auto shotLane = scoreKickLane(virtualBall, shotDir, obstacles, fd);
    double shotLanePenalty = shotLane.score * 0.015 + (shotLane.blocked ? 1.2 : 0.0);

    double goalProgress = (target.x + fd.length / 2.0) / max(0.1, fd.length);
    double goalProgressReward = cap(goalProgress, 1.0, 0.0) * 0.8;
    double teammatePenalty = assistTeammatePenalty(target, tmStatus, selfIdx);
    double hysteresisReward = hasLastTarget && dist2D(target, lastTarget) < 0.8 ? 0.35 : 0.0;

    return reachCost
        + turnCost
        + passDistancePenalty
        + passLinePenalty
        + directShotBlockPenalty
        + shotLanePenalty
        + fieldBorderPenalty(target, fd)
        + teammatePenalty
        - goalProgressReward
        - hysteresisReward;
}

bool isSegmentClearOfObstacles(
    const Pose2D &from,
    const Point &to,
    const vector<GameObject> &obstacles,
    double clearance,
    double obstacleConfidenceThreshold)
{
    Line path = {from.x, from.y, to.x, to.y};
    for (auto &obs : obstacles) {
        if (obs.label == "Ball") continue;
        if (obs.confidence < obstacleConfidenceThreshold) continue;
        if (pointMinDistToLine(Point2D({obs.posToField.x, obs.posToField.y}), path) < clearance) {
            return false;
        }
    }
    return true;
}

SetupApproachCandidate makeSetupCandidate(
    const Pose2D &robotPose,
    const GameObject &ball,
    const Pose2D &kickPoint,
    double kickDir,
    double side,
    double kickPointDist,
    double ballPathClearance,
    double detourReleaseClearance,
    double lateralOffset,
    const FieldDimensions &fd,
    const vector<GameObject> &obstacles,
    bool detourActive,
    double activeSide)
{
    SetupApproachCandidate candidate;
    candidate.side = side;

    double lateral = calcSetupLateralOffset(robotPose, ball.posToField, kickDir);
    double signedLateral = side * lateral;
    if (signedLateral >= detourReleaseClearance) {
        double laneLateralOffset = side * cap(signedLateral, lateralOffset, detourReleaseClearance);
        candidate.target = calcSetupLanePose(ball, kickDir, -kickPointDist, laneLateralOffset, fd);
        candidate.phase = "lane";
    } else {
        candidate.target = calcSetupTangentPose(robotPose, ball, kickDir, side, lateralOffset, fd);
        candidate.phase = "tangent";
    }

    double targetDir = atan2(candidate.target.y - robotPose.y, candidate.target.x - robotPose.x);
    double pathCost = dist2D(robotPose, candidate.target) + 0.65 * dist2D(candidate.target, kickPoint);
    double turnCost = fabs(toPInPI(targetDir - robotPose.theta)) * 0.25;
    double finalPoseCost = fabs(toPInPI(kickDir - candidate.target.theta)) * 0.2;
    double ballPenalty =
        segmentClearancePenalty(robotPose, candidate.target, ball.posToField, ballPathClearance, 12.0)
        + segmentClearancePenalty(candidate.target, kickPoint, ball.posToField, ballPathClearance * 0.85, 4.0);
    double borderPenalty = fieldBorderPenalty(candidate.target, fd);
    double obstaclePenalty = obstaclePathPenalty(robotPose, candidate.target, kickPoint, obstacles, 0.42);
    double switchPenalty = (detourActive && side != activeSide) ? 0.8 : 0.0;

    candidate.score = pathCost + turnCost + finalPoseCost + ballPenalty + borderPenalty + obstaclePenalty + switchPenalty;
    return candidate;
}
}

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(SmartFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(PrepareKickPose)
    REGISTER_BUILDER(FaceKickTarget)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    // REGISTER_BUILDER(SelfLocate)
    // REGISTER_BUILDER(SelfLocateEnterField)
    // REGISTER_BUILDER(SelfLocate1M)
    // REGISTER_BUILDER(SelfLocateBorder)
    // REGISTER_BUILDER(SelfLocate2T)
    // REGISTER_BUILDER(SelfLocateLT)
    // REGISTER_BUILDER(SelfLocatePT)
    // REGISTER_BUILDER(SelfLocate2X)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(GoalieBlock)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(Assist)

    // 注册 Locator 相关的节点
    brain->registerLocatorNodes(factory);

    // Action Nodes for debug
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)
    REGISTER_BUILDER(PlaySound)
    REGISTER_BUILDER(Speak)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // 构造完成后，初始化 blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);
    setEntry<double>("goalie_block_x", 0);
    setEntry<double>("goalie_block_y", 0);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("need_check_behind", false);

    setEntry<bool>("is_lead", true); 
    setEntry<string>("goalie_mode", "attack"); 

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // 自动视觉校准相关
    setEntry<string>("calibrate_state", "pitch");
    setEntry<double>("calibrate_pitch_center", 0.0);
    setEntry<double>("calibrate_pitch_step", 1.0);
    setEntry<double>("calibrate_yaw_center", 0.0);
    setEntry<double>("calibrate_yaw_step", 1.0);
    setEntry<double>("calibrate_z_center", 0.0);
    setEntry<double>("calibrate_z_step", 0.01);
}

void BrainTree::tick()
{
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw, ballX, ballY, deltaX, deltaY;
    const double pixToleranceX = brain->config->camPixX / 4.; 
    const double pixToleranceY = brain->config->camPixY / 4.;
    const double xCenter = brain->config->camPixX / 2;
    const double yCenter = brain->config->camPixY / 2; 

    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/CamTrackBall", rerun::TextLog(msg));
    };
    auto logTrackingBox = [=](int color, string label) {
        brain->log->setTimeNow();
        vector<rerun::Vec2D> mins;
        vector<rerun::Vec2D> sizes;
        mins.push_back(rerun::Vec2D{xCenter - pixToleranceX, yCenter - pixToleranceY});
        sizes.push_back(rerun::Vec2D{pixToleranceX * 2, pixToleranceY * 2});
        brain->log->log(
            "image/track_ball",
            rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                .with_labels({label})
                .with_colors(color)
        );   

    };

    bool iSeeBall = brain->data->ballDetected;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
        return NodeStatus::SUCCESS;

    if (!iSeeBall)
    { 
        if (iKnowBallPos) {
            pitch = brain->data->ball.pitchToRobot;
            yaw = brain->data->ball.yawToRobot;
        } else if (tmBallPosReliable) {
            pitch = brain->data->tmBall.pitchToRobot;
            yaw = brain->data->tmBall.yawToRobot;
        } else {
            log("reached impossible condition");
        }
        logTrackingBox(0x000000FF, "ball not detected"); 
    }
    else {      
        ballX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin);
        ballY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin);
        deltaX = ballX - xCenter;
        deltaY = ballY - yCenter; 
        
        if (std::fabs(deltaX) < pixToleranceX && std::fabs(deltaY) < pixToleranceY)
        {
            auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f", ballX, ballY, deltaX, deltaY);
            logTrackingBox(0x00FF00FF, label);
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
        auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f, pitch: %.1f, yaw: %.1f", ballX, ballY, deltaX, deltaY, pitch, yaw);
        logTrackingBox(0xFF0000FF, label);
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 1.0;
    double highPitch = 0.45;
    double leftYaw = 1.1;
    double rightYaw = -1.1;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 200;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    } 
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {                 
        _cmdIndex = 0; 
    }
    else
    { 
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Chase4", rerun::TextLog(msg));
    };
    log("ticked");
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;


    double vx, vy, vtheta;
    Pose2D target_f, target_r; 
    static string targetType = "direct"; 
    static double circleBackDir = 1.0; 
    double kickBallAngleError = fabs(toPInPI(kickDir - theta_rb));
    bool kickBallAngleObtuse = kickBallAngleError > M_PI / 2.0;
    double dirThreshold = M_PI / 2;
    if (targetType == "direct" && !kickBallAngleObtuse) dirThreshold *= 1.2;


    // calculate target point
    if (kickBallAngleError < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
    brain->log->setTimeNow();
    brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);
            
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 1.0;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
    } else {
        vx = 1.0; //min(vxLimit, brain->data->ball.range);
        vy = 0;
        vtheta = targetDir;
        if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
        vx *= sigmoid((fabs(vtheta)), 1, 3); 
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0.7 + vx * 0.3;
    smoothVy = smoothVy * 0.7 + vy * 0.3;
    smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

    // brain->client->setVelocity(smoothVx, smoothVy, smoothVtheta, false, false, false);
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0; 

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -1.0);    
    vy = cap(vy, vyLimit, -vyLimit); 

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
        // if (fabs(brain->data->ball.yawToRobot) < stopAngle) vtheta = 0; 
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


NodeStatus GoToFreekickPosition::onStart() {
    brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
    _isInFinalAdjust = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");


    string side;
    getInput("side", side);
    if (side !="attack" && side != "defense") return NodeStatus::SUCCESS;
    
    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    if (side == "attack") {
        double targetDir = brain->data->kickDir;
       double dist;
       getInput("attack_dist", dist);

       targetPose.x = ballPos.x - dist * cos(targetDir);
       targetPose.y = ballPos.y - dist * sin(targetDir);
       targetPose.theta = targetDir;

        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = 0;
                targetPose.x -= 1.5;
                if (targetPose.x < -fd.length / 2.0 + fd.goalAreaLength) targetPose.x = -fd.length / 2.0 + fd.goalAreaLength;
                auto buffer = 2.0;
                auto targetXPose = brain->config->fieldDimensions.length / 2 - buffer;
                if (targetPose.x > targetXPose) {
                    targetPose.x = targetXPose;
                    targetPose.theta = 0;
                }
            }
        }

    } else if (side == "defense") {
        double targetDir = atan2(ballPos.y, ballPos.x + fd.length / 2);
        double dist;
        getInput("defense_dist", dist);
        targetPose.x = ballPos.x - dist * cos(targetDir);
        targetPose.y = ballPos.y - dist * sin(targetDir);
        targetPose.theta = targetDir;
        if (ballPos.x < -fd.length / 2 + 1.0)  targetPose.x = -fd.length / 2 + 1.5;

        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = targetPose.y > 0 ? targetPose.y - 1.0 : targetPose.y + 1.0;
            }
        }
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double deltaDir = toPInPI(targetPose.theta - robotPose.theta);


    if ( 
        dist < 0.2 
        && fabs(deltaDir) < 0.1
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() || dist < 1.0 || _isInFinalAdjust) {
        _isInFinalAdjust = true; 
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 4.0; 

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
        vx *= linearFactor;
        vy *= linearFactor;


        Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
        if (
            pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.5
            && brain->data->ball.range < 1.0
        ) {
            vx = min(0.0, vx);
            vy = vy >= 0 ? vy + 0.1: vy - 0.1;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);
        vx = cap(vx, vxLimit, -1.0);    
        vy = cap(vy, vyLimit, -vyLimit);    
        

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->log->log("debug/freekick_position", rerun::TextLog(format("stage move: targetPose: (%.2f, %.2f, %.2f)", targetPose.x, targetPose.y, targetPose.theta)));
    brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

    return NodeStatus::RUNNING;
}

void GoToFreekickPosition::onHalted() {
    brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
}

NodeStatus GoToGoalBlockingPosition::tick() {
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/GoToGoalBlockingPosition", rerun::TextLog(msg));
    };
    log("GoToGoalBlockingPosition ticked");

    brain->log->setTimeNow();
    brain->log->log("tree/GoToGoalBlockingPosition", rerun::TextLog("GoToGoalBlockingPosition tick"));
    
    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    string curRole = brain->tree->getEntry<string>("player_role");

    Pose2D targetPose;
    if (curRole == "striker") {
        targetPose.x = std::max(-fd.length / 2.0 + distToGoalline, ballPos.x - 1.5);
        if (ballPos.x + fd.length / 2.0 < distToGoalline) {
            targetPose.y = ballPos.y > 0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0;
        } else {
            targetPose.y = ballPos.y * distToGoalline / (ballPos.x + fd.length / 2.0);
            targetPose.y = cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0);
        }
    } else {
        double goalCenterX = -fd.length / 2.0;
        double dx = ballPos.x - goalCenterX;
        double dy = ballPos.y;
        double ballDistToGoalCenter = norm(dx, dy);
        double ratio = ballDistToGoalCenter > distToGoalline
            ? distToGoalline / ballDistToGoalCenter
            : 0.5;
        targetPose.x = goalCenterX + dx * ratio;
        targetPose.y = cap(dy * ratio, fd.penaltyAreaWidth / 2.0, -fd.penaltyAreaWidth / 2.0);
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // 认为到达了目标位置
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0; 


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -vxLimit);    
    vy = cap(vy, vyLimit, -vyLimit);    
    

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieBlock::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/GoalieBlock", rerun::TextLog(msg));
    };

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double vxLimit = getInput<double>("vx_limit").value();
    double vyLimit = getInput<double>("vy_limit").value();
    double vthetaLimit = getInput<double>("vtheta_limit").value();
    double vthetaGain = getInput<double>("vtheta_gain").value();

    auto fd = brain->config->fieldDimensions;
    auto robotPose = brain->data->robotPoseToField;
    auto ballPos = brain->data->ball.posToField;

    Pose2D targetPose;
    targetPose.x = brain->tree->getEntry<double>("goalie_block_x");
    targetPose.y = brain->tree->getEntry<double>("goalie_block_y");
    targetPose.x = cap(targetPose.x, -fd.length / 2.0 + fd.goalAreaLength, -fd.length / 2.0 + 0.4);
    targetPose.y = cap(targetPose.y, fd.goalWidth / 2.0 + 0.4, -fd.goalWidth / 2.0 - 0.4);
    targetPose.theta = atan2(ballPos.y - robotPose.y, ballPos.x - robotPose.x);

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double thetaError = toPInPI(targetPose.theta - robotPose.theta);
    if (dist < distTolerance && fabs(thetaError) < thetaTolerance) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = cap(targetPose_r.x, vxLimit, -vxLimit);
    double vy = cap(targetPose_r.y, vyLimit, -vyLimit);
    double vtheta = cap(thetaError * vthetaGain, vthetaLimit, -vthetaLimit);

    log(format("target: (%.2f, %.2f), dist: %.2f, thetaErr: %.2f, vel: (%.2f, %.2f, %.2f)",
        targetPose.x, targetPose.y, dist, thetaError, vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
    static bool hasLastTarget = false;
    static Pose2D lastTarget;

    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Assist", rerun::TextLog(msg));
    };
    log("ticked");

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;
    string curRole = brain->tree->getEntry<string>("player_role");

    bool isSecondary = false; 
    bool has2Assists = false;
    int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; 

        auto tmStatus = brain->data->tmStatus[i];
        if (!tmStatus.isAlive) continue; 
        if (tmStatus.isLead) continue; 
        if (tmStatus.role != "striker") continue; 

        has2Assists = true;
        log("2 assists found");
        if (tmStatus.robotPoseToField.x > robotPose.x) {
            log("i am secondary");
            isSecondary = true; 
        }
    }
    log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));


    double assistSide = fabs(ballPos.y) > 0.2
        ? (ballPos.y > 0.0 ? -1.0 : 1.0)
        : (robotPose.y > 0.0 ? -1.0 : 1.0);
    double safeGoalSideX = fd.length / 2.0 - 1.0;
    double wideY = cap(assistSide * max(1.2, fabs(ballPos.y) + 0.8), fd.width / 2.0 - 0.7, -fd.width / 2.0 + 0.7);
    double centerSideY = assistSide * 0.7;
    auto obstacles = brain->data->getObstacles();

    vector<AssistCandidate> candidates;
    auto addCandidate = [&](string name, double x, double y, double bias = 0.0) {
        Pose2D target = clampAssistTarget(Pose2D({x, y, 0.0}), fd, distToGoalline);
        target.theta = atan2(ballPos.y - target.y, ballPos.x - target.x);
        AssistCandidate candidate;
        candidate.target = target;
        candidate.name = name;
        candidate.score = scoreAssistTarget(
            target,
            robotPose,
            brain->data->ball,
            obstacles,
            fd,
            brain->data->tmStatus,
            selfIdx,
            hasLastTarget,
            lastTarget) + bias;
        candidates.push_back(candidate);
    };

    Pose2D legacyTarget = makeLegacyAssistTarget(ballPos, has2Assists, isSecondary, fd, distToGoalline);
    addCandidate("legacy", legacyTarget.x, legacyTarget.y, 0.25);

    if (isSecondary) {
        addCandidate("secondary_safety", ballPos.x - 4.2, wideY, -0.15);
        addCandidate("secondary_center", ballPos.x - 5.0, centerSideY, 0.0);
        addCandidate("secondary_far_side", ballPos.x - 3.0, ballPos.y + assistSide * 2.4, 0.05);
    } else {
        addCandidate("near_support", ballPos.x - 1.4, ballPos.y + assistSide * 1.5, -0.1);
        addCandidate("wide_support", ballPos.x - 0.5, wideY, -0.05);
        addCandidate("shoot_pocket", min(ballPos.x + 1.1, safeGoalSideX), assistSide * 1.2, -0.2);
        addCandidate("center_pocket", min(ballPos.x + 0.7, safeGoalSideX), centerSideY, 0.0);
        if (!has2Assists) {
            addCandidate("late_run", min(ballPos.x + 1.7, safeGoalSideX), wideY, 0.1);
        }
    }

    AssistCandidate bestCandidate = candidates.front();
    for (auto &candidate : candidates) {
        if (candidate.score < bestCandidate.score) {
            bestCandidate = candidate;
        }
    }

    Pose2D targetPose = bestCandidate.target;
    lastTarget = targetPose;
    hasLastTarget = true;
    log(format(
        "target %s: (%.2f, %.2f), score: %.2f, side: %.0f",
        bestCandidate.name.c_str(),
        targetPose.x,
        targetPose.y,
        bestCandidate.score,
        assistSide));


    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( 
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx, vy, vtheta;
    auto targetPose_r = brain->data->field2robot(targetPose);
    double targetDir = atan2(targetPose_r.y, targetPose_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = brain->data->ball.yawToRobot;
    } else {
        vx = targetPose_r.x;
        vy = targetPose_r.y;
        vtheta = brain->data->ball.yawToRobot * 4.0; 
    }


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -1.0);     
    vy = cap(vy, vyLimit, -vyLimit);     
    

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    auto log = [=](string msg) { 
        brain->log->setTimeNow();
        brain->log->log("debug/adjust5", rerun::TextLog(msg)); 
    };
    log("enter");
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    // double st = cap(fabs(deltaDir), st_far, st_near);
    double st = st_far; 
    double R = ballRange; 
    double r = range;
    double sr = cap(R - r, 0.5, 0); 
    log(format("R: %.2f, r: %.2f, sr: %.2f", R, r, sr));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
        // sr = 0.;
        // vxLimit = 0.1;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 
    // vtheta = toPInPI(ballYaw + st / R * (deltaDir > 0 ? 1.0 : -1.0)); 
    vtheta = ballYaw;
    vtheta *= vtheta_factor; 

    if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.; 
    if (
        fabs(ballYaw) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { 
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus PrepareKickPose::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/PrepareKickPose", rerun::TextLog(msg));
    };

    if (!(brain->tree->getEntry<bool>("ball_location_known") || brain->tree->getEntry<bool>("tm_ball_pos_reliable")))
    {
        _ballDetourActive = false;
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double kickPointDist, vxLimit, vyLimit, vthetaLimit, vthetaGain, distTolerance, ballPathClearance, ballDetourLateralOffset;
    getInput("kick_point_dist", kickPointDist);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("vtheta_gain", vthetaGain);
    getInput("dist_tolerance", distTolerance);
    getInput("ball_path_clearance", ballPathClearance);
    getInput("ball_detour_lateral_offset", ballDetourLateralOffset);
    double ballXTolerance, ballYTolerance, ballYawTolerance, orientTolerance, preciseAdjustRange, preciseStartDist;
    getInput("ball_x_tolerance", ballXTolerance);
    getInput("ball_y_tolerance", ballYTolerance);
    getInput("ball_yaw_tolerance", ballYawTolerance);
    getInput("orient_tolerance", orientTolerance);
    getInput("precise_adjust_range", preciseAdjustRange);
    getInput("precise_start_dist", preciseStartDist);
    bool strictBehindFirst;
    getInput("strict_behind_first", strictBehindFirst);
    const double detourReleaseClearance = ballPathClearance * 1.15;
    const double effectiveBallDetourLateralOffset = max(ballDetourLateralOffset, detourReleaseClearance);

    auto kickPoint_f = calcKickSetupPose(brain->data->ball, brain->data->kickDir, kickPointDist);
    auto target_f = kickPoint_f;
    auto robotPose = brain->data->robotPoseToField;
    auto ball = brain->data->ball;

    double dx = robotPose.x - ball.posToField.x;
    double dy = robotPose.y - ball.posToField.y;
    double forward = dx * cos(brain->data->kickDir) + dy * sin(brain->data->kickDir);
    double setupKickBallAngleError = fabs(toPInPI(brain->data->kickDir - brain->data->robotBallAngleToField));
    bool setupKickBallAngleObtuse = setupKickBallAngleError > M_PI / 2.0;
    bool robotOnWrongSideOfBall = forward > 0.05 || setupKickBallAngleObtuse;
    bool ballBlocksDirectPath = isBallInSetupPath(
        robotPose,
        kickPoint_f,
        ball.posToField,
        ballPathClearance);

    bool directPathReady = !robotOnWrongSideOfBall && !ballBlocksDirectPath;
    if (_ballDetourActive && directPathReady && forward < -0.05) {
        _ballDetourActive = false;
    }

    // Refresh temporary ball obstacle only while PrepareKickPose is ticking.
    auto obstacles = brain->data->getObstacles();
    vector<GameObject> refreshedObstacles;
    for (auto &obs : obstacles) {
        if (obs.label != "Ball") {
            refreshedObstacles.push_back(obs);
        }
    }

    bool detourNeeded = _ballDetourActive || !directPathReady;
    if (detourNeeded) {
        GameObject ballObstacle = ball;
        ballObstacle.label = "Ball";
        ballObstacle.confidence = max<double>(
            ballObstacle.confidence,
            brain->get_parameter("obstacle_avoidance.occupancy_threshold").as_int()
        );
        ballObstacle.timePoint = brain->get_clock()->now();

        // Ensure both coordinate frames are current for distToObstacle().
        brain->updateRelativePos(ballObstacle);
        refreshedObstacles.push_back(ballObstacle);

        log(format(
            "add temporary ball obstacle, forward: %.2f, angleErr: %.2f, pathBlocked: %d",
            forward,
            setupKickBallAngleError,
            ballBlocksDirectPath));
    }

    brain->data->setObstacles(refreshedObstacles);

    if (detourNeeded) {
        auto leftCandidate = makeSetupCandidate(
            robotPose,
            ball,
            kickPoint_f,
            brain->data->kickDir,
            1.0,
            kickPointDist,
            ballPathClearance,
            detourReleaseClearance,
            effectiveBallDetourLateralOffset,
            brain->config->fieldDimensions,
            refreshedObstacles,
            _ballDetourActive,
            _ballDetourSide);
        auto rightCandidate = makeSetupCandidate(
            robotPose,
            ball,
            kickPoint_f,
            brain->data->kickDir,
            -1.0,
            kickPointDist,
            ballPathClearance,
            detourReleaseClearance,
            effectiveBallDetourLateralOffset,
            brain->config->fieldDimensions,
            refreshedObstacles,
            _ballDetourActive,
            _ballDetourSide);

        auto bestCandidate = leftCandidate.score <= rightCandidate.score ? leftCandidate : rightCandidate;
        _ballDetourActive = true;
        _ballDetourSide = bestCandidate.side;
        target_f = bestCandidate.target;
        log(format(
            "detour %s side: %.0f score L/R: %.2f/%.2f wrongSide: %d blocked: %d",
            bestCandidate.phase.c_str(),
            bestCandidate.side,
            leftCandidate.score,
            rightCandidate.score,
            robotOnWrongSideOfBall,
            ballBlocksDirectPath));
    } else {
        target_f = kickPoint_f;
        log("direct path to kick point");
    }

    auto target_r = brain->data->field2robot(target_f);
    double targetDist = norm(target_r.x, target_r.y);

    brain->log->setTimeNow();
    brain->log->logBall("field/striker_kick_point", Point({kickPoint_f.x, kickPoint_f.y, 0}), 0x00FFFFFF, false, false);
    if (target_f.x != kickPoint_f.x || target_f.y != kickPoint_f.y) {
        brain->log->logBall("field/striker_ball_detour_point", Point({target_f.x, target_f.y, 0}), 0xFF00FFFF, false, false);
    }

    if (!strictBehindFirst && !detourNeeded && ball.range < preciseAdjustRange && targetDist < preciseStartDist) {
        const double xError = ball.posToRobot.x - kickPointDist;
        const double yError = ball.posToRobot.y;
        const double orientError = toPInPI(brain->data->kickDir - robotPose.theta);
        const bool ballCentered = isBallCenteredInFront(ball, kickPointDist, ballXTolerance, ballYTolerance, ballYawTolerance);
        const bool bodyAligned = fabs(orientError) < orientTolerance;
        if (ballCentered && bodyAligned) {
            brain->client->setVelocity(0, 0, 0);
            return NodeStatus::SUCCESS;
        }

        const double deltaDir = toPInPI(brain->data->kickDir - brain->data->robotBallAngleToField);
        const bool closeEnoughToOrbit =
            fabs(xError) < ballXTolerance * 2.2
            && fabs(yError) < ballYTolerance * 2.4;
        const bool orbitUseful =
            closeEnoughToOrbit
            && fabs(deltaDir) > orientTolerance * 0.8
            && fabs(ball.yawToRobot) < ballYawTolerance * 2.5;
        if (orbitUseful) {
            const double orbitSide = deltaDir > 0.0 ? -1.0 : 1.0;
            const double tangentDirRobot = toPInPI(
                brain->data->robotBallAngleToField + M_PI / 2.0 * orbitSide - robotPose.theta);
            const double radialDirRobot = ball.yawToRobot;
            double tangentSpeed = cap(fabs(deltaDir) * 0.45, min(vyLimit, 0.28), 0.12);
            double radialSpeed = cap(xError * 0.5, min(vxLimit, 0.24), -min(vxLimit, 0.18));

            double vx = tangentSpeed * cos(tangentDirRobot) + radialSpeed * cos(radialDirRobot);
            double vy = tangentSpeed * sin(tangentDirRobot) + radialSpeed * sin(radialDirRobot);
            double vtheta = cap(ball.yawToRobot * min(vthetaGain, 2.4), vthetaLimit, -vthetaLimit);
            if (fabs(ball.yawToRobot) < ballYawTolerance * 0.7) vtheta = 0.0;

            vx = cap(vx, vxLimit, -min(vxLimit, 0.25));
            vy = cap(vy, vyLimit, -vyLimit);
            log(format(
                "orbit final setup deltaDir: %.2f tangent: %.2f xErr: %.2f yErr: %.2f vel: (%.2f, %.2f, %.2f)",
                deltaDir,
                tangentSpeed,
                xError,
                yError,
                vx,
                vy,
                vtheta));
            brain->client->setVelocity(vx, vy, vtheta, false, false, false);
            return NodeStatus::SUCCESS;
        }
    }

    if (targetDist < distTolerance) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = cap(target_r.x, vxLimit, -vxLimit);
    double vy = cap(target_r.y, vyLimit, -vyLimit);
    double vtheta = cap(brain->data->ball.yawToRobot * vthetaGain, vthetaLimit, -vthetaLimit);

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);
    double targetDir = atan2(target_r.y, target_r.x);
    if (avoidObstacle && brain->distToObstacle(targetDir) < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        double speed = min(0.4, norm(vx, vy));
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = cap(brain->data->ball.yawToRobot * vthetaGain, vthetaLimit, -vthetaLimit);
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus FaceKickTarget::tick()
{
    double angleTolerance, vthetaLimit, vthetaGain;
    getInput("angle_tolerance", angleTolerance);
    getInput("vtheta_limit", vthetaLimit);
    getInput("vtheta_gain", vthetaGain);

    double deltaTheta = toPInPI(brain->data->kickDir - brain->data->robotPoseToField.theta);
    if (fabs(deltaTheta) < angleTolerance) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vtheta = cap(deltaTheta * vthetaGain, vthetaLimit, -vthetaLimit);
    brain->client->setVelocity(0, 0, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    // read and log inputs
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);
    double shootGoalPostMargin;
    getInput("shoot_goal_post_margin", shootGoalPostMargin);
    static int sideFrontKickSide = 0;

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1;

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;
    auto color = 0xFFFFFFFF; // for log

    if (isOwnBackfieldGoalThreat(bPos, fd)) {
        brain->data->kickType = "clear";
        color = 0xFF0000FF;
        brain->data->kickDir = calcClearKickDir(
            brain->data->ball,
            brain->data->robotPoseToField,
            fd);
        auto clearKickPoint = calcKickSetupPose(
            brain->data->ball,
            brain->data->kickDir,
            0.5);
        brain->log->setTimeNow();
        brain->log->log(
            "debug/CalcKickDir",
            rerun::TextLog(format(
                "clear kick dir: %.2f maxAngle: %.2f kickPoint: (%.2f, %.2f)",
                brain->data->kickDir,
                calcClearanceMaxAngle(brain->data->ball.posToField, fd),
                clearKickPoint.x,
                clearKickPoint.y)));
    }
    else if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    }
    else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        color = 0xFFFF00FF;
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );

    } else { 
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = brain->calcKickDir(shootGoalPostMargin);
        if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    }

    if (brain->data->kickType == "shoot") {
        const double setupReadyRange = 1.2;
        const double setupReadyAngle = 0.9;
        const double baseKickDir = brain->data->kickDir;
        const double setupAngleError = fabs(toPInPI(baseKickDir - brain->data->robotBallAngleToField));

        if (brain->data->ball.range < setupReadyRange && setupAngleError < setupReadyAngle) {
            double sideKickDir = baseKickDir;
            auto obstacles = brain->data->getObstacles();
            if (chooseSideFrontKickDir(
                    brain->data->ball,
                    baseKickDir,
                    obstacles,
                    fd,
                    sideFrontKickSide,
                    sideKickDir)) {
                brain->data->kickDir = sideKickDir;
                color = 0xFF8800FF;
                brain->log->setTimeNow();
                brain->log->log(
                    "debug/CalcKickDir",
                    rerun::TextLog(format(
                        "side-front kick, setupAngle: %.2f, side: %d, dir: %.2f",
                        setupAngleError,
                        sideFrontKickSide,
                        sideKickDir)));
            }
        } else {
            sideFrontKickSide = 0;
        }
    } else {
        sideFrontKickSide = 0;
    }

    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);
    double kickDirTolerance, kickMaxBallYaw;
    getInput("kick_dir_tolerance", kickDirTolerance);
    getInput("kick_max_ball_yaw", kickMaxBallYaw);
    double kickPointDist, positionTolerance, orientTolerance;
    getInput("kick_point_dist", kickPointDist);
    getInput("position_tolerance", positionTolerance);
    getInput("orient_tolerance", orientTolerance);
    double kickBallXTolerance, kickBallYTolerance;
    getInput("kick_ball_x_tolerance", kickBallXTolerance);
    getInput("kick_ball_y_tolerance", kickBallYTolerance);
    int forceKickAfterMsecs;
    double forceKickToleranceScale, forceKickMaxOrientError, forceKickMaxBallYaw;
    getInput("force_kick_after_msecs", forceKickAfterMsecs);
    getInput("force_kick_tolerance_scale", forceKickToleranceScale);
    getInput("force_kick_max_orient_error", forceKickMaxOrientError);
    getInput("force_kick_max_ball_yaw", forceKickMaxBallYaw);

    double kickDir = brain->data->kickDir;
    bool fastClear = brain->data->kickType == "clear";
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    auto fd = brain->config->fieldDimensions;
    bool goalieAttackingBall = false;
    int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        auto status = brain->data->tmStatus[i];
        if (!status.isAlive) continue;
        if (status.role != "goal_keeper") continue;
        if (!status.isAttackingBall) continue;

        goalieAttackingBall = true;
        break;
    }
    auto kickPoint = calcKickSetupPose(ball, kickDir, kickPointDist);
    double positionError = norm(
        kickPoint.x - brain->data->robotPoseToField.x,
        kickPoint.y - brain->data->robotPoseToField.y
    );
    double orientError = fabs(toPInPI(kickDir - brain->data->robotPoseToField.theta));
    bool setupPositionReady = positionError <= positionTolerance;
    bool clearanceAngleGood = isClearanceAngleGood(kickDir, ball.posToField, fd);
    
    const double goalpostMargin = 0.3; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    static bool hasDirectRushBallSample = false;
    static Point lastDirectRushBallPos;
    static rclcpp::Time lastDirectRushBallTime;
    static double directRushBallSpeed = 1e9;
    static deque<double> directRushBallSpeedSamples;
    const int directRushBallVelocitySampleCount = 10;
    if (brain->data->ballDetected && (!hasDirectRushBallSample || ball.timePoint.nanoseconds() > lastDirectRushBallTime.nanoseconds())) {
        if (hasDirectRushBallSample) {
            double dt = (ball.timePoint - lastDirectRushBallTime).nanoseconds() / 1e9;
            if (dt > 0.02) {
                double observedSpeed = norm(
                    ball.posToField.x - lastDirectRushBallPos.x,
                    ball.posToField.y - lastDirectRushBallPos.y) / dt;
                directRushBallSpeedSamples.push_back(observedSpeed);
                while (static_cast<int>(directRushBallSpeedSamples.size()) > directRushBallVelocitySampleCount) {
                    directRushBallSpeedSamples.pop_front();
                }

                directRushBallSpeed = 0.0;
                for (double speed : directRushBallSpeedSamples) {
                    directRushBallSpeed += speed;
                }
                directRushBallSpeed /= directRushBallSpeedSamples.size();
            }
        }
        lastDirectRushBallPos = ball.posToField;
        lastDirectRushBallTime = ball.timePoint;
        hasDirectRushBallSample = true;
    } else if (!brain->data->ballDetected) {
        directRushBallSpeedSamples.clear();
        directRushBallSpeed = 1e9;
        hasDirectRushBallSample = false;
    }

    const double goalLineX = fd.length / 2.0;
    const double lineDx = ball.posToField.x - brain->data->robotPoseToField.x;
    const double lineDy = ball.posToField.y - brain->data->robotPoseToField.y;
    const double goalT = (goalLineX - brain->data->robotPoseToField.x) / max(0.05, lineDx);
    const double yAtGoal = brain->data->robotPoseToField.y + goalT * lineDy;
    const double directRushGoalMargin = 0.20;
    const Point directRushGoalPoint = {goalLineX, yAtGoal, 0.0};
    const bool lineHitsGoal =
        lineDx > 0.15
        && goalT > 1.0
        && fabs(yAtGoal) < fd.goalWidth / 2.0 - directRushGoalMargin;
    const double directRushPathClearance = 0.45;
    const double obstacleConfidenceThreshold = static_cast<double>(
        brain->get_parameter("obstacle_avoidance.occupancy_threshold").as_int());
    const bool directRushPathClear = isSegmentClearOfObstacles(
        brain->data->robotPoseToField,
        directRushGoalPoint,
        brain->data->getObstacles(),
        directRushPathClearance,
        obstacleConfidenceThreshold);
    const bool ballStaticForDirectRush =
        brain->data->ballDetected
        && directRushBallSpeedSamples.size() >= 3
        && directRushBallSpeed < 0.28;
    const bool directRush =
        brain->data->ballDetected
        && ballStaticForDirectRush
        && lineHitsGoal
        && directRushPathClear
        && ballRange < 2.8
        && fabs(ballYaw) < 0.28;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, posErr: %.2f, orientErr: %.2f, angleGoodForKick: %d, goalieAttacking: %d, rush: %d yGoal: %.2f ballSpeed: %.2f clear: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, positionError, orientError, angleGoodForKick, goalieAttackingBall, directRush, yAtGoal, directRushBallSpeed, directRushPathClear));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    const double clearLineTolerance = deg2rad(24);
    bool clearLineReady = fabs(deltaDir) < clearLineTolerance;
    bool clearSetupReady = setupPositionReady && clearLineReady;
    auto now = brain->get_clock()->now();
    bool kickWindowGood =
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff)
            || fabs(deltaDir) < kickDirTolerance
        )
        && fabs(ballYaw) < kickMaxBallYaw;
    bool ballCenteredForKick = isBallCenteredInFront(
        ball,
        kickPointDist,
        kickBallXTolerance,
        kickBallYTolerance,
        kickMaxBallYaw);
    const double effectiveOrientTolerance = angleGoodForKick
        ? max(orientTolerance, forceKickMaxOrientError * 0.65)
        : orientTolerance;
    bool readyToKick =
        brain->data->ballDetected
        && !avoidKick
        && setupPositionReady
        && (
            (fastClear && clearanceAngleGood && clearLineReady)
            || (
                orientError <= effectiveOrientTolerance
                && kickWindowGood
                && ballCenteredForKick
            )
        );
    const double relaxedBallYaw = max(kickMaxBallYaw, forceKickMaxBallYaw);
    const double relaxedDirTolerance = max(kickDirTolerance, forceKickMaxOrientError);
    bool goodEnoughToKick =
        brain->data->ballDetected
        && !avoidKick
        && setupPositionReady
        && (
            (fastClear && clearanceAngleGood && clearLineReady)
            || (
                orientError <= forceKickMaxOrientError
                && (
                    (angleGoodForKick && !brain->data->isFreekickKickingOff)
                    || fabs(deltaDir) < relaxedDirTolerance
                )
                && isBallCenteredInFront(
                    ball,
                    kickPointDist,
                    kickBallXTolerance * forceKickToleranceScale,
                    kickBallYTolerance * forceKickToleranceScale,
                    relaxedBallYaw)
            )
        );
    if (goodEnoughToKick) {
        if (!goodEnoughKickTimerActive) {
            timeGoodEnoughKick = now;
            goodEnoughKickTimerActive = true;
        }
    } else {
        goodEnoughKickTimerActive = false;
    }
    bool forceKick =
        goodEnoughToKick
        && (
            forceKickAfterMsecs <= 0
            || brain->msecsSince(timeGoodEnoughKick) > forceKickAfterMsecs
        );
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } else if (fastClear && !clearSetupReady) {
        newDecision = "position";
        color = 0xFF0000FF;
    } else if (fastClear && readyToKick) {
        newDecision = "clear";
        color = 0xFF0000FF;
        brain->data->isFreekickKickingOff = false;
        goodEnoughKickTimerActive = false;
    } else if (fastClear) {
        newDecision = "position";
        color = 0xFF0000FF;
    } else if (goalieAttackingBall) {
        newDecision = "assist";
        color = 0x00FFFFFF;
    } else if (!brain->data->tmImLead) {
        newDecision = "assist";
        color = 0x00FFFFFF;
    } else if (directRush) {
        newDecision = "rush";
        color = 0x00AAFFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } else if (!setupPositionReady) {
        newDecision = "position";
        color = 0xFFFF00FF;
    } else if (orientError > orientTolerance) {
        newDecision = "orient";
        color = 0xFF00FFFF;
    } else if (readyToKick || forceKick) {
        string kickDecision = brain->data->kickType == "cross" ? "cross" : "kick";
        newDecision = kickDecision;
        color = forceKick ? 0xFFAA00FF : 0x00FF00FF;
        brain->data->isFreekickKickingOff = false;
        goodEnoughKickTimerActive = false;
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }
    if (
        newDecision != "position"
        && newDecision != "orient"
        && newDecision != "adjust"
        && newDecision != "kick"
        && newDecision != "clear"
        && newDecision != "cross"
    ) {
        goodEnoughKickTimerActive = false;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f deltaDir: %.2f posErr: %.2f orientErr: %.2f readyKick: %d forceKick: %d angleGoodForKick: %d lead: %d fastClear: %d clearAngleGood: %d clearLine: %d",
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, deltaDir, positionError, orientError, readyToKick, forceKick, angleGoodForKick, brain->data->tmImLead, fastClear, clearanceAngleGood, clearLineReady
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);

    double dir_rb_f = brain->data->robotBallAngleToField;
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    double ballDistToGoalLine = ballPos.x + fd.length / 2.0;
    string lastDecision;
    getInput("decision_in", lastDecision);
    double setupRange, kickPointDist, positionTolerance, orientTolerance;
    int ballVelocitySampleCount;
    getInput("setup_range", setupRange);
    getInput("kick_point_dist", kickPointDist);
    getInput("position_tolerance", positionTolerance);
    getInput("orient_tolerance", orientTolerance);
    getInput("ball_velocity_sample_count", ballVelocitySampleCount);
    ballVelocitySampleCount = max(1, ballVelocitySampleCount);

    auto ballTime = brain->data->ball.timePoint;
    if (hasLastBallForVelocity && ballTime.nanoseconds() > lastBallVelocityTime.nanoseconds()) {
        double dt = (ballTime.nanoseconds() - lastBallVelocityTime.nanoseconds()) / 1e9;
        if (dt > 0.02 && dt < 1.0) {
            ballVxSamplesForVelocity.push_back((ballPos.x - lastBallPosForVelocity.x) / dt);
            ballVySamplesForVelocity.push_back((ballPos.y - lastBallPosForVelocity.y) / dt);
            while (static_cast<int>(ballVxSamplesForVelocity.size()) > ballVelocitySampleCount) {
                ballVxSamplesForVelocity.pop_front();
                ballVySamplesForVelocity.pop_front();
            }

            ballVxToField = 0.0;
            ballVyToField = 0.0;
            for (int i = 0; i < static_cast<int>(ballVxSamplesForVelocity.size()); i++) {
                ballVxToField += ballVxSamplesForVelocity[i];
                ballVyToField += ballVySamplesForVelocity[i];
            }
            ballVxToField /= ballVxSamplesForVelocity.size();
            ballVyToField /= ballVySamplesForVelocity.size();
        } else if (dt >= 1.0) {
            ballVxSamplesForVelocity.clear();
            ballVySamplesForVelocity.clear();
            ballVxToField = 0.0;
            ballVyToField = 0.0;
        }
    }
    if (!hasLastBallForVelocity || ballTime.nanoseconds() >= lastBallVelocityTime.nanoseconds()) {
        lastBallPosForVelocity = ballPos;
        lastBallVelocityTime = ballTime;
        hasLastBallForVelocity = true;
    }

    const double BLOCK_X = -fd.length / 2.0 + 0.6;
    const double BLOCK_TIME_MAX = lastDecision == "block" ? 9.0 : 8.5;
    const double BALL_SPEED = norm(ballVxToField, ballVyToField);
    bool ballMovingToGoal = ballVxToField < -0.15 && BALL_SPEED > 0.20;
    double blockTime = ballMovingToGoal ? (BLOCK_X - ballPos.x) / ballVxToField : -1.0;
    double blockY = ballPos.y;
    if (blockTime > 0.0) blockY = ballPos.y + ballVyToField * blockTime;
    double rawBlockY = blockY;
    bool ballLineThreatensGoal =
        ballMovingToGoal
        && blockTime > 0.0
        && blockTime < BLOCK_TIME_MAX
        && fabs(rawBlockY) < fd.goalWidth / 2.0 + 0.5;
    blockY = cap(blockY, fd.goalWidth / 2.0 + 0.4, -fd.goalWidth / 2.0 - 0.4);
    bool movingBallInGoalArea =
        BALL_SPEED > 0.15
        && ballDistToGoalLine < chaseRangeThreshold
        && fabs(ballPos.y) < fd.penaltyAreaWidth / 2.0;
    bool goalieShouldBlock = ballLineThreatensGoal || (movingBallInGoalArea && lastDecision == "block");
    brain->tree->setEntry<double>("goalie_block_x", BLOCK_X);
    brain->tree->setEntry<double>("goalie_block_y", blockY);

    auto kickPoint = calcKickSetupPose(brain->data->ball, brain->data->kickDir, kickPointDist);
    double positionError = norm(
        kickPoint.x - brain->data->robotPoseToField.x,
        kickPoint.y - brain->data->robotPoseToField.y
    );
    double orientError = fabs(toPInPI(brain->data->kickDir - brain->data->robotPoseToField.theta));

    bool strikerClearlyBetter = false;
    int selfIdx = brain->config->playerId - 1;
    const double STRIKER_COST_MARGIN = 1.5;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;
        auto status = brain->data->tmStatus[i];
        if (!status.isAlive || status.role != "striker") continue;
        if (status.cost + STRIKER_COST_MARGIN < brain->data->tmMyCost) {
            strikerClearlyBetter = true;
            break;
        }
    }

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (goalieShouldBlock)
    {
        newDecision = "block";
        color = 0xFF00FFFF;
    }
    else if (strikerClearlyBetter)
    {
        newDecision = ballRange <= chaseRangeThreshold ? "retreat_near" : "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold)
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > setupRange)
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (BALL_SPEED > 0.15)
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (positionError > positionTolerance)
    {
        newDecision = "position";
        color = 0xFFFF00FF;
    }
    else if (orientError > orientTolerance)
    {
        newDecision = "orient";
        color = 0xFF00FFFF;
    }
    else
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f goalLineDist: %.2f threshold: %.2f setupRange: %.2f posErr: %.2f orientErr: %.2f strikerBetter: %d ballVel: (%.2f, %.2f) block: %d blockY: %.2f",
                                newDecision.c_str(), ballRange, ballYaw, brain->data->kickDir, dir_rb_f, ballDistToGoalLine, chaseRangeThreshold, setupRange, positionError, orientError, strikerClearlyBetter, ballVxToField, ballVyToField, goalieShouldBlock, blockY),
                            color);
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed()    //计算踢球时运动速度
{
    double vx, vy, msecKick;

    // 获取输入参数
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);      // 前后速度限制
    getInput("vy_limit", vyLimit);      // 左右速度限制
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);  // 最小踢球时间
    double vxFactor = brain->config->vxFactor;   // 前后速度因子 0.5
    double yawOffset = brain->config->yawOffset; // 角度偏移校准 0.0

    // 球在机器人坐标系中的x、y坐标与yaw
    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; 
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)    //球在正前方
    { 
        vx = vxLimit;   //全速前进
        vy = 0.0;
    }
    else
    { 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);
    // 踢球时间
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);    //创建元组
}

NodeStatus Kick::onStart()  //初始化踢球状态并进行避障检测
{
    _minRange = brain->data->ball.range;    //初始球距
    _speed = 1.0;   //（可修改，原版：0.5）
    _startTime = brain->get_clock()->now();
    _state = "kick";

    // 避障检测
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    if (
        avoidPushing    //开启避障
        && (role != "goal_keeper")  //非守门员
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()    //执行踢球
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Kick", rerun::TextLog(msg));
    };

    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;    // 移动阈值 0.3 米
    const double BALL_LOST_THRESHOLD = 1000;    // 丢失阈值 1000ms
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS; //球发生移动，终止踢球
    }

    if (ballRange < _minRange) _minRange = ballRange;    
    
    // 二次避障检测（暂时去除）
    // bool avoidPushing;
    // brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    // double kickAoSafeDist;
    // brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    // if (
    //     avoidPushing
    //     && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
    //     && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    // ) {
    //     brain->client->setVelocity(-0.1, 0, 0);
    //     return NodeStatus::SUCCESS;
    // }


    // 达到预估踢球时间，完成踢球
    double msecs = getInput<double>("min_msec_kick").value();
    double speed = getInput<double>("speed_limit").value(); //在brain_tree.hpp的kick中修改speed_limit参数
    msecs = msecs + brain->data->ball.range / speed * 1000;
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        _speed += 0.15; 
        speed = min(speed, _speed);
        double vtheta = brain->data->kickType == "clear"
            ? 0.0
            : toPInPI(brain->data->kickDir - brain->data->robotPoseToField.theta);
        brain->client->crabWalk(angle, speed, vtheta);
    }

    return NodeStatus::RUNNING;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

NodeStatus StandStill::onStart()
{

    _startTime = brain->get_clock()->now();


    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
}

NodeStatus StandStill::onRunning()
{
    double msecs;
    getInput("msecs", msecs);
    if (brain->msecsSince(_startTime) < msecs) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }
    return NodeStatus::SUCCESS;
}

void StandStill::onHalted()
{
    double msecs;
    getInput("msecs", msecs);
    _startTime -= rclcpp::Duration(- 2 * msecs, 0);
}


NodeStatus RobotFindBall::onStart()
{
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onStart");

    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onRunning");

    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    if (brain->data->ball.range < 0.3)
    { 
      // vx = cap(-brain->data->ball.posToRobot.x, 0.2, -0.2);
      // vy = cap(-brain->data->ball.posToRobot.y, 0.2, -0.2);
    }
    // vtheta = _turnDir > 0 ? vyawLimit : -vyawLimit;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onHalted");
    _turnDir = 1.0;
}

NodeStatus SmartFindBall::onStart()
{
    _startTime = brain->get_clock()->now();
    _timeLastHeadCmd = _startTime;
    _scanIndex = 0;

    double pitch = 0.0;
    double yaw = 0.0;
    if (getBallSearchGuess(pitch, yaw)) {
        _turnDir = yaw >= 0.0 ? 1.0 : -1.0;
    } else {
        _turnDir = brain->data->headYaw >= 0.0 ? 1.0 : -1.0;
    }

    commandGuessHead(0.0);
    brain->client->setVelocity(0, 0, 0, false, false, false);
    return NodeStatus::RUNNING;
}

NodeStatus SmartFindBall::onRunning()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/SmartFindBall", rerun::TextLog(msg));
    };

    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0, false, false, false);
        return NodeStatus::SUCCESS;
    }

    double reacquireMSec, localScanMSec, globalScanMSec, headIntervalMSec;
    double bodyTurnSpeed, bodyTurnMSec, bodyPauseMSec;
    getInput("reacquire_msecs", reacquireMSec);
    getInput("local_scan_msecs", localScanMSec);
    getInput("global_scan_msecs", globalScanMSec);
    getInput("head_interval_msecs", headIntervalMSec);
    getInput("body_turn_speed", bodyTurnSpeed);
    getInput("body_turn_msecs", bodyTurnMSec);
    getInput("body_pause_msecs", bodyPauseMSec);

    const double elapsed = brain->msecsSince(_startTime);
    if (brain->msecsSince(_timeLastHeadCmd) >= headIntervalMSec) {
        if (elapsed < reacquireMSec) {
            static const double offsets[] = {0.0, 0.25, -0.25, 0.55, -0.55};
            commandGuessHead(offsets[_scanIndex % (sizeof(offsets) / sizeof(offsets[0]))]);
            _scanIndex++;
            log("phase reacquire");
        } else if (elapsed < localScanMSec) {
            commandLocalScan();
            log("phase local scan");
        } else {
            commandGlobalScan();
            log("phase global scan");
        }
        _timeLastHeadCmd = brain->get_clock()->now();
    }

    if (elapsed < globalScanMSec) {
        brain->client->setVelocity(0, 0, 0, false, false, false);
        return NodeStatus::RUNNING;
    }

    const double cycleMSec = max(1.0, bodyTurnMSec + bodyPauseMSec);
    const double cycleTime = fmod(elapsed - globalScanMSec, cycleMSec);
    if (cycleTime < bodyTurnMSec) {
        brain->client->setVelocity(0, 0, bodyTurnSpeed * _turnDir, false, false, false);
    } else {
        brain->client->setVelocity(0, 0, 0, false, false, false);
    }

    return NodeStatus::RUNNING;
}

void SmartFindBall::onHalted()
{
    brain->client->setVelocity(0, 0, 0, false, false, false);
    _scanIndex = 0;
    _turnDir = 1.0;
}

bool SmartFindBall::getBallSearchGuess(double &pitch, double &yaw)
{
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

    if (iKnowBallPos && brain->data->ball.range > 0.05) {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
        return std::isfinite(pitch) && std::isfinite(yaw);
    }

    if (tmBallPosReliable && brain->data->tmBall.range > 0.05) {
        pitch = brain->data->tmBall.pitchToRobot;
        yaw = brain->data->tmBall.yawToRobot;
        return std::isfinite(pitch) && std::isfinite(yaw);
    }

    if (
        brain->data->ball.range > 0.05
        && brain->msecsSince(brain->data->ball.timePoint) < 8000
    ) {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
        return std::isfinite(pitch) && std::isfinite(yaw);
    }

    return false;
}

void SmartFindBall::commandGuessHead(double yawOffset)
{
    double pitch = 0.85;
    double yaw = 0.0;
    if (getBallSearchGuess(pitch, yaw)) {
        yaw += yawOffset;
    } else {
        yaw = yawOffset;
    }

    brain->client->moveHead(pitch, yaw);
}

void SmartFindBall::commandLocalScan()
{
    static const double offsets[] = {0.0, 0.35, -0.35, 0.7, -0.7, 1.05, -1.05};
    commandGuessHead(offsets[_scanIndex % (sizeof(offsets) / sizeof(offsets[0]))]);
    _scanIndex++;
}

void SmartFindBall::commandGlobalScan()
{
    static const double scan[][2] = {
        {0.95, 0.0},
        {0.95, 0.65},
        {0.95, -0.65},
        {0.70, 0.0},
        {0.70, 1.05},
        {0.70, -1.05},
        {0.45, 0.0},
        {0.45, 1.05},
        {0.45, -1.05},
    };

    const int scanLen = sizeof(scan) / sizeof(scan[0]);
    brain->client->moveHead(scan[_scanIndex % scanLen][0], scan[_scanIndex % scanLen][1]);
    _scanIndex++;
}

NodeStatus CamFastScan::onStart()
{
    _cmdIndex = 0;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus CamFastScan::onRunning()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(_timeLastCmd) < interval) return NodeStatus::RUNNING;

    // else 
    if (_cmdIndex >= 6) return NodeStatus::SUCCESS;

    // else
    _cmdIndex++;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
        "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
        _angle, _cumAngle, deltaAngle, turnTime
    )));
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else 
    brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);
    return NodeStatus::RUNNING;
}

NodeStatus MoveToPoseOnField::tick()
{
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/Move", rerun::TextLog(msg));

    };
    log("Move ticked");

    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);
    bool avoidObstacle;
    getInput("avoid_obstacle", avoidObstacle);

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoToReadyPosition::tick()
{
    auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/GoToReadyPosition", rerun::TextLog(msg));
    };
    log("GoToReadyPosition ticked");

    double distTolerance, thetaTolerance;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);
    string role = brain->tree->getEntry<string>("player_role");
    bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    auto fd = brain->config->fieldDimensions;


    double tx = 0, ty = 0, ttheta = 0; 
    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    if (brain->distToBorder() > - 1.0) { 
        vxLimit = 0.5;
        vyLimit = 0.3;
    }
    double vthetaLimit = 1.3;
    bool avoidObstacle = true;

    if (role == "striker" && isKickoff) {
        tx = -2.0;
        ty = 0;
        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (brain->isPrimaryStriker()) {
                ty = 0.7;
            } else {
                ty = -0.7;
            }
        }
        ttheta = 0;
    } else if (role == "striker" && !isKickoff) {
        tx = -2.0;
        ty = 0;
        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
        {
            if (brain->isPrimaryStriker()) {
                ty = 1.0;
            } else {
                ty = -1.0;
            }
        }
        ttheta = 0;
    } else if (role == "goal_keeper") {
        tx = -fd.length / 2.0 + fd.goalAreaLength;
        ty = 0;
        ttheta = 0;
    }

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/GoBackInField", rerun::TextLog(msg));
    };
    log("GoBackInField ticked");

    double valve;
    getInput("valve", valve);
    double vx = 0; 
    double vy = 0; 
    double dir = 0;
    auto fd = brain->config->fieldDimensions;
    if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve) dir = - M_PI;
    else if (brain->data->robotPoseToField.x < - fd.length / 2.0 + valve) dir = 0;
    else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve) dir = - M_PI / 2.0;
    else if (brain->data->robotPoseToField.y < - fd.width / 2.0 - valve) dir = M_PI / 2.0;
    else { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus MoveHead::tick()
{
    double pitch, yaw;
    getInput("pitch", pitch);
    getInput("yaw", yaw);
    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 1) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }
    brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex)));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && 
        brain->data->currentRobotModeIndex == 3 && 
        brain->data->recoveryPerformedRetryCount < brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->speak("Trying to stand up");
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount)));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 12) {
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog(format("Add retry count: %d", brain->data->recoveryPerformedRetryCount)));
    }


    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->data->currentRobotModeIndex == 8) { 
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}


NodeStatus CalibrateOdom::tick()
{
    double x, y, theta;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->calibrateOdom(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus PlaySound::tick()
{
    string sound;
    getInput("sound", sound);
    bool allowRepeat;
    getInput("allow_repeat", allowRepeat);
    brain->playSound(sound, allowRepeat);
    return NodeStatus::SUCCESS;
}

NodeStatus Speak::tick()
{
    const string lastText;
    string text;
    getInput("text", text);
    if (text == lastText) return NodeStatus::SUCCESS;

    brain->speak(text, false);
    return NodeStatus::SUCCESS;
}
