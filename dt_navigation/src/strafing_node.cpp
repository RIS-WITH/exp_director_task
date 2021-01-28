#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <dt_navigation/MoveAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

class Strafing{
public:
    struct Params{
        double goalTolerance;
        double maxSpeed;
        double p;
        double i;
        double maxIntegral;
        ros::Duration controlPeriod;
        std::string tableFrame;
    };

    Strafing(ros::NodeHandle& nh): nh_(nh), strafeActionServer_(nh, "dt_strafe", boost::bind(&Strafing::onStrafe, this, _1), false){
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 10, true);
        strafeActionServer_.start();

    }

    void onStrafe(const dt_navigation::MoveGoalConstPtr& goal){
        ros::Time timeZero(0.0);
        Params params;
        getParams(params);
        
        dt_navigation::MoveResult asResult;
        std::string objectName = goal->frame_id;
        if (!tfBuffer_.canTransform(params.tableFrame, objectName, timeZero)){
            ROS_WARN_STREAM_NAMED("dt_navigation", "Can't transform frame '" << objectName << "' in table frame '" << params.tableFrame << "'. Aborting action.");
            asResult.error_code = -1;
            strafeActionServer_.setAborted(asResult);
            return;
        }
        geometry_msgs::TransformStamped table2footprintMsg, table2objectMsg, table2poseMsg, map2tableMsg;
        tf2::Transform map2table, table2pose, map2pose;
        if (goal->move_type == dt_navigation::MoveGoal::GO_IN_FRONT){
            table2footprintMsg = tfBuffer_.lookupTransform(params.tableFrame, "base_footprint", timeZero);
            table2objectMsg = tfBuffer_.lookupTransform(params.tableFrame, objectName, timeZero);
            table2poseMsg.header = table2objectMsg.header;
            table2poseMsg.transform.rotation.w = 1.0;
            table2poseMsg.transform.rotation.x = 0.0;
            table2poseMsg.transform.rotation.y = 0.0;
            table2poseMsg.transform.rotation.z = 0.0;
            table2poseMsg.transform.translation.x = table2objectMsg.transform.translation.x;
            table2poseMsg.transform.translation.y = table2footprintMsg.transform.translation.y;
            table2poseMsg.transform.translation.z = table2footprintMsg.transform.translation.z;
            table2poseMsg.child_frame_id = "target_pose";
            map2tableMsg = tfBuffer_.lookupTransform("map", params.tableFrame, timeZero);
            tf2::fromMsg(map2tableMsg.transform, map2table);
            tf2::fromMsg(table2poseMsg.transform, table2pose);
            map2pose = map2table * table2pose;
        }else if (goal->move_type == dt_navigation::MoveGoal::GO_NEXT){
            ROS_ERROR_NAMED("dt_navigation", "GO NEXT navigation type not supported yet."); //TODO (guilhembn)
        }else {
            ROS_ERROR_STREAM_NAMED("dt_navigation", "Move type asked: '" << goal->move_type << "' is not defined.");
        }

        geometry_msgs::TransformStamped map2footprintMsg = tfBuffer_.lookupTransform("map", "base_footprint", timeZero);
        tf2::Transform map2footprint;
        tf2::fromMsg(map2footprintMsg.transform, map2footprint);

        double error = tf2::tf2Distance2(map2pose.getOrigin(), map2footprint.getOrigin());
        double cmd = 0.0, integral;
        ros::Time lastControl = ros::Time::now();
        ros::Time now = lastControl;
        geometry_msgs::Twist cmdVel;
        integral = 0.0;
        tf2::Transform footprint2pose;
        while (std::abs(error) > params.goalTolerance){
            map2footprintMsg = tfBuffer_.lookupTransform("map", "base_footprint", ros::Time(0.0));
            tf2::fromMsg(map2footprintMsg.transform, map2footprint);
            footprint2pose = map2footprint.inverse() * map2pose;
            error = std::pow(footprint2pose.getOrigin().getX(), 2) + std::pow(footprint2pose.getOrigin().getY(), 2);
            getParams(params);
            now = ros::Time::now();
            if (strafeActionServer_.isPreemptRequested()){
                cmdVel.linear.x = 0.0;
                cmdVel.linear.y = 0.0;
                cmdVelPub_.publish(cmdVel);
                asResult.error_code = -2;
                strafeActionServer_.setPreempted(asResult);
                return;
            }
            integral = std::max(-params.maxIntegral, std::min(params.maxIntegral, integral + error * (now - lastControl).toSec()));
            cmd = std::max(-params.maxSpeed, std::min(params.maxSpeed, params.p * error + params.i * integral));
            double angle = std::atan2(footprint2pose.getOrigin().getY(), footprint2pose.getOrigin().getX());
            cmdVel.linear.x = cmd * std::cos(angle);
            cmdVel.linear.y = cmd * std::sin(angle);
            cmdVelPub_.publish(cmdVel);
            lastControl = now;
            params.controlPeriod.sleep();
        }
        cmdVel.linear.x = 0.0;
        cmdVel.linear.y = 0.0;
        for (unsigned int j = 0; j < 10; j++){  // TODO (guilhembn): does not work with morse otherwise, check on real robot
            cmdVelPub_.publish(cmdVel);
            params.controlPeriod.sleep();
        }
        cmdVelPub_.publish(cmdVel);
        asResult.error_code = 0;
        strafeActionServer_.setSucceeded(asResult);
    }

    void getParams(Params& params){
        double controlPeriod;
        nh_.param("goal_tolerance", params.goalTolerance, 0.2);
        params.goalTolerance *= params.goalTolerance;
        nh_.param("max_speed", params.maxSpeed, 0.2);
        nh_.param("P", params.p, 0.1);
        nh_.param("I", params.i, 0.05);
        nh_.param("max_integral", params.maxIntegral, 5.0);
        nh_.param("control_period", controlPeriod, 0.1);
        params.controlPeriod.fromSec(controlPeriod);
        nh_.param("table_frame", params.tableFrame, std::string("table_1"));
    }



protected:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    actionlib::SimpleActionServer<dt_navigation::MoveAction> strafeActionServer_;
    ros::Publisher cmdVelPub_;
};

int main (int argc, char** argv){
    ros::init(argc, argv, "dt_navigation");
    ros::NodeHandle nh("~");
    Strafing strafing(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
    
}
