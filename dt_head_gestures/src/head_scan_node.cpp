#include <ros/ros.h>
#include <pr2_head_manager_msgs/StateMachineRegister.h>
#include <resource_management_msgs/StateMachinesStatus.h>
#include <resource_management_msgs/StateMachinesCancel.h>
#include <actionlib/server/simple_action_server.h>
#include <dt_head_gestures/HeadScanAction.h>


class HeadScan{
public:
    HeadScan(const ros::NodeHandlePtr nh): nh_(nh), isScanOver_(false), currentId_(-1), 
        scan_server_(*nh_, "head_scan", boost::bind(&HeadScan::onScanRequest, this, _1), false), start_time_(0){
        head_sm_client_ = nh->serviceClient<pr2_head_manager_msgs::StateMachineRegister>("/pr2_head_manager/state_machines_register");
        head_sm_cancel_client_ = nh->serviceClient<resource_management_msgs::StateMachinesCancel>("/pr2_head_manager/state_machine_cancel");
	    fsm_status_sub_ = nh->subscribe("/pr2_head_manager/state_machine_status", 10, &HeadScan::onFSMStatus, this);
        scan_server_.start();
    };

    ~HeadScan(){
        if(currentId_ != -1){
            cancel();
        }
    }

    bool cancel(){
        resource_management_msgs::StateMachinesCancel srv_obj;
        srv_obj.request.id = currentId_;
        return head_sm_cancel_client_.call(srv_obj);
    }

    void onScanRequest(const dt_head_gestures::HeadScanGoalConstPtr &goal){
        dt_head_gestures::HeadScanResult res;
        dt_head_gestures::HeadScanFeedback feedback;
        pr2_head_manager_msgs::StateMachineRegister srv_obj;
        pr2_head_manager_msgs::StateMachineRegisterRequest fsm;
        fsm.header.begin_dead_line = ros::Time::now() + ros::Duration(10.0);
        fsm.header.timeout = ros::Duration(120.0);
        fsm.header.priority.value = resource_management_msgs::MessagePriority::STANDARD;
        double y_min = goal->central_point.point.y - goal->width / 2;
        double y_max = goal->central_point.point.y + goal->width / 2;
        double z_min = goal->central_point.point.z - goal->height / 2;
        double z_max = goal->central_point.point.z + goal->height / 2;
        std::string frame_id = goal->central_point.header.frame_id;
        ros::Time stamp = goal->central_point.header.stamp;

        int y_way = 1;
        int yi = 0;
        int zi = 0;
        char buf[100];
        sprintf(buf, state_name_base, yi, zi);
        fsm.header.initial_state = buf;
        pr2_head_manager_msgs::StateMachineStatePoint* previousState = nullptr;
        while (z_min + std::max(zi - 1, 0) * goal->step_length < z_max){
            while(y_min + std::max(yi - 1, 0) * goal->step_length < y_max && y_min + yi * goal->step_length - y_min >= -0.000001){
                pr2_head_manager_msgs::StateMachineStatePoint state;
                sprintf(buf, state_name_base, yi, zi);
                state.header.id = buf;
                state.data.header.frame_id = frame_id;
                state.data.header.stamp = stamp;
                state.data.point.x = goal->central_point.point.x;
                state.data.point.y = y_min + yi * goal->step_length;
                state.data.point.z = z_min + zi * goal->step_length;
                if (previousState != nullptr){
                    resource_management_msgs::StateMachineTransition t;
                    t.next_state = buf;
                    t.end_condition.duration = goal->duration_per_point.data;
                    t.end_condition.timeout = ros::Duration(-1);
                    previousState->header.transitions.push_back(t);
                }
                fsm.state_machine.states_Point.push_back(state);
                previousState = &(fsm.state_machine.states_Point.back());
                yi += y_way;
            }
            zi++;
            yi -= y_way;
            y_way *= -1;
        }
        if (previousState != nullptr){
            resource_management_msgs::StateMachineTransition t;
            t.next_state = "end";
            t.end_condition.duration = goal->duration_per_point.data;
            t.end_condition.timeout = ros::Duration(-1);
            previousState->header.transitions.push_back(t);
        }else{
            scan_server_.setAborted();
            ROS_ERROR("No state machine created for scan, is step_length too large?");
            return;
        }
        srv_obj.request = fsm;
        bool registerSuccess = false;
        currentId_ = -1;
        start_time_ = ros::Time(0);
        registerSuccess = head_sm_client_.call(srv_obj);
        if (!registerSuccess){
            //res.success = false;
        }
        isScanOver_ = false;
        currentId_ = srv_obj.response.id;
        ros::Duration sleepTime = ros::Duration(0, 1000000);
        while ((!isScanOver_) && ros::ok()){
            if (scan_server_.isPreemptRequested()){
                cancel();
                res.success = false;
                res.action_end = ros::Time::now();
                scan_server_.setPreempted(res);
                return;
            }
            if (start_time_ != ros::Time(0)){
                feedback.action_start = start_time_;
                scan_server_.publishFeedback(feedback);
            }
            sleepTime.sleep();
        }
        res.success = true;
        res.action_end = ros::Time::now();
        scan_server_.setSucceeded(res);
    }

    void onFSMStatus(const resource_management_msgs::StateMachinesStatus& msg){
        if (currentId_ == -1 || msg.id != currentId_){
	        return;
        }
        if (start_time_ == ros::Time(0)){
            start_time_ = ros::Time::now();
        }
        if (msg.state_name == ""){
            isScanOver_ = true;
            currentId_ = -1;
            start_time_ = ros::Time(0);
        }
    }

private:
    ros::NodeHandlePtr nh_;
    ros::ServiceClient head_sm_client_;
    ros::ServiceClient head_sm_cancel_client_;
    actionlib::SimpleActionServer<dt_head_gestures::HeadScanAction> scan_server_;
    ros::Subscriber fsm_status_sub_;
    int currentId_;

    bool isScanOver_;
    ros::Time start_time_;

    static constexpr const char* state_name_base = "scan_state_y%d_z%d";

};

int main(int argc, char** argv){
    ros::init(argc, argv, "dt_head_gestures");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    HeadScan hs(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::cout << "dt_head_geatures is ready" << std::endl;

    ros::waitForShutdown();
    
    


    return 0;
}
