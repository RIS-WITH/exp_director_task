#include <ros/ros.h>
#include <pr2_head_manager_msgs/StateMachineRegister.h>
#include <dt_head_gestures/HeadScan.h>


class HeadScan{
public:
    HeadScan(const ros::NodeHandlePtr nh): nh_(nh){
        head_sm_client_ = nh->serviceClient<pr2_head_manager_msgs::StateMachineRegister>("/pr2_head_manager/state_machines_register");
        scan_server_  = nh_->advertiseService("head_scan", &HeadScan::onScanRequest, this);
    };

    bool onScanRequest(dt_head_gestures::HeadScanRequest& req, dt_head_gestures::HeadScanResponse& res){
        pr2_head_manager_msgs::StateMachineRegister srv_obj;
        pr2_head_manager_msgs::StateMachineRegisterRequest fsm;
        fsm.header.begin_dead_line = ros::Time::now() + ros::Duration(10.0);
        fsm.header.timeout = ros::Duration(120.0);
        fsm.header.priority.value = resource_management_msgs::MessagePriority::STANDARD;
        double y_min = req.central_point.point.y - req.width / 2;
        double y_max = req.central_point.point.y + req.width / 2;
        double z_min = req.central_point.point.z - req.height / 2;
        double z_max = req.central_point.point.z + req.height / 2;
        std::string frame_id = req.central_point.header.frame_id;
        ros::Time stamp = req.central_point.header.stamp;

        int y_way = 1;
        int yi = 0;
        int zi = 0;
        char buf[100];
        sprintf(buf, state_name_base, yi, zi);
        fsm.header.initial_state = buf;
        pr2_head_manager_msgs::StateMachineStatePoint* previousState = nullptr;
        while (z_min + std::max(zi - 1, 0) * req.step_length < z_max){
            while(y_min + std::max(yi - 1, 0) * req.step_length < y_max && y_min + yi * req.step_length - y_min >= -0.000001){
                pr2_head_manager_msgs::StateMachineStatePoint state;
                sprintf(buf, state_name_base, yi, zi);
                state.header.id = buf;
                state.data.header.frame_id = frame_id;
                state.data.header.stamp = stamp;
                state.data.point.x = req.central_point.point.x;
                state.data.point.y = y_min + yi * req.step_length;
                state.data.point.z = z_min + zi * req.step_length;
                if (previousState != nullptr){
                    resource_management_msgs::StateMachineTransition t;
                    t.next_state = buf;
                    t.end_condition.duration = req.duration_per_point.data;
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
            t.end_condition.duration = req.duration_per_point.data;
            t.end_condition.timeout = ros::Duration(-1);
            previousState->header.transitions.push_back(t);
        }else{
            ROS_ERROR("No state machine created for scan, is step_length too large?");
        }
        srv_obj.request = fsm;
        res.success = head_sm_client_.call(srv_obj);
        return res.success;
    }

private:
    ros::NodeHandlePtr nh_;
    ros::ServiceClient head_sm_client_;
    ros::ServiceServer scan_server_;

    static constexpr const char* state_name_base = "scan_state_y%d_z%d";

};

int main(int argc, char** argv){
    ros::init(argc, argv, "dt_head_gestures");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    HeadScan hs(nh);

    ros::spin();
    


    return 0;
}