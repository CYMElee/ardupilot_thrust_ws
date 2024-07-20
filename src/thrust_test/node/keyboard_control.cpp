/*include ros.h*/
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "getch.h"
#include "string.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/Thrust.h"


mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped R;
mavros_msgs::Thrust T;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argv,char** argc)
{

    ros::init(argv,argc,"keyboard");
    ros::NodeHandle nh;
    int c_prev = EOF;
    int UAV_ID;

/***************************************************************/
/*                     Initialize                              */
/***************************************************************/

T.thrust = 0.4;
R.pose.orientation.w = 1;
R.pose.orientation.x = 0;
R.pose.orientation.y = 0;
R.pose.orientation.z = 0;
/***************************************************************/
/*                     Takeoff                                */
/***************************************************************/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

/***************************************************************/
/*                       Accelerator                           */
/***************************************************************/

    ros::Publisher T_pub = nh.advertise<mavros_msgs::Thrust>
        ("mavros/setpoint_attitude/thrust", 10);
    ros::Publisher R_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_attitude/attitude", 10);

    ros::Rate rate(100.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ros::param::get("UAV_ID", UAV_ID);
   // UAV_ID = 1;
    ROS_INFO("Wait for setting origin and home position...");
    std::string mavlink_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/mavlink/to");
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic);
    ROS_INFO("Message received or timeout reached. Continuing execution.");
 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    //send a few setpoints before starting

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 0.5;
    if (takeoff_cl.call(srv_takeoff)) {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
    }

    sleep(10);

    while(ros::ok())
    {
        int c = getch();


        if(c != EOF){
            switch (c)
            {
            case 'A':
                {
                    T.thrust+=0.5;
                }
                break;
            case 'Z':
                {
                    T.thrust-=0.5;
                }
                break;
            case 'E':
                {
                    ROS_WARN("kill!");
                    offb_set_mode.request.custom_mode = "LAND";
                    set_mode_client.call(offb_set_mode);
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                }
                break;

            }
            }
        ROS_INFO("Current_Thrust:%f",T.thrust);
        T_pub.publish(T);
        R_pub.publish(R);
        ros::spinOnce();
        rate.sleep();
        }


    return 0;
}
