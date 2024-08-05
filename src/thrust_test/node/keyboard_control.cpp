/*include ros.h*/
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "getch.h"
#include "string.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/Thrust.h"
#include "cmath"
#include "Eigen/Dense"

float dt = 0.01;
int t;
float theta = 0;
float theta_2 = 0;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped R;
std_msgs::Float32MultiArray rd;
Eigen::Matrix<float, 3, 3> Rr;

mavros_msgs::AttitudeTarget T;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Attitude(int t){
    theta = 0.5*sin(t*dt);
    theta_2 = 0.5*cos(t*dt);
    float yaw = 0.0;
    float roll = theta;
    float pitch = theta_2;

    Eigen::Quaternionf quaternion;
    
    quaternion = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
               * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
               * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    T.orientation.w =quaternion.w();
    T.orientation.x =quaternion.x();
    T.orientation.y =quaternion.y();
    T.orientation.z =quaternion.z();
    Rr = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
               * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
               * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    
}


int main(int argv,char** argc)
{

    ros::init(argv,argc,"keyboard");
    ros::NodeHandle nh;
    int c_prev = EOF;
    int UAV_ID;
    t = 0;

/***************************************************************/
/*                     Initialize                              */
/***************************************************************/

T.orientation.w = 1;
T.orientation.x = 0;
T.orientation.y = 0;
T.orientation.z = 0;
T.type_mask = T.IGNORE_PITCH_RATE | \
T.IGNORE_ROLL_RATE |T.IGNORE_YAW_RATE ;


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

    ros::Publisher T_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("mavros/setpoint_raw/attitude", 10);


    ros::Rate rate(100.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ros::param::get("UAV_ID", UAV_ID);

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
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }


    while(ros::ok())
    {
        int c = getch();


        if(c != EOF){
            switch (c)
            {
            case 'A':
                {
                    T.thrust+=0.1;
                }
                break;
            case 'Z':
                {
                    T.thrust-=0.1;
                }
                break;
            case 'E':
                {
                    ROS_WARN("kill!");
                    offb_set_mode.request.custom_mode = "AUTO.LAND";
                    set_mode_client.call(offb_set_mode);
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                }
                break;

            }
            }
        ROS_INFO("Current_Thrust:%f",T.thrust);


        Attitude(t);
        T_pub.publish(T);
        t++;
        ros::spinOnce();
        rate.sleep();
        }


    return 0;
}
