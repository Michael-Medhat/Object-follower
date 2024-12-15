#include <ros/ros.h>
#include <string>
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include <chrono>
#include <thread>


void delay(int x){
    std::this_thread::sleep_for(std::chrono::milliseconds(x));
}

std::string motor_topics[] = {"/vel_cmd/back_left_wheel", "/vel_cmd/back_right_wheel"
                      ,"/vel_cmd/front_left_wheel", "/vel_cmd/front_right_wheel"};

double distance; 

void forward(ros::Publisher* motor_controllers, float carSpeed){
    std_msgs::Float32 msg;
    msg.data = -1 *carSpeed;
    for(int i = 0; i < 4; i++){
        motor_controllers[i].publish(msg);
    }  
    ROS_INFO("Forward");
}

void back(ros::Publisher* motor_controllers, float carSpeed){
    std_msgs::Float32 msg;
    msg.data = 1 *carSpeed;
    for(int i = 0; i < 4; i++){
        motor_controllers[i].publish(msg);
    }  
    ROS_INFO("Back");
}

void stop(ros::Publisher* motor_controllers){
    std_msgs::Float32 msg;
    msg.data = 0;
    for(int i = 0; i < 4; i++){
        motor_controllers[i].publish(msg);
    }  
    ROS_INFO("Stop");
}

void right(ros::Publisher* motor_controllers, float carSpeed){
    std_msgs::Float32 msg_forward;
    std_msgs::Float32 msg_backward;
    msg_forward.data = -1 *carSpeed;
    msg_backward.data =  carSpeed;
    motor_controllers[0].publish(msg_forward); //front left
    motor_controllers[1].publish(msg_backward); //front right
    motor_controllers[2].publish(msg_backward);// back left
    motor_controllers[3].publish(msg_forward);// back right

    ROS_INFO("left");
} 

void left(ros::Publisher* motor_controllers, float carSpeed){
    std_msgs::Float32 msg_forward;
    std_msgs::Float32 msg_backward;
    msg_forward.data = -1 *carSpeed;
    msg_backward.data = carSpeed;
    motor_controllers[0].publish(msg_backward); //front left
    motor_controllers[1].publish(msg_forward); //front right
    motor_controllers[2].publish(msg_forward);// back left
    motor_controllers[3].publish(msg_backward);// back right

    ROS_INFO("right");
}

void loop(ros::Publisher* motor_controllers, ros::Publisher& servo, float carSpeed) { 
    std_msgs::Float32 degree;
    degree.data = 10;
    servo.publish(degree);
    delay(200);
    ros::spinOnce();
    double rightDistance = distance;

    degree.data = -10;
    servo.publish(degree);
    delay(200);
    ros::spinOnce();
    double leftDistance = distance;

    std::cout<<"Right motor_controllersDistance: "<<rightDistance<<" Left Distance: "<<leftDistance<<std::endl;
    if((rightDistance > 30)&&(leftDistance > 30)){
      stop(motor_controllers);
    }else if(rightDistance >= 5 && leftDistance >= 5 && rightDistance < 30 && leftDistance < 30) {     
      forward(motor_controllers, carSpeed);
    }else if((rightDistance <= 3) && (leftDistance <= 3)) {
        back(motor_controllers, carSpeed);
        delay(100);
    }else if(rightDistance - 1 > leftDistance) {
        left(motor_controllers, carSpeed / 4);
        delay(250);
    }else if(rightDistance + 1 < leftDistance) {
        right(motor_controllers, carSpeed / 4);
        delay(250);
    }else{
      stop(motor_controllers);
    }
}                 

void distanceCallback(const sensor_msgs::Range::ConstPtr& msg){
    distance = msg->range;
}


int main(int argc, char ** argv){
    if(argc < 2){
        ROS_INFO("Usage nodename carspeed");
        return 1;
    }

    ros::init(argc, argv, "controller");
    
    ros::NodeHandle node;

    ros::Publisher motor_controllers[4];

    ros::Publisher servo;

    float carSpeed = atof(argv[1]);

    for(int i = 0; i < 4; i++){
        motor_controllers[i] = node.advertise<std_msgs::Float32>(motor_topics[i], 100);
    }

    servo = node.advertise<std_msgs::Float32>("/vel_cmd/servo", 100);

    ros::Subscriber distance = node.subscribe("/sensor/Range", 1000, distanceCallback);
    
    ros::Rate loop_rate(60);
    
    while(ros::ok()){

        loop(motor_controllers,servo, carSpeed);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
