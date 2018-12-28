#include "mbed.h"
#include "Motor.h"
#include "QEI.h"
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

Motor leftMotor(PB_6, PC_5, PC_8); //pwm, inA, inB 
Motor rightMotor(PA_15, PA_13, PA_14);   //pwm, inB, inA

QEI leftQei(PB_4, PB_5, NC, 600);  //chanB, chanA, index, ppr 
QEI rightQei(PA_8, PA_9,NC, 600); //chanA, chanB, index, ppr

DigitalIn emer(PB_2);
DigitalOut led1(LED1);

const char errormsg[20] = "DEBUG...";
Timer t;
Timer proc[4];

float Motorprd = 0.009;  // pwm period (sec)  0.01
float control_rate = 50; // Control loop freq (Hz.)
float encode_rate = 52;  // Encoder reading, odom calculation freq (Hz.)
float odom_rate = 17;    // Odom,TF publish freq (Hz.)
float tf_rate = 17;

float pi = 3.14159;      // Adjust pi = 3.14159
float wheel = 0.2032;    // wheel diameter (meter)   3.14*0.2032 = 0.638
float wheel_dist = 0.62; // wheel separation distance
float minspeed_p = 376;  // (pulse/sec)Used in getSpeed
float maxspeed_p = 940;  // (pulse/sec)Used in getSpeed
float maxLinear = 0.5;   // m/s
float minLinear = 0.0;   // m/s
float maxAngular = 0.8;  // rad/s


float Vl = 0;
float Vr = 0;
float Vl_p = 0;
float Vr_p = 0;
float leftVelocity = 0.00; //absolute valu
float rightVelocity = 0.00; //absolute valu

//PID variable////////////////////////////////////////////
    //Left motor
float lKp = 0.29 ;     // left Kp
float lKi = 0.03  ;       // left Ki
float lKd = 0.05 ;       // left Kd
    //Right motor
float rKp = 0.29;       // right Kp
float rKi = 0.03;       // right Ki
float rKd = 0.05;       // right Kdss

float L = 0;   //PID compute
float R = 0;   //PID compute
//Odometry variable///////////////////////////////////////
char base_link_id[] = "/base_link";
char odom_id[] = "/odom";
float odom_pose[3];
double odom_vel[3];
double leftwheelspeed = 0;
double rightwheelspeed = 0;
bool isconnected = false;

void readSpeed();
void pidControl();
void getSpeed(const geometry_msgs::Twist &msg);
void reset_base(const std_msgs::Empty &reset_msg);
void get_imu(const sensor_msgs::Imu &imu);
void init_odom();
void publish_odom();
void publish_tf();
void motorGo();

nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped tfs;
ros::Time current_rostime, last_rostime;
geometry_msgs::Quaternion imuo;

ros::NodeHandle nh;
tf::TransformBroadcaster tf_broadcaster;
ros::Publisher odom_publisher("odom", &odom_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &getSpeed);
ros::Subscriber<std_msgs::Empty> reset_sub("reset_base", &reset_base);
ros::Subscriber<sensor_msgs::Imu> imu_sub("imu", &get_imu);

void readSpeed()
{ 
    float rate = proc[1].read();
    static float srightVelocity = 0.00;
    static float sleftVelocity = 0.00;
    int leftPulses = 0;     
    int leftPrevPulses = 0; 
    int rightPulses = 0;     
    int rightPrevPulses = 0; 
    static double prev_yaw = 0.0;
    double yaw = 0.0;

    leftPulses = leftQei.getPulses();
    sleftVelocity = (leftPulses - leftPrevPulses) / rate;
    leftVelocity = sleftVelocity;
    leftPrevPulses = leftPulses;

    rightPulses = rightQei.getPulses();
    srightVelocity = (rightPulses - rightPrevPulses) / rate;
    rightVelocity = srightVelocity;
    rightPrevPulses = rightPulses;
    
    leftwheelspeed = (sleftVelocity * pi * wheel) /  1200;      //DEBUG
    rightwheelspeed = (srightVelocity * pi * wheel) /  1200;    //DEBUG
    
    odom_vel[0] = ((srightVelocity + sleftVelocity) * pi * wheel) / (2 * 1200);
    //odom_vel[2] = ((srightVelocity - sleftVelocity) * pi * wheel) / (1200*wheel_dist); //yaw from wheel
    
    yaw = atan2(2.0*(imuo.x*imuo.y + imuo.w*imuo.z), imuo.w*imuo.w + imuo.x*imuo.x - imuo.y*imuo.y - imuo.z*imuo.z);
    float delta_th = yaw - prev_yaw;
    prev_yaw = yaw; 
    odom_vel[2] = delta_th / rate;
    
    float delta_x = (odom_vel[0] * cos(odom_pose[2])) * rate;
    float delta_y = (odom_vel[0] * sin(odom_pose[2])) * rate;
    
    odom_pose[0] += delta_x;
    odom_pose[1] += delta_y;
    odom_pose[2] += delta_th;
    leftQei.reset();
    rightQei.reset();
    
    nh.spinOnce();
    
} /// readSpeed

void pidControl()
{
    static float L_err = 0;     //left error
    static float L_preverr = 0; //previous left error
    static float L_sumerr = 0;  //left sum error
    static float L_u = 0;       //PID control u left wheel
    static float R_err = 0;     //right error
    static float R_preverr = 0; //previous right error
    static float R_sumerr = 0;  //right sum error
    static float R_u = 0;       //PID control u right wheel

    if(Vl_p==0){
        L_sumerr = 0;    
    }
    if(Vr_p==0){
        R_sumerr = 0;    
    }

    L_preverr = L_err;
    L_err = Vl_p - leftVelocity;
    L_sumerr += L_err;

    L_u = (lKp * L_err) + (lKd * (L_err - L_preverr)) + (lKi*(L_sumerr))  ;
    L = L_u / 3600;
    L += L;
            

    R_preverr = R_err;
    R_err = Vr_p - rightVelocity;
    R_sumerr += R_err;

    R_u = (rKp * R_err) + (rKd * (R_err - R_preverr)) + (rKi*(R_sumerr)) ;
    R = R_u / 3600;
    R += R;
} /// pidControl

void getSpeed(const geometry_msgs::Twist &msg)
{
    float V_linear = msg.linear.x; // m/s
    float omega = msg.angular.z;   // rad/s
    
    if (V_linear > maxLinear){
         V_linear = maxLinear;
    }
    else if (V_linear < -maxLinear){
         V_linear = -maxLinear;
    }
    
    if (omega > maxAngular){
        omega = maxAngular;    
    }
    else if (omega < -maxAngular){
        omega = -maxAngular;    
    }

    Vl = V_linear - ((omega * wheel_dist) / 2); // m/s
    Vr = V_linear + ((omega * wheel_dist) / 2); 

    Vl_p = (Vl * 1200) / (pi * wheel); // pulse/s
    Vr_p = (Vr * 1200) / (pi * wheel); 

} /// getSpeed

void reset_base(const std_msgs::Empty &reset_msg)
{
    for (int i = 0; i < 3; i++)
    {
        odom_pose[i] = 0.0;
        odom_vel[i]  = 0.0;
    }

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 0.0;

    odom_msg.twist.twist.linear.x  = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    tfs.transform.translation.x = 0.0;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;

    tfs.transform.rotation.x = 0.0;
    tfs.transform.rotation.y = 0.0;
    tfs.transform.rotation.z = 0.0;
    tfs.transform.rotation.w = 0.0;

} /// reset_base

void get_imu(const sensor_msgs::Imu &imu)
{
    imuo = imu.orientation;
} /// get_imu

void init_odom(void)
{
    for (int i = 0; i < 3; i++)
    {
        odom_pose[i] = 0.0;
        odom_vel[i]  = 0.0;
    }

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 0.0;

    odom_msg.twist.twist.linear.x  = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    tfs.transform.translation.x = 0.0;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;

    tfs.transform.rotation.x = 0.0;
    tfs.transform.rotation.y = 0.0;
    tfs.transform.rotation.z = 0.0;
    tfs.transform.rotation.w = 0.0;
}

void publish_odom()
{
    odom_msg.header.stamp = current_rostime;
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = base_link_id;
    odom_msg.pose.pose.position.x = odom_pose[0];
    odom_msg.pose.pose.position.y = odom_pose[1];
    odom_msg.pose.pose.position.z = 0.0;
        
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
        
    odom_msg.twist.twist.linear.x = odom_vel[0];
    //odom_msg.twist.twist.linear.y = odom_vel[1];    // DEBUG
    //odom_msg.twist.twist.linear.z = rightwheelspeed;    // DEBUG
    odom_msg.twist.twist.angular.z = odom_vel[2];
        
    odom_publisher.publish(&odom_msg);
    
}
void publish_tf()
{
    //publish TF
    tfs.header.stamp = current_rostime;
    tfs.header.frame_id = odom_id;
    tfs.child_frame_id = base_link_id;
    tfs.transform.translation.x = odom_pose[0];
    tfs.transform.translation.y = odom_pose[1];
    tfs.transform.translation.z = 0.0;

    tfs.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster.sendTransform(tfs);

} /// publish_tf

void motorGo(){
    if (abs(Vl_p) <= 0.001)
        {
            leftMotor.brake(0);
        }
    else
        {
            leftMotor.speed(L);
        }
        
    if (abs(Vr_p) <= 0.001)
        {
            rightMotor.brake(0);
        }
    else
        {
            rightMotor.speed(R);
        }
}

void main(int argc, char **argv){
    //
    
    nh.getHardware()->setBaud(250000);
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(reset_sub);
    nh.subscribe(imu_sub);
    nh.advertise(odom_publisher);
    tf_broadcaster.init(nh);
    current_rostime = nh.now();
    last_rostime = nh.now();
    leftMotor.period(Motorprd);
    rightMotor.period(Motorprd);
    leftMotor.brake(0);
    rightMotor.brake(0);
    emer.mode(PullUp);
    
    while(!nh.connected() && !isconnected){
        nh.spinOnce(); 
        wait_ms(1);   
    }
    isconnected = true;
    
    init_odom();
    leftQei.reset();
    rightQei.reset();
    
    //Start process timers
    proc[0].start();
    proc[1].start();
    proc[2].start();
    proc[3].start();
  
    while(1){
       
         //set to true in getSpeed() callback
        
        current_rostime = nh.now();

        if(proc[1].read_ms() >= (1000/encode_rate)) {
            readSpeed();
            proc[1].reset();
        }
        if(proc[2].read_ms() >= (1000/control_rate) && emer) {
            pidControl();
            proc[2].reset();
        }
        
        if(proc[3].read_ms() >= (1000/tf_rate)) {
            publish_tf();;
            proc[3].reset();
        }
        else if(proc[0].read_ms() >= (1000/odom_rate)) {
            publish_odom();
            proc[0].reset(); 
        }
        
        motorGo();
    
        last_rostime = current_rostime;
        nh.spinOnce();
        wait_ms(1);    
    } // main while loop
    
}