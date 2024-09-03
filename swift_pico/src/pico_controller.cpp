/*This cpp file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			    /throttle_pid
		            			/pitch_pid
		            			/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.*/

// importing the required libraries
#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_array.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "pid_msg/msg/pid_tune.hpp"
#include "pid_msg/msg/pid_error.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct CMD
{
    int rc_roll;
    int rc_pitch;
    int rc_yaw;
    int rc_throttle;
    int rc_aux4;
};

struct ERROR
{
    float roll_error;
    float pitch_error;
    float throttle_error;
    float yaw_error;

};


class Swift_Pico : public rclcpp::Node
{
    public:
        Swift_Pico() : Node("pico_controller") //initializing ros node with name pico_controller
        {
            /* This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		     [x,y,z]*/
            drone_position[0] = 0.0;
            drone_position[1] = 0.0;
            drone_position[2] = 0.0;

            /* [x_setpoint, y_setpoint, z_setpoint]
            whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly*/
            setpoint[0] = 2;
            setpoint[1] = 2;
            setpoint[2] = 20;

            //Declaring a cmd of message type swift_msgs and initializing values
            cmd.rc_roll = 1500;
            cmd.rc_pitch = 1500;
            cmd.rc_yaw = 1500;
            cmd.rc_throttle = 1500;

            /*initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		    after tuning and computing corresponding PID parameters, change the parameters*/

            //proportional
            Kp[0] = 0;
            Kp[1] = 0;
            Kp[2] = 0;

            //integral
            Ki[0] = 0;
            Ki[1] = 0;
            Ki[2] = 0;

            //derivative
            Kd[0] = 0;
            Kd[1] = 0;
            Kd[2] = 0;
            /*-----------------------Add other required variables for pid here ----------------------------------------------*/









            /* Hint : Add variables for storing previous errors in each axis, like prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]. 
            Add variables for limiting the values like max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
            										   min_values = [1000,1000,1000] corresponding to [roll, pitch, throttle]
            Add variables for publishing error.
            You can change the upper limit and lower limit accordingly. 
            ---------------------------------------------------------------------------------------------------------*/

            /*This is the sample time in which you need to run pid. Choose any time which you seem fit.*/
        
            sample_time = 60ms; //in milli-seconds

            //Publishing /drone_command, /throttle_error, /pitch_error, /roll_error
            command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
            pid_error_pub = this->create_publisher<pid_msg::msg::PIDError>("/pid_error", 10);


            //Add othher ROS 2 Publishers here

            //Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
            whycon_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/whycon/poses", 1, std::bind(&Swift_Pico::whycon_callback, this, _1));
            throttle_pid_sub = this->create_subscription<pid_msg::msg::PIDTune>("/throttle_pid", 1, std::bind(&Swift_Pico::altitude_set_pid, this, _1));

            //------------------------Add other ROS 2 Subscribers here-----------------------------------------------------


            //Arming the drone
            arm();

            //Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(C++)




        }

    private:

        //declare all the variables, arrays, strcuts etc. here
        float drone_position[3];
        int setpoint[3];
        CMD cmd;
        std::chrono::milliseconds sample_time;
        float Kp[3];
        float Ki[3];
        float Kd[3];

        //declare the publishers and subscribers here
        rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub;
        rclcpp::Publisher<pid_msg::msg::PIDError>::SharedPtr pid_error_pub;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr whycon_sub;
        rclcpp::Subscription<pid_msg::msg::PIDTune>::SharedPtr throttle_pid_sub;

        //define functions and callbacks here

        void disarm()
        {
            auto cmd = swift_msgs::msg::SwiftMsgs();
            cmd.rc_roll = 1000;
            cmd.rc_pitch = 1000;
            cmd.rc_yaw = 1000;
            cmd.rc_throttle = 1000;
            cmd.rc_aux4 = 1000;
            command_pub->publish(cmd);
        }

        void arm()
        {   
            auto cmd = swift_msgs::msg::SwiftMsgs();
            disarm();
            cmd.rc_roll = 1500;
            cmd.rc_pitch = 1500;
            cmd.rc_yaw = 1500;
            cmd.rc_throttle = 1500;
            cmd.rc_aux4 = 2000;
            command_pub->publish(cmd);

        }

        void whycon_callback(const geometry_msgs::msg::PoseArray & msg)
        {
            drone_position[0] = msg.poses[0].position.x;
            /*--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------



	
		    ------------------------------------------------------------------------------------------------------------------------*/

        }

        //Callback function for /throttle_pid
	    //This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	
        void altitude_set_pid(const pid_msg::msg::PIDTune & alt)
        {
            Kp[2] = alt.kp * 0.03;  // This is just for an example. You can change the ratio/fraction value accordingly
		    Ki[2] = alt.ki * 0.008;
		    Kd[2] = alt.kd * 0.6;
        }


        //----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

        //----------------------------------------------------------------------------------------------------------------------


        void pid()
        {
            auto cmd = swift_msgs::msg::SwiftMsgs();
            auto error_pub = pid_msg::msg::PIDError();
            /*-----------------------------Write the PID algorithm here--------------------------------------------------------------

            # Steps:
            # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
            #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
            #	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
            #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
            #	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
            #	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
            #																														self.cmd.rcPitch = self.max_values[1]
            #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
            #	8. Add error_sum


















            #------------------------------------------------------------------------------------------------------------------------*/
            command_pub->publish(cmd);
            //calculate throttle error, pitch error and roll error, then publish it accordingly
            pid_error_pub->publish(error_pub);

        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Swift_Pico>());
    rclcpp::shutdown();
    return 0;
}