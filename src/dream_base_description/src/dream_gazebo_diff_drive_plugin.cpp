/**
 * This plugin is partially inspired by gazebo_ros_diff_drive: 
 * https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp
 * What this plugin does:
 *  - Set joint vel instataneously
    - perfect odom: base_link pose, vel;  for testing
    - publishWheelJointState: perfect encoder, 
 * Gazebo Notes: 
    - a plugin is a shared lib and inserted into simulation. Has direct access to all functionalities in Gazebo
        - Overview: https://classic.gazebosim.org/tutorials?tut=plugins_hello_world
    - rate for onUpdate is set in world, which is usually 1000hz
        - That depends on the real time factor, too.    
    - A ROS subscriber is added onto a separate ROS callback queue, which allows it to execute on a separate thread.
        - http://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i6
    - To register on_update, pass the on_update method to the Gazebo Simulator's event class instance as a callback:
        event::Events::ConnectWorldUpdateBegin(...)
 * Cpp notes:
    - Anonymous enum
        - enum class itself does not support anonymity
        - anonymous enum is used only in the current translational unit. They are basically glbal constants in the 
            current namespace
    - static double prev_update_rate = 0.0; 
        - static here gives the variable static storage duration. 
        - So, it is created when the program starts, and destroyed when the program ends.
        - static also gives internal linkage, meaning, the variable can be only accessed within the same translation unit.
 */
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
    enum{
        LEFT,
        RIGHT,
    };

    template<typename T>
    void find_element(const std::string& element_name, T& element, sdf::ElementPtr _sdf) {
        // Read from gazebo.xacro
        if (_sdf->HasElement(element_name)){
            element = _sdf->Get<T>(element_name);
            ROS_INFO_STREAM("Got " << element_name << ": " << element);
        }
        else{
            ROS_ERROR_STREAM("NO PARAM " << element_name);
        }
    };

    //Note: Each plugin must inherit a class
    class DreamGazeboDiffDrivePlugin: public ModelPlugin{
        private: ros::Subscriber wheel_vel_cmd_sub_;
        private: std::unique_ptr<ros::NodeHandle> nh_;
        private: ros::Publisher odom_pub_;
        private: ros::Publisher wheel_joint_states_pub_;
        // A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue node_callback_queue_;
        private: std::thread node_callback_queue_thread_;
        private: std::vector<physics::JointPtr> wheel_joints_;
        private: std::array<double, 2> wheel_vel_cmd_ = {0.0, 0.0};
        private: event::ConnectionPtr update_connection_;
        private: physics::ModelPtr model;
        private: double WHEEL_RADIUS_;

        public: DreamGazeboDiffDrivePlugin(){
        }
        public: ~DreamGazeboDiffDrivePlugin(){}
        // Receives SDF
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
            const char* node_name = "dream_gazebo_client"; 
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
            }
            nh_.reset(new ros::NodeHandle(node_name));

            /********************** Get the joints params **********************/
            ROS_INFO("Loading wheel params for DreamGazeboDiffDrivePlugin");
            std::string left_wheel_joint, right_wheel_joint;
            find_element<std::string>("left_wheel_joint", left_wheel_joint, _sdf);
            find_element<std::string>("right_wheel_joint", right_wheel_joint, _sdf);
            find_element<double>("wheel_radius", WHEEL_RADIUS_, _sdf);
            wheel_joints_.resize(2);
            this->model = _parent;
            wheel_joints_[LEFT] = _parent->GetJoint(left_wheel_joint);
            wheel_joints_[RIGHT] = _parent->GetJoint(right_wheel_joint);
            wheel_joints_[LEFT]->SetParam("fmax", 0, 10000.0);
            wheel_joints_[RIGHT]->SetParam("fmax", 0, 10000.0);

            ROS_INFO("Initializing connection for DreamGazeboDiffDrivePlugin");
            /********************** Initialize Callbacks **********************/
            // Listen to the update event. This event is broadcast every simulation iteration.
            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DreamGazeboDiffDrivePlugin::OnUpdate, this));
            std::string wheel_vel_topic;
            find_element<std::string>("commanded_wheel_vel_topic", wheel_vel_topic, _sdf);
            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                wheel_vel_topic,
                1,
                boost::bind(&DreamGazeboDiffDrivePlugin::wheel_vel_cmd_sub_cb, this, _1), ros::VoidPtr(), &this->node_callback_queue_);
            wheel_vel_cmd_sub_ = nh_->subscribe(so);

            // Spin up the queue helper thread.
            this->node_callback_queue_thread_ = std::thread(std::bind(&DreamGazeboDiffDrivePlugin::node_callback_queue_thread_func, this));

            // Spin up publishers for odom and wheel joint states
            std::string odom_topic, wheel_joint_states_topic;
            find_element<std::string>("odom_topic", odom_topic, _sdf);
            find_element("wheel_joint_states_topic", wheel_joint_states_topic, _sdf);
            odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 1);
            wheel_joint_states_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(wheel_joint_states_topic, 1);

            ROS_INFO("DreamGazeboDiffDrivePlugin loaded.");
        }
        // what's the frequency this gets called?
        public: void OnUpdate(){
            // model must be updated in this func
            // setparam must be called here, otherwise, there will be a crash
            // setting joint velocities in rad/s
            wheel_joints_[LEFT]->SetParam("vel", 0, wheel_vel_cmd_[LEFT]);
            wheel_joints_[RIGHT]->SetParam("vel", 0, wheel_vel_cmd_[RIGHT]);
        }

        private: void wheel_vel_cmd_sub_cb(const std_msgs::Float32MultiArrayConstPtr& msg){
            // Here we are following a convention: [left vel, -right vel]
            // Notice the -right vel
            ROS_INFO_STREAM("Received wheel linear velocity: " << msg->data[0] << " " << msg->data[1]);
            wheel_vel_cmd_[LEFT] = msg->data[0] / WHEEL_RADIUS_;
            wheel_vel_cmd_[RIGHT] = -msg->data[1] / WHEEL_RADIUS_;
            ROS_INFO_STREAM("Sending wheel linear velocity: " << wheel_vel_cmd_[LEFT] << " " << wheel_vel_cmd_[RIGHT]);
            ROS_INFO_STREAM("WHEEL RADIUS: " << WHEEL_RADIUS_);
        }
        
        /**
         * @brief function that executes callbacks from the node callback queue on a separate thread. 
         * This helps so that gazebo client can work on a separate thread.
         */
        private: void node_callback_queue_thread_func(){
            constexpr static const double timeout = 0.01;
            while (nh_->ok())
            {
                this->node_callback_queue_.callAvailable(ros::WallDuration(timeout));
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(DreamGazeboDiffDrivePlugin);
}