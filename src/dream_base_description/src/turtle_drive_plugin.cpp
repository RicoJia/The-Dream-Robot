/**
 * @brief This is a gazebo plugin that models a turtlebot3 burger
    SUBSCRIBES:
    wheel_cmd (nuturtlebot/WheelCommands) - commanded wheel veloctities at a specified rate in the gazebo xacro file.
    PUBLISHES:
    sensor_data (nuturtlebot/SensorData) - wheel velocities
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include <cmath>



namespace gazebo
{
    enum {
        LEFT, RIGHT,
    };

    constexpr double PI=3.14159265358;

    /// \brief Wrap an angle to [0, 2*pi]
    inline double wrap_angle(double x){
        double min = -1.0*PI;
        double max = PI;
        return min+ fmod((max - min + ( fmod( (x - min), (max - min) ) ) ), (max - min));
    }

    class turtle_drive_plugin : public ModelPlugin
    {
    private: ros::Subscriber joint_state_sub;
    private: ros::CallbackQueue rosQueue;
    private: std::unique_ptr<ros::NodeHandle> ros_node;
        /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    private: double omega_l;
    private: double omega_r;
    private: event::ConnectionPtr updateConnection;
    private: physics::ModelPtr model;
    private: std::string left_wheel_joint;
    private: std::string right_wheel_joint;
    private: std::string wheel_cmd_topic;
    private: std::string sensor_data_topic;
    private: std::vector<physics::JointPtr> joints;
    private: double max_motor_rot_vel;
    private: double max_wheel_command;
    private: int sensor_frequency;
    private: int encoder_ticks_per_rev;

    private: ros::Publisher sensor_data_pub;
    private:
        gazebo::common::Time updateRate;
        gazebo::common::Time prevUpdateRate;
        gazebo::common::Time prev_msg_time;

    public: turtle_drive_plugin() : ModelPlugin()
        {}

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // see http://gazebosim.org/tutorials?tut=ros_plugins Accessed 02/09/2020
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL("A ROS node for Gazebo has not been initialized."
                          "Unable to load plugin. Load the Gazebo system plugin"
                          "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
                return;
            }

            ros::NodeHandle nh;
            ros::NodeHandle nh2("~");

            // Read from gazebo.xacro

            if (_sdf->HasElement("left_wheel_joint")){
                 left_wheel_joint = _sdf->Get<std::string>("left_wheel_joint");
            }
            else{
                ROS_INFO("NO PARAM left_wheel_joint!!");
            }

            if (_sdf->HasElement("right_wheel_joint")){
                right_wheel_joint = _sdf->Get<std::string>("right_wheel_joint");
            }
            else{
                ROS_INFO("NO PARAM right_wheel_joint!!");
            }


            if (_sdf->HasElement("sensor_frequency")){
                sensor_frequency = _sdf->Get<int>("sensor_frequency");
            }
            else{
                sensor_frequency = 200;
            }

            if (_sdf->HasElement("wheel_cmd_topic")){
                wheel_cmd_topic = _sdf->Get<std::string>("wheel_cmd_topic");
            }
            else{
                ROS_INFO("NO PARAM wheel_cmd_topic");
            }

            if (_sdf->HasElement("sensor_data_topic")){
                sensor_data_topic = _sdf->Get<std::string>("sensor_data_topic");
            }
            else{
                ROS_INFO("NO PARAM sensor_data_topic");
            }

            if (_sdf->HasElement("max_wheel_command")){
                max_wheel_command = _sdf->Get<double>("max_wheel_command");
            }
            else{
                ROS_INFO("NO PARAM max_wheel_command");
            }

            if (_sdf->HasElement("max_motor_rot_vel")){
                max_motor_rot_vel = _sdf->Get<double>("max_motor_rot_vel");
            }
            else{
                ROS_INFO("NO PARAM max_motor_rot_vel");
            }

            if (_sdf->HasElement("encoder_ticks_per_rev")){
                encoder_ticks_per_rev  = _sdf->Get<int>("encoder_ticks_per_rev");
            }
            else{
                ROS_INFO("NO PARAM encoder_ticks_per_rev ");
            }

            // subscriber function.
            // alternatively, you can do nh.subscribe ...
            this->ros_node.reset(new ros::NodeHandle("gazebo_client"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<nuturtlebot::WheelCommands>(
                    "/wheel_cmd",
                    1,
                    boost::bind(&turtle_drive_plugin::OnRosMsg, this, _1),//TODO: what is this?
                    ros::VoidPtr(), &this->rosQueue);
            this->joint_state_sub = this->ros_node->subscribe(so);

            this->rosQueueThread =
                    std::thread(std::bind(&turtle_drive_plugin::QueueThread, this));

            /// Store the pointer to the model
            this->model = _parent;

            /// Listen to the update event. This event is broadcast every
            /// simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&turtle_drive_plugin::OnUpdate, this));

            /// joint setup
            this->joints.resize(2);
            this->joints[LEFT] = this->model->GetJoint(this->left_wheel_joint);
            this->joints[RIGHT] = this->model->GetJoint(this->right_wheel_joint);

            //publisher setup
            sensor_data_pub = nh.advertise<nuturtlebot::SensorData>(sensor_data_topic, 100);

            //Update rate setup
            this->updateRate = common::Time(0, common::Time::SecToNano(0.01));
            this->prevUpdateRate = common::Time::GetWallTime();
            
            //wheel_position set up
            this->omega_l = 0;
            this->omega_r = 0;
            this->prev_msg_time = common::Time::GetWallTime();
        }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    public: void OnRosMsg(const nuturtlebot::WheelCommandsConstPtr& msg_ptr)        //what const ptr here?
        {
            double time_diff = ( common::Time::GetWallTime() - this->prevUpdateRate ).Double();
            prev_msg_time = common::Time::GetWallTime();
            this-> omega_l = 1.0 * (msg_ptr->left_velocity*1.0*max_motor_rot_vel/max_wheel_command);
            this-> omega_r = 1.0 * (msg_ptr->right_velocity*1.0*max_motor_rot_vel/max_wheel_command);

            ROS_INFO_STREAM("[Gazebo_plugin]: omega_l:" <<omega_l<<" | omega_r: "<<omega_r);
        }


    /// \brief ROS helper function that processes messages
    private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->ros_node->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    // Called by the world update start event
    public: void OnUpdate()
        {
            double time_diff = ( common::Time::GetWallTime() - this->prevUpdateRate ).Double();
            if (time_diff < (this->updateRate).Double() ){
                return;
            }
            else{
                this->prevUpdateRate = common::Time::GetWallTime();
            }
            
            this->joints[LEFT]->SetParam("fmax", 0, 10000.0);
            this->joints[RIGHT]->SetParam("fmax", 0, 10000.0);
            this->joints[LEFT]->SetParam("vel", 0, omega_l);
            this->joints[RIGHT]->SetParam("vel", 0, omega_r);

            //publish sensor message.
            nuturtlebot::SensorData sensor_data_msg;
            sensor_data_msg.stamp = ros::Time::now();
            sensor_data_msg.left_encoder = wrap_angle( this->joints[LEFT]->Position() )* encoder_ticks_per_rev/(2.0*PI);
            sensor_data_msg.right_encoder = wrap_angle( this->joints[RIGHT]->Position() )* encoder_ticks_per_rev/(2.0*PI);
            sensor_data_pub.publish(sensor_data_msg);
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(turtle_drive_plugin);
}