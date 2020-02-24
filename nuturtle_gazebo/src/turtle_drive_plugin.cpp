/// \file
/// \brief Gazebo plugin for turtlbot wheel control

#include <functional>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rigid2d/rigid2d.hpp>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>



namespace gazebo
{
  class TurtleDrivePlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL("A ROS node for Gazebo has not been initialized."
                  "Unable to load plugin. Load the Gazebo system plugin"
                  "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
      }

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TurtleDrivePlugin::OnUpdate, this));


      // wheel_cmd_topic
      if(!_sdf->GetElement("wheel_cmd_topic"))
      {
        ROS_INFO("TurtleDrivePlugin missing: wheel_cmd_topic");
      }

      else
      {
        this->wheel_cmd_topic = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
      }


      // sensor_topic
      if(!_sdf->GetElement("sensor_topic"))
      {
        ROS_INFO("TurtleDrivePlugin missing: sensor_topic");
      }

      else
      {
        this->sensor_topic = _sdf->GetElement("sensor_topic")->Get<std::string>();
      }

      // left_joint_name
      if(!_sdf->GetElement("left_wheel_joint"))
      {
        ROS_INFO("TurtleDrivePlugin missing: left_wheel_joint");
      }

      else
      {
        this->left_wheel_joint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
        ROS_WARN("%s", left_wheel_joint.c_str());
      }

      // right_joint_name
      if(!_sdf->GetElement("right_wheel_joint"))
      {
        ROS_INFO("TurtleDrivePlugin missing: right_wheel_joint");
      }

      else
      {
        this->right_wheel_joint = _sdf->GetElement("right_wheel_joint")->Get<std::string>();
        ROS_WARN("%s", right_wheel_joint.c_str());
      }

      // encoder_tics_per_rev
      if(!_sdf->GetElement("encoder_ticks_per_rev"))
      {
        ROS_INFO("TurtleDrivePlugin missing: encoder_ticks_per_rev");
      }

      else
      {
        this->encoder_ticks_per_rev = _sdf->GetElement("encoder_ticks_per_rev")->Get<double>();
      }

      // max_motor_rot_vel
      if(!_sdf->GetElement("max_motor_rot_vel"))
      {
        ROS_INFO("TurtleDrivePlugin missing: max_motor_rot_vel");
      }

      else
      {
        this->max_motor_rot_vel = _sdf->GetElement("max_motor_rot_vel")->Get<double>();
      }

      // max_motor_power
      if(!_sdf->GetElement("max_motor_power"))
      {
        ROS_INFO("TurtleDrivePlugin missing: max_motor_power");
      }

      else
      {
        this->max_motor_power = _sdf->GetElement("max_motor_power")->Get<double>();
      }


      // max_motor_torque
      if(!_sdf->GetElement("max_motor_torque"))
      {
        ROS_INFO("TurtleDrivePlugin missing: max_motor_torque");
      }

      else
      {
        this->max_motor_torque = _sdf->GetElement("max_motor_torque")->Get<double>();
      }


      // sensor_frequency
      if(!_sdf->HasElement("sensor_frequency"))
      {
        ROS_INFO("TurtleDrivePlugin sensor_frequency NOT specified setting to 200 Hz");
        this->Hz = 200.0;
        this->update_period = 1.0 / 200.0;
      }

      else
      {
        this->Hz = _sdf->GetElement("sensor_frequency")->Get<double>();
        this->update_period = 1.0 / this->Hz;

      }


      // initialize subscribers
      this->wheel_cmd_sub = node_handle.subscribe(this->wheel_cmd_topic, 1, &TurtleDrivePlugin::wheelCallBack, this);

      // initialize publishers
      this->sensor_pub = node_handle.advertise<nuturtlebot::SensorData>(this->sensor_topic, 1);

      // // initialize joints
      this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0, max_motor_torque);
      this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0, 0.0);

      this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0, max_motor_torque);
      this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0, 0.0);

      // initialize other variables
      this->wheel_cmd_flag = false;
      this->last_update_time = model->GetWorld()->SimTime();

      this->left_joint_speed = 0.0;
      this->right_joint_speed = 0.0;


      ROS_INFO("TurtleDrive Plugin Loaded!");

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      gazebo::common::Time current_time = model->GetWorld()->SimTime();
      double seconds_since_last_update = (current_time - last_update_time).Double();

      if (seconds_since_last_update >= update_period)
      {
        if (wheel_cmd_flag)
        {
          this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0, left_joint_speed);
          this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0, right_joint_speed);


          ROS_INFO("left_joint_speed %f", left_joint_speed);
          ROS_INFO("right_joint_speed %f", right_joint_speed);

          wheel_cmd_flag = false;
        }

        // in rad
        double left_encoder = this->model->GetJoint(left_wheel_joint)->Position();
        double right_encoder = this->model->GetJoint(right_wheel_joint)->Position();

        // ROS_INFO("left_encoder %f", left_encoder);
        // ROS_INFO("right_encoder %f", right_encoder);

        // convert to ticks
        left_encoder = (static_cast<double> (encoder_ticks_per_rev) / (2.0 * rigid2d::PI)) * left_encoder;
        right_encoder = (static_cast<double> (encoder_ticks_per_rev) / (2.0 * rigid2d::PI)) * right_encoder;



        nuturtlebot::SensorData sensor_data;
        sensor_data.left_encoder = static_cast<int> (std::round(left_encoder));
        sensor_data.right_encoder = static_cast<int> (std::round(right_encoder));


        last_update_time = current_time;

        sensor_pub.publish(sensor_data);
      }
    }


    /// \brief updates wheel commands
    /// \param msg - contains wheel commands
    public: void wheelCallBack(const nuturtlebot::WheelCommands::ConstPtr &msg)
    {
      left_joint_speed = (max_motor_rot_vel / static_cast<double> (max_motor_power)) * msg->left_velocity;
      right_joint_speed = (max_motor_rot_vel / static_cast<double> (max_motor_power)) * msg->right_velocity;

      wheel_cmd_flag = true;
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: std::string wheel_cmd_topic;

    private: std::string sensor_topic;

    private: std::string left_wheel_joint;

    private: std::string right_wheel_joint;

    private: double Hz;

    private: double max_motor_rot_vel;

    private: double max_motor_power;

    private: double max_motor_torque;

    private: double update_period;

    private: int encoder_ticks_per_rev;

    private: bool wheel_cmd_flag;

    private: double left_joint_speed;

    private: double right_joint_speed;

    private: gazebo::common::Time last_update_time;

    private: ros::NodeHandle node_handle; /*std::unique_ptr<ros::NodeHandle> node_handle;*/

    private: ros::Subscriber wheel_cmd_sub;

    private: ros::Publisher sensor_pub;

    private: std::vector<physics::JointPtr> joints;
  };

}

GZ_REGISTER_MODEL_PLUGIN(gazebo::TurtleDrivePlugin)













// end file
