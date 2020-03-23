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
    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
        model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&TurtleDrivePlugin::OnUpdate, this));


        // wheel_cmd_topic
        if(!_sdf->GetElement("wheel_cmd_topic"))
        {
          ROS_INFO("TurtleDrivePlugin missing: wheel_cmd_topic");
        }

        else
        {
          wheel_cmd_topic = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
        }


        // sensor_topic
        if(!_sdf->GetElement("sensor_topic"))
        {
          ROS_INFO("TurtleDrivePlugin missing: sensor_topic");
        }

        else
        {
          sensor_topic = _sdf->GetElement("sensor_topic")->Get<std::string>();
        }

        // left_joint_name
        if(!_sdf->GetElement("left_wheel_joint"))
        {
          ROS_INFO("TurtleDrivePlugin missing: left_wheel_joint");
        }

        else
        {
          left_wheel_joint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
          ROS_INFO("%s", left_wheel_joint.c_str());
        }

        // right_joint_name
        if(!_sdf->GetElement("right_wheel_joint"))
        {
          ROS_INFO("TurtleDrivePlugin missing: right_wheel_joint");
        }

        else
        {
          right_wheel_joint = _sdf->GetElement("right_wheel_joint")->Get<std::string>();
          ROS_INFO("%s", right_wheel_joint.c_str());
        }

        // encoder_tics_per_rev
        if(!_sdf->GetElement("encoder_ticks_per_rev"))
        {
          ROS_INFO("TurtleDrivePlugin missing: encoder_ticks_per_rev");
        }

        else
        {
          encoder_ticks_per_rev = _sdf->GetElement("encoder_ticks_per_rev")->Get<double>();
        }

        // max_motor_rot_vel
        if(!_sdf->GetElement("max_motor_rot_vel"))
        {
          ROS_INFO("TurtleDrivePlugin missing: max_motor_rot_vel");
        }

        else
        {
          max_motor_rot_vel = _sdf->GetElement("max_motor_rot_vel")->Get<double>();
        }

        // max_motor_power
        if(!_sdf->GetElement("max_motor_power"))
        {
          ROS_INFO("TurtleDrivePlugin missing: max_motor_power");
        }

        else
        {
          max_motor_power = _sdf->GetElement("max_motor_power")->Get<double>();
        }


        // max_motor_torque
        if(!_sdf->GetElement("max_motor_torque"))
        {
          ROS_INFO("TurtleDrivePlugin missing: max_motor_torque");
        }

        else
        {
          max_motor_torque = _sdf->GetElement("max_motor_torque")->Get<double>();
        }


        // sensor_frequency
        if(!_sdf->HasElement("sensor_frequency"))
        {
          ROS_INFO("TurtleDrivePlugin sensor_frequency NOT specified setting to 200 Hz");
          Hz = 200.0;
          update_period = 1.0 / 200.0;
        }

        else
        {
          Hz = _sdf->GetElement("sensor_frequency")->Get<double>();
          update_period = 1.0 / Hz;

        }


        // initialize subscribers
        wheel_cmd_sub = node_handle.subscribe(wheel_cmd_topic, 1, &TurtleDrivePlugin::wheelCallBack, this);

        // initialize publishers
        sensor_pub = node_handle.advertise<nuturtlebot::SensorData>(sensor_topic, 1);

        // // initialize joints
        model->GetJoint(left_wheel_joint)->SetParam("fmax", 0, max_motor_torque);
        model->GetJoint(left_wheel_joint)->SetParam("vel", 0, 0.0);

        model->GetJoint(right_wheel_joint)->SetParam("fmax", 0, max_motor_torque);
        model->GetJoint(right_wheel_joint)->SetParam("vel", 0, 0.0);

        // initialize other variables
        wheel_cmd_flag = false;
        last_update_time = model->GetWorld()->SimTime();

        left_joint_speed = 0.0;
        right_joint_speed = 0.0;


        ROS_INFO("TurtleDrive Plugin Loaded!");
      }

      // Called by the world update start event
      void OnUpdate()
      {
        gazebo::common::Time current_time = model->GetWorld()->SimTime();
        double seconds_since_last_update = (current_time - last_update_time).Double();

        if (seconds_since_last_update >= update_period)
        {
          if (wheel_cmd_flag)
          {
            model->GetJoint(left_wheel_joint)->SetParam("vel", 0, left_joint_speed);
            model->GetJoint(right_wheel_joint)->SetParam("vel", 0, right_joint_speed);


            // ROS_INFO("left_joint_speed %f", left_joint_speed);
            // ROS_INFO("right_joint_speed %f", right_joint_speed);

            wheel_cmd_flag = false;
          }

          // in rad
          double left_encoder = model->GetJoint(left_wheel_joint)->Position();
          double right_encoder = model->GetJoint(right_wheel_joint)->Position();

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
      void wheelCallBack(const nuturtlebot::WheelCommands::ConstPtr &msg)
      {
        left_joint_speed = (max_motor_rot_vel / static_cast<double> (max_motor_power)) * msg->left_velocity;
        right_joint_speed = (max_motor_rot_vel / static_cast<double> (max_motor_power)) * msg->right_velocity;

        wheel_cmd_flag = true;
      }


    private:
      // Pointer to the model
      physics::ModelPtr model;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      std::string wheel_cmd_topic;

      std::string sensor_topic;

      std::string left_wheel_joint;

      std::string right_wheel_joint;

      double Hz;

      double max_motor_rot_vel;

      double max_motor_power;

      double max_motor_torque;

      double update_period;

      int encoder_ticks_per_rev;

      bool wheel_cmd_flag;

      double left_joint_speed;

      double right_joint_speed;

      gazebo::common::Time last_update_time;

      ros::NodeHandle node_handle;

      ros::Subscriber wheel_cmd_sub;

      ros::Publisher sensor_pub;

      std::vector<physics::JointPtr> joints;
  };

}

GZ_REGISTER_MODEL_PLUGIN(gazebo::TurtleDrivePlugin)













// end file
