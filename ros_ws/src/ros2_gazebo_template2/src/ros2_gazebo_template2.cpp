#ifndef _ROS2_GAZEBO_TEMPLATE2_
#define _ROS2_GAZEBO_TEMPLATE2_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
// #include <gazebo/physics/World.hh>
// #include <gazebo/physics/Model.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
// #include <gazebo_ros/builtin_interfaces.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// #include <geometry_msgs/msg/pose2_d.hpp>
// #include <nav_msgs/msg/odometry.hpp> // navigation
#include <sdf/sdf.hh> // All the other files under the folder "sdf".

#include <memory>
#include <string>
#include <vector>

namespace gazebo
{
    class GazeboRosTemplate2 : public ModelPlugin
    {
        public: GazeboRosTemplate2(){}
        public: virtual void
        Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model_ = _model;
            this->joint_ = _model->GetJoints()[0];
            this->pid_ = gazebo::common::PID(0.1, 0, 0);
            this->model_->GetJointController()->SetVelocityPID(
                this->joint_->GetScopedName(), this->pid_);
            double veloicity = 10;
            this->SetVelocity(veloicity);
            
            // Initialize ROS node
            this->ros_node_ = gazebo_ros::Node::Get(_sdf);
 
            // Get QoS profiles
            const gazebo_ros::QoS& qos = this->ros_node_->get_qos();

            this->cmd_vel_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
                "cmd_vel",
                qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
                std::bind(&GazeboRosTemplate2::OnCmdVel, this, std::placeholders::_1)
            );

            RCLCPP_INFO(
                this->ros_node_->get_logger(), "Subscribed to [%s]",
                this->cmd_vel_sub_->get_topic_name()
            );

            // Listen to the update event (broadcast every simulation iteration)
            this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&GazeboRosTemplate2::OnUpdate, this, std::placeholders::_1)
            );
        }
        public: void
        OnUpdate(const gazebo::common::UpdateInfo& _info)
        {

        }
        public: void
        SetVelocity(const double& _vel)
        {
            this->model_->GetJointController()->SetVelocityTarget(
                this->joint_->GetScopedName(), _vel
            );
        }
        public: void
        OnCmdVel(const geometry_msgs::msg::Vector3::SharedPtr _msg)
        {
            this->SetVelocity(_msg->x);
        }

        private: gazebo::physics::ModelPtr model_;
        private: gazebo::physics::JointPtr joint_;
        private: gazebo::common::PID pid_;
        private: gazebo::event::ConnectionPtr update_connection_;
        
        private: gazebo_ros::Node::SharedPtr ros_node_;
        private: rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_vel_sub_;

    };

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate2)
}





#endif