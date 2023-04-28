#include <gazebo/physics/Model.hh>

#include <gazebo_plugins/gazebo_ros_elevator.hpp>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "ros2_gazebo_template.hpp"

namespace gazebo
{
	class GazeboRosTemplatePrivate
	{
	public:
		/// Connection to the world update event. Callback will be called while this is alive.
		gazebo::event::ConnectionPtr update_connection_;
		/// Node for ROS communication.
		gazebo_ros::Node::SharedPtr ros_node_;
		/// Subscription to elevator commands;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
	};

	GazeboRosTemplate::GazeboRosTemplate()
		: impl_(std::make_unique<GazeboRosTemplatePrivate>())
	{}

	GazeboRosTemplate::~GazeboRosTemplate()
	{}

	void GazeboRosTemplate::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		// Create a GazeboRos node instead of a common ROS node.
		// Pass it's SDF parameters so that common options
		// like namespace and remapping can be handled.
		impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
		// Get QoS profiles.
		const gazebo_ros::QoS& qos = impl_->ros_node_->get_qos();
		// The ModelPtr can give you direct access to the physics object,
		// for example:
		RCLCPP_INFO(impl_->ros_node_->get_logger(), _model->GetName().c_str());
		impl_->subscriber_ = impl_->ros_node_->create_subscription<std_msgs::msg::String>(
			"elevator",
			// topic name
			qos.get_subscription_qos("elevator", rclcpp::QoS(1)),
			// qos_history_depth(The depth of the subscription's incoming message queue.)
			std::bind(&GazeboRosTemplate::OnMsg, this, std::placeholders::_1)
			// rclcpp::callback_group::CallbackGroup::SharedPtr
			// callback(The user-defined callback function.)
			// std::bind 用来将可调用对象与其参数一起进行绑定。
			// 绑定后的结果可以使用 std::function 进行保存（实际上 std::bind 的返回类型是一个 stl 内部定义的仿函数类型），
			// 并延迟调用到任何我们需要的时候。
			// std::placeholders::_1 是一个占位符，代表这个位置将在函数调用时，
			// 被传入的第一个参数所替代。
		);
		RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->subscriber_->get_topic_name());
		// Create a connection so that the OnUpdate function will be called with every simulation iteration. 
		// Remove this call, the connection and this callback if they are not needed.
		impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
			std::bind(&GazeboRosTemplate::OnUpdate, this)
		);
	}

	void GazeboRosTemplate::OnMsg(const std_msgs::msg::String::ConstSharedPtr _msg)
	{
		RCLCPP_INFO(impl_->ros_node_->get_logger(), "I heard!");
	}

	void GazeboRosTemplate::OnUpdate()
	{
		// Do something with every simulation iteration.
	}
	// OnUpdate函数。这个函数可以用来让模型作为Publisher向某个ROS节点发布消息，
	// 这样就可以获取机器人的一些信息。

	GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate)
}
