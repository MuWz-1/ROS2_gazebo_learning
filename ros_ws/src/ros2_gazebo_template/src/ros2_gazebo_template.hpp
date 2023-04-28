
#ifndef GAZEBO_PLUGIN__GAZEBO_ROS2_TEMPLATE_HPP_
#define GAZEBO_PLUFIN__GAZEBO_ROS2_TEMPLATE_HPP_


#include <gazebo/common/Plugin.hh>
#include <std_msgs/msg/string.hpp>


#include <memory>


namespace gazebo
{
    
    class GazeboRosTemplatePrivate;

    class GazeboRosTemplate : public gazebo::ModelPlugin
    {
        public: GazeboRosTemplate();
        
        public: virtual ~GazeboRosTemplate();

        public: void 
        Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        protected: virtual void
        OnMsg(const std_msgs::msg::String::ConstSharedPtr msg);

        protected: virtual void
        OnUpdate();

        private: std::unique_ptr<GazeboRosTemplatePrivate> impl_;
    };
    
} // namespace gazebo

#endif