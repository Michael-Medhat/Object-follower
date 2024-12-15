#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <string>
#include <gz/math/Pose3.hh>



namespace gazebo
{
    /// \brief A plugin to control a Velodyne sensor.
    class controlPlugin : public ModelPlugin
    {
        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: event::ConnectionPtr update_connection_;
            /// \brief A ROS subscriber
        private: ros::Subscriber rosSub[5];

            /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue[5];

            /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread[5];
            /// \brief Pointer to the model.
        private: physics::ModelPtr model;

            /// \brief Pointer to the joint.
        private: physics::JointPtr joint[4];

        private: physics::LinkPtr servo;

        private: gz::math::Pose3d servo_pose;

        private:double wheel_speed[4];
            
        public: controlPlugin() {}
        
        public: void OnRosMsg(const std_msgs::Float32ConstPtr& _msg, int index)
        {
            wheel_speed[index] = _msg->data;
            
        }

        public: void OnRosMsg_servo(const std_msgs::Float32ConstPtr& _msg)
        {
            gz::math::Pose3d c_pose = this->servo->RelativePose();
            c_pose.Rot().Z() = _msg->data * ((22.0 / 7.0) / 180.0);
            servo_pose = c_pose;
        }

        private: void QueueThread(int index)
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue[index].callAvailable(ros::WallDuration(timeout));
            }
        }

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Just output a message for now
            std::cerr << "\nThe Control plugin is attach to model[" <<
                _model->GetName() << "]\n";
            // Store the model pointer for convenience.
            this->model = _model;

            

            //Get Joint pointers
            //Set max torque to 20
            for (int i = 0; i < 4; i++) {
                this->joint[i] = _model->GetJoints()[i];
                this->joint[i]->SetParam("fmax", 0, 20);
                wheel_speed[i] = 0;

            }

            servo = _model->GetLinks()[5];

            servo_pose = this->servo->RelativePose();


            // Initialize ros, if it has not already been initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                    ros::init_options::NoSigintHandler);
            }

            // Create our ROS node.
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            std::string joints_name[] = { "/vel_cmd/front_left_wheel", "/vel_cmd/front_right_wheel",
                            "/vel_cmd/back_left_wheel", "/vel_cmd/back_right_wheel" };
            for (int i = 0; i < 4; i++) {
                // Create a named topic, and subscribe to it.
                std::cout << joints_name[i] << std::endl;
                ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<std_msgs::Float32>(
                        joints_name[i],
                        1,
                        boost::bind(&controlPlugin::OnRosMsg, this, _1, i),
                        ros::VoidPtr(), &this->rosQueue[i]);
                this->rosSub[i] = this->rosNode->subscribe(so);

                // Spin up the queue helper thread.
                this->rosQueueThread[i] =
                    std::thread(std::bind(&controlPlugin::QueueThread, this, i));
            }

            ros::SubscribeOptions so =
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/vel_cmd/servo",
                    1,
                    boost::bind(&controlPlugin::OnRosMsg_servo, this, _1),
                    ros::VoidPtr(), &this->rosQueue[4]);
            this->rosSub[4] = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread[4] =
                std::thread(std::bind(&controlPlugin::QueueThread, this, 4));

            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&controlPlugin::UpdateChild, this));
        }

        void UpdateChild()
        {
            for (int index = 0; index < 4; index++) {
                this->joint[index]->SetVelocity(0, wheel_speed[index] / (0.3));
            }

            this->servo->SetRelativePose(servo_pose);

        }

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(controlPlugin)
}
#endif
