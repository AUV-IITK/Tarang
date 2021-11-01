#ifndef _GRABBER_CONTROL_PLUGIN_HH_
#define _GRABBER_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/LinkStates.h>

#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Bool.h"


namespace gazebo
{

    class GrabberControlPlugin : public gazebo::ModelPlugin
    {
        public:

        GrabberControlPlugin(){}

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, Arm Control Plugin not loaded\n";
                return;
            }

            this->model = _model;
            this->grabber_state = false;

            this->claw_joint_1 = _model->GetJoint("triton/grabber_claw_0_joint");
            this->claw_joint_2 = _model->GetJoint("triton/grabber_claw_1_joint");
            this->claw_joint_3 = _model->GetJoint("triton/grabber_claw_2_joint");
            this->claw_joint_4 = _model->GetJoint("triton/grabber_claw_3_joint");
            this->rod_joint = _model->GetJoint("triton/grabber_rod_joint");

            this->node= gazebo::transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->Name());

            // std::string linkTopicName = "gazebo/link_states";
            std::string grabberStateTopicName = this->model->GetName()+"/grabber_state";

            if (!ros::isInitialized())
            {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "grabber_gazebo_plugin_node",ros::init_options::NoSigintHandler);
            }
            this->rosNode.reset(new ros::NodeHandle("grabber_gazebo_plugin_node"));

            // this->linkSubscriber = this->node->Subscribe(linkTopicName, &GrabberControlPlugin::link_callback, this);
            // this->linkPublisher = this->node->Advertise<gazebo_msgs::LinkStates>(linkTopicName, 1000);

            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(grabberStateTopicName, 1, 
                                                                                        boost::bind(&GrabberControlPlugin::state_callback, this, _1),
                                                                                        ros::VoidPtr(), &this->rosQueue);
            this->grabberStateSubscriber = this->rosNode->subscribe(so);

            this->rosQueueThread = std::thread(std::bind(&GrabberControlPlugin::QueueThread, this));

        }

        void state_callback(const std_msgs::BoolConstPtr &_msg)
        {
            // gazebo_msgs::LinkStates initial_state = this->presentLinkStates;
            float initialRodState;
            float initialClawState;
            float finalRodState;
            float finalClawState;
            float rodStepSize = 0.0001;
            float clawStepSize = 0.0069;


            if(_msg->data)
            {
                finalRodState = 0.01;
                finalClawState = -0.69;
                initialRodState = 0.0;
                initialClawState = 0.0;
            }

            else
            {
                finalRodState = 0.0;
                finalClawState = 0.0;
                initialRodState = 0.01;
                initialClawState = -0.69;
            }

            float steps = 100;
            float rodStepSign = (finalRodState-initialRodState)/abs(finalRodState-initialRodState);
            float clawStepSign = (finalClawState-initialClawState)/abs(finalClawState-initialClawState);
            float rodLength = initialRodState;
            float clawAngle = initialClawState;

            if(this->grabber_state != _msg->data)
            {
                this->grabber_state =_msg->data;

                for (int i=1; i<=steps; i++)
                {
                    rodLength += (rodStepSign*rodStepSize);
                    clawAngle += (clawStepSign*clawStepSize);

                    this->claw_joint_1->SetLowerLimit(0,clawAngle);
                    this->claw_joint_1->SetUpperLimit(0,clawAngle);
                    this->claw_joint_2->SetLowerLimit(0,clawAngle);
                    this->claw_joint_2->SetUpperLimit(0,clawAngle);
                    this->claw_joint_3->SetLowerLimit(0,clawAngle);
                    this->claw_joint_3->SetUpperLimit(0,clawAngle);
                    this->claw_joint_4->SetLowerLimit(0,clawAngle);
                    this->claw_joint_4->SetUpperLimit(0,clawAngle);
                    this->rod_joint->SetLowerLimit(0,rodLength);
                    this->rod_joint->SetUpperLimit(0,rodLength);

                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            
        }

        private:

        // void link_callback(const gazebo_msgs::LinkStatesConstPtr &_msg)
        // {
        //     this->presentLinkStates = _msg->data;
        // }

        void QueueThread()
        {
            static const double timeout = 0.01;

            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private:

        gazebo::physics::ModelPtr model;

        gazebo::physics::JointPtr claw_joint_1;
        gazebo::physics::JointPtr claw_joint_2;
        gazebo::physics::JointPtr claw_joint_3;
        gazebo::physics::JointPtr claw_joint_4;
        gazebo::physics::JointPtr rod_joint;

        gazebo::transport::NodePtr node;

        std::unique_ptr<ros::NodeHandle> rosNode;

        ros::Subscriber grabberStateSubscriber;

        // gazebo::transport::SubscriberPtr linkSubscriber;
        // gazebo::transport::PublisherPtr linkPublisher;

        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        bool grabber_state;

        // gazebo_msgs::LinkStatesPtr presentLinkStates;
    };
    GZ_REGISTER_MODEL_PLUGIN(GrabberControlPlugin)
}
#endif
