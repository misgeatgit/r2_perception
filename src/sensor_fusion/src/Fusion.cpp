#include "ros/ros.h"
#include "ros/time.h"

// Dynamic reconfigure includes.
#include "dynamic_reconfigure/server.h"

// Auto-generated from cfg/ directory.
#include "sensor_fusion/hearing_pipelineConfig.h"
#include "sensor_fusion/vision_pipelineConfig.h"

// Auto-generated from msg/ directory.
#include "sensor_fusion/CandidateFace.h"
#include "sensor_fusion/CandidateHand.h"
#include "sensor_fusion/CandidateSaliency.h"
#include "sensor_fusion/EstablishedFace.h"
#include "sensor_fusion/EstablishedHand.h"
#include "sensor_fusion/EstablishedSaliency.h"


// Message Subscribers
void CandidateFaceCB(const sensor_fusion::CandidateFace::ConstPtr &msg){

}
void CandidateHandCB(const sensor_fusion::CandidateFace::ConstPtr &msg){

}
void CandidateSaliencyCB(const sensor_fusion::CandidateFace::ConstPtr &msg){

}

// Message Publishers
void PublishFaceMsg(const ros::Publisher& pub){

}

void PublishHandMsg(const ros::Publisher& pub){

}

void PublishSaliencyMsg(const ros::Publisher& pub){

}


int main(int argc, char** args){

    // Published by fusion
    ros::NodeHandle n;

    ros::Publisher pub_eface = n.advertise<sensor_fusion::EstablishedFace>("face", 5);
    ros::Publisher pub_ehand = n.advertise<sensor_fusion::EstablishedHand>("hand", 5);
    ros::Publisher pub_esaliency = n.advertise<sensor_fusion::EstablishedSaliency>("saliency", 5);

    // Fusion node listens to:
    ros::Subscriber sub_leye_cface = n.subscribe("lefteye/cface", 1000, &CandidateFaceCB);
    ros::Subscriber sub_reye_cface = n.subscribe("righteye/cface", 1000, &CandidateFaceCB);
    ros::Subscriber sub_rlsense_cface = n.subscribe("realsense/cface", 1000, &CandidateFaceCB);
    ros::Subscriber sub_wideangle_cface = n.subscribe("wideangle/cface", 1000, &CandidateFaceCB);

    ros::Subscriber sub_leye_chand = n.subscribe("lefteye/chand", 1000, &CandidateHandCB);
    ros::Subscriber sub_reye_chand = n.subscribe("righteye/chand", 1000, &CandidateHandCB);
    ros::Subscriber sub_rlsense_chand = n.subscribe("realsense/chand", 1000, &CandidateHandCB);
    ros::Subscriber sub_wideangle_chand = n.subscribe("wideangle/chand", 1000, &CandidateHandCB);

    ros::Subscriber sub_leye_csaliency = n.subscribe("lefteye/csaliency", 1000, &CandidateSaliencyCB);
    ros::Subscriber sub_reye_csaliency = n.subscribe("righteye/csaliency", 1000, &CandidateSaliencyCB);
    ros::Subscriber sub_rlsense_csaliency = n.subscribe("realsense/csaliency", 1000, &CandidateSaliencyCB);
    ros::Subscriber sub_wideangle_csaliency = n.subscribe("wideangle/csaliency", 1000, &CandidateSaliencyCB);


    ROS_INFO("Ready to add two ints.");
    ros::spin();

    // Tell ROS how fast to run this node.
    //ros::Rate r(500);
    // Main loop.
    while (n.ok())
    {
        // Publish the message.
        PublishFaceMsg(pub_eface);
        PublishHandMsg(pub_ehand);
        PublishSaliencyMsg(pub_esaliency);

        ros::spinOnce();
        //r.sleep();
    }
    return 0;
}

