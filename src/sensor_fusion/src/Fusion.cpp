#include <mutex>
#include <thread>

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

class Camera{
     private:
         std::map<unsigned int, sensor_fusion::CandidateHand::ConstPtr> chands;
         std::map<unsigned int, sensor_fusion::CandidateFace::ConstPtr> cfaces;
         std::map<unsigned int, sensor_fusion::CandidateSaliency::ConstPtr> csaliencies;
         
     public:
         Camera(unsigned int cam_id){
             id = cam_id;
         }
        // ~Camera(){}
       
         //Camera() = default;  

         Camera(const Camera& camera){}
        
         bool operator < (const Camera& camera) const
         {
             return id < camera.id;
         }
         
         void insert_hand_data(const unsigned int hand_id, sensor_fusion::CandidateHand::ConstPtr chand){
             chands[hand_id] = chand;
         }
         void insert_face_data(const unsigned int face_id, sensor_fusion::CandidateFace::ConstPtr cface){
             cfaces[face_id] = cface;
         }
         void insert_saliency_data(const unsigned int saliency_id, sensor_fusion::CandidateSaliency::ConstPtr csaliency){
             csaliencies[saliency_id] = csaliency;
         }

         unsigned int id;
};

std::map<unsigned int, Camera> cameras;
std::mutex hand_mtx, face_mtx, saliency_mtx;

// Message Subscribers
void CandidateFaceCB(const sensor_fusion::CandidateFace::ConstPtr &msg){
    auto it = cameras.find(msg->camera_id);
    if(it != cameras.end()){
        std::lock_guard<std::mutex> lock(face_mtx);
        (it->second).insert_face_data(msg->cface_id, msg);
    } else {
        Camera cm(msg->camera_id);
        cm.insert_face_data(msg->cface_id, msg);
        cameras[cm.id] = cm;
    }
}
void CandidateHandCB(const sensor_fusion::CandidateHand::ConstPtr &msg){
    auto it = cameras.find(msg->camera_id);
    if(it != cameras.end()){
        std::lock_guard<std::mutex> lock(hand_mtx);
        (it->second).insert_hand_data(msg->chand_id, msg);
    } else {
        Camera cm(msg->camera_id);
        cm.insert_hand_data(msg->chand_id, msg);
        // cameras[cm.id] = cm;
    }
}
void CandidateSaliencyCB(const sensor_fusion::CandidateSaliency::ConstPtr &msg){
    auto it = cameras.find(msg->camera_id);
    if(it != cameras.end()){
        std::lock_guard<std::mutex> lock(saliency_mtx);
        (it->second).insert_saliency_data(msg->csaliency_id, msg);
    } else {
        Camera cm(msg->camera_id);
        cm.insert_saliency_data(msg->csaliency_id, msg);
        // cameras[cm.id] = cm;
    }
}

// Message Publishers
void PublishFaceMsg(const ros::Publisher& pub){

}

void PublishHandMsg(const ros::Publisher& pub){

}

void PublishSaliencyMsg(const ros::Publisher& pub){

}

// Timer call back
void timerCB(const ros::TimerEvent& event){

}

int main(int argc, char** args){

    // Published by fusion
    ros::NodeHandle n;

    ros::Publisher pub_eface = n.advertise<sensor_fusion::EstablishedFace>("face", 5);
    ros::Publisher pub_ehand = n.advertise<sensor_fusion::EstablishedHand>("hand", 5);
    ros::Publisher pub_esaliency = n.advertise<sensor_fusion::EstablishedSaliency>("saliency", 5);

    // Fusion node listeners to:
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

    //XXX Here according to Desmond's architecture, subscriber's store values
    // they read and HandleTimer will be invoked every few cycles to process
    // the read values and finally publishes each pocessed information to one
    // of the publishers.
    //timer = rospy.Timer(rospy.Duration(1.0 / self.fusion_rate),self.HandleTimer)

    double fusion_rate;
    std::string visualize;
    std::string store_thumbs_flag;
    std::string session_tag;
    ros::param::get("/session_tag", session_tag);
    ros::param::get("/store_thumbs_flag", store_thumbs_flag);
    ros::param::get("/visualize", visualize);
    ros::param::get("/fusion_rate", fusion_rate);

    ros::Timer timer = n.createTimer(ros::Duration(1/fusion_rate), timerCB);
    /*
    // Tell ROS how fast to run this node.
    ros::Rate r(fusion_rate);
    // Main loop.
    while (n.ok()){
    // Publish the message.
    PublishFaceMsg(pub_eface);
    PublishHandMsg(pub_ehand);
    PublishSaliencyMsg(pub_esaliency);

    ros::spinOnce();
    r.sleep();
    }*/

    return 0;
}

