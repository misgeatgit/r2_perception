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

#include "timeoctomap/TimeOctomap.h"


class SaliencyFuse : public Fusion {
    // (saliency_id, saliency_data)
    using csaliencies = std::map<unsigned int, sensor_fusion::CandidateSaliency::ConstPtr>;
    // (camera_id, csaliencies)
    std::map<unsigned int, csaliencies> cam_csaliencies;
    // Estabilished saliency
    sensor_fusion::EstablishedSaliency esaliency;

    void CSaliencyCB(const sensor_fusion::CandidateSaliency::ConstPtr &msg){}
};



/**************/

class Camera{
    public:
        std::map<unsigned int, sensor_fusion::CandidateHand::ConstPtr> chands;
        std::map<unsigned int, sensor_fusion::CandidateFace::ConstPtr> cfaces;
        std::map<unsigned int, sensor_fusion::CandidateSaliency::ConstPtr> csaliencies;

    public:
        Camera(unsigned int cam_id){
            id = cam_id;
        }
        Camera() = default;  

        bool operator < (const Camera& camera) const{
            return id < camera.id;
        }
        void insert_hand_data(const unsigned int hand_id,
                sensor_fusion::CandidateHand::ConstPtr chand){
            chands[hand_id] = chand;
        }
        void insert_face_data(const unsigned int face_id,
                sensor_fusion::CandidateFace::ConstPtr cface){
            cfaces[face_id] = cface;
        }
        void insert_saliency_data(const unsigned int saliency_id,
                sensor_fusion::CandidateSaliency::ConstPtr csaliency){
            csaliencies[saliency_id] = csaliency;
        }

        unsigned int id;
};

// To be stored in the octomap
struct PerceptionData{
    sensor_fusion::EstablishedFace eface;
    sensor_fusion::EstablishedHand ehand;
    sensor_fusion::EstablishedSaliency esaliency;
    PerceptionData(){
        eface.position.x = 0.0;
        eface.position.y = 0.0;
        eface.position.z = 0.0;
        eface.confidence = 0.0;
        eface.smile = 0.0;
        eface.frown = 0.0;
        eface.expressions.push_back("");
        eface.age = 0.0;
        eface.age_confidence = 0.0;
        eface.gender = 0;
        eface.gender_confidence = 0.0;
        eface.identity = 0;
        eface.identity_confidence = 0;
    }
};

struct FaceLink{
    unsigned int camera_id;
    unsigned int cface_id;
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
        cameras[cm.id] = cm;
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
        cameras[cm.id] = cm;
    }
}

// Message Publishers
void PublishFaceMsg(const ros::Publisher& pub){

}

void PublishHandMsg(const ros::Publisher& pub){

}

void PublishSaliencyMsg(const ros::Publisher& pub){

}

#define FACE_FUSE_DISTANCE 0.2 //FIXME
// Timer call back
void timerCB(const ros::TimerEvent& event){
    static std::vector<std::vector<FaceLink>> facegroups;
    static std::vector<sensor_fusion::EstablishedFace> efaces;

    assert(cameras.size() == 2);

    Camera& camera_1 = cameras[0];
    Camera& camera_2 = cameras[1];

    for(const auto& p1 : camera_1.cfaces){
        for(const auto& p2 : camera_2.cfaces){
            float dx = (p1.second)->position.x - (p2.second)->position.x;
            float dy = (p1.second)->position.y - (p2.second)->position.y;
            float dz = (p1.second)->position.z - (p2.second)->position.z;
            float distance = sqrt(dx*dx + dy*dy + dz*dz);

            if(distance < FACE_FUSE_DISTANCE){
                //TODO find cface code here.
                FaceLink flink_2;
                flink_2.camera_id =  camera_2.id;
                flink_2.cface_id = (p2.second)->cface_id;

                // iterate overall combinations of faces and group them.
                auto it = std::find_if(facegroups.begin(), facegroups.end(), [=](std::vector<FaceLink> vec){
                        auto it = std::find_if(vec.begin(), vec.end(), [=](const FaceLink& fl){ 
                            return fl.cface_id == (p2.second)->cface_id; });
                        return it != vec.end(); });
                if(it != facegroups.end()){
                    (*it).push_back(flink_2);
                }else{
                    FaceLink flink_1;
                    flink_1.camera_id =  camera_1.id;
                    flink_1.cface_id = (p1.second)->cface_id;
                    std::vector<FaceLink> groups;
                    groups.push_back(flink_1);        
                    groups.push_back(flink_2);     
                    facegroups.push_back(groups);   
                }
            }
        }
    }

    for (std::vector<FaceLink>& group : facegroups){
        PerceptionData pdata;
        sensor_fusion::EstablishedFace& eface = pdata.eface; 
        auto cfaces = camera_1.cfaces;
        for(const FaceLink& link : group){
            auto id = link.cface_id;
            eface.position.x += cfaces[id]->position.x;
            eface.position.y += cfaces[id]->position.y;
            eface.position.z += cfaces[id]->position.z;
            eface.confidence += cfaces[id]->confidence;
            eface.smile +=cfaces[id]->smile;
            eface.frown += cfaces[id]->frown;
            //FIXME eface.expressions.insert(eface.expressions.end(), (cfaces[id]->expressions).begin(), 
            //                         (cfaces[id]->expressions).end());
            eface.age += cfaces[id]->age;
            eface.age_confidence += cfaces[id]->age_confidence;
        }

        size_t n = group.size();
        eface.position.x /= n;
        eface.position.y /= n;
        eface.position.z /= n;
        eface.confidence /= n;
        eface.smile /= n;
        eface.frown /= n;
        eface.age /= n;
        eface.age_confidence /= n;

        efaces.push_back(eface);
    }
    // TODO: gender is the most likely of any of the group
    // TODO: identity is the most likely of any of the group
    // create established face for all faces not referenced in any group
    auto  convert_to_established_face = [=] (const Camera& cm){
        for (const std::vector<FaceLink>& group : facegroups){
            std::map<unsigned int, sensor_fusion::CandidateFace::ConstPtr>  cfaces = cm.cfaces;
            bool found = false;
            for(const auto& p : cfaces ){
                for(const FaceLink& link : group){
                    if (link.camera_id == cm.id and link.cface_id == p.first){}
                    found = true;
                    break;
                }
                if(found){
                    break;
                } else{
                    PerceptionData pdata;
                    sensor_fusion::EstablishedFace& eface = pdata.eface; 
                    //FIXME
                    //eface.session_id = self.session_id
                    //eface.ts = ts
                    eface.position.x = (p.second)->position.x;
                    eface.position.y = (p.second)->position.y;
                    eface.position.z = (p.second)->position.z;
                    eface.confidence = (p.second)->confidence;
                    eface.smile = (p.second)->smile;
                    eface.frown = (p.second)->frown;
                    //FIXME
                    //eface.expressions = (p.second)->expressions;
                    eface.age = (p.second)->age;
                    eface.age_confidence = (p.second)->age_confidence;
                    eface.gender = (p.second)->gender;
                    eface.gender_confidence = (p.second)->gender_confidence;
                    eface.identity = (p.second)->identity;
                    eface.identity_confidence = (p.second)->identity_confidence;

                    efaces.push_back(eface);
                }
            }
        }
    };

    convert_to_established_face(camera_1);
    convert_to_established_face(camera_2);
    // fuse candidate hands between pipelines
    // check existing hands and either fuse or add
    // fuse candidate salient points between pipelines by calculating shortest vector distance
    // check existing salient points and combine; here just match the vectors, only keep fused salient points for now
    // fuse faces and saliencies to improve saliency confidence
    // TODO: compare salient points and faces to improve faces
    // fuse hands and saliencies to improve saliency confidence
    // TODO: compare salient points and hands to improve hands
    // fuse sounds and faces
    // fuse sounds and hands
    // fuse sounds and saliency
    // output all established stuff
    // TODO: send markers to RViz
    // clean out old saliencies from self.csaliencies
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
     * XXX OPTIONAL TO THE TIMER INVOCATION AS IN ABOVE.
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
