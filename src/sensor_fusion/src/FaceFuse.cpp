#include "Fusion.h"

class FaceFuse : public Fusion {
    private:
        // (face_id, saliency_data)
        using cfaces = std::map<unsigned int, sensor_fusion::CandidateFace::ConstPtr>;
        // (camera_id, cfaces)
        std::map<unsigned int, cfaces>  cam_cfaces;
        // Established face
        sensor_fusion::EstablishedFace eface;
        struct FaceLink{
            unsigned int camera_id;
            unsigned int cface_id;
        };

        void init_eface(void){
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

        // Candidate Face topic call back handler
        void CFaceCB(const sensor_fusion::CandidateFace::ConstPtr &msg);

        void processCFaces(void);

        void run(ros::NodeHandle* node){

        }
};

void FaceFusion::CFaceCB(const sensor_fusion::CandidateFace::ConstPtr &msg){
    auto it = cam_cfaces.find(msg->camera_id);
    if(it != cameras.end()){
        //XXX do we need this mutex?
        //std::lock_guard<std::mutex> lock(face_mtx);
        (it->second)[msg->cface_id] = msg;
    } else {
        cam_cfaces[msg->camera_id] = {{msg->cface_id, msg}};
    }
}

#define FACE_FUSE_DISTANCE 0.2
void FaceFusion::processCFaces(void){
    std::vector<std::vector<FaceLink>> facegroups;
    std::vector<eface> efaces;
    
    for(const auto& p1 : cam_cfaces){
        for(const auto& p2 : cam_cfaces){
            cam_id1 = p1.first;
            cam_id2 = p2.first;
// the IDs are numeric hashes based on the unique pipeline name
            if( cam_id1 < cam_id2 ){ 
                auto fdata1 = p2.second;
                auto fdata2 = p2.second;

                float dx = fdata1->position.x - fdata2->position.x;
                float dy = fdata1->position.y - fdata2->position.y;
                float dz = fdata1->position.z - fdata2->position.z;
                float distance = sqrt(dx*dx + dy*dy + dz*dz);

                if(distance < FACE_FUSE_DISTANCE){
                    //TODO find cface code here.
                    FaceLink flink_2;
                    flink_2.camera_id =  camera_2.id;
                    flink_2.cface_id = fdata2->cface_id;

                    // iterate overall combinations of faces and group them.
                    auto it = std::find_if(facegroups.begin(), facegroups.end(),
                             [=](std::vector<FaceLink> vec){
                             auto it = std::find_if(vec.begin(), vec.end(),
                                       [=](const FaceLink& fl){ 
                                               return fdata2->camera_id == cam_id1
                                               and fl.cface_id == fdata2->cface_id;
                                        });

                            return it != vec.end(); 
                            });

                    if(it != facegroups.end()){
                        (*it).push_back(flink_2);
                    }else{
                        FaceLink flink_1;
                        flink_1.camera_id =  cam_id1;
                        flink_1.cface_id = fdata1->cface_id;
                        std::vector<FaceLink> groups;
                        groups.push_back(flink_1);        
                        groups.push_back(flink_2);     
                        facegroups.push_back(groups);   
                    }
                }
            }
        }
    }
    for (std::vector<FaceLink>& group : facegroups){
        init_eface();
        for(const FaceLink& link : group){
            auto id = link.cface_id;
            eface.position.x += cam_cfaces[link.camera_id][link.cface_id]->position.x;
            eface.position.y += cam_cfaces[link.camera_id][link.cface_id]->position.y;
            eface.position.z +=  cam_cfaces[link.camera_id][link.cface_id]->position.z;
            eface.confidence += cam_cfaces[link.camera_id][link.cface_id]->confidence;
            eface.smile += cam_cfaces[link.camera_id][link.cface_id]->smile;
            eface.frown += cam_cfaces[link.camera_id][link.cface_id]->frown;
            //FIXME eface.expressions.insert(eface.expressions.end(), (cfaces[id]->expressions).begin(), 
            //                         (cfaces[id]->expressions).end());
            eface.age += cam_cfaces[link.camera_id][link.cface_id]->age;
            eface.age_confidence += cam_cfaces[link.camera_id][link.cface_id]->age_confidence;
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
    for(const auto& p : cam_cfaces){
        for (const std::vector<FaceLink>& group : facegroups){
            cfaces cfs = p.second;
            bool found = false;
            for(const auto& p2 : cfs ){
                for(const FaceLink& link : group){
                    if (link.camera_id == p.first and link.cface_id == p2.first){}
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
    }
}
