#include "Fusion.h"

class HandFuse : public Fusion {
    // (hands_id, saliency_data)
    using chands = std::map<unsigned int, sensor_fusion::CandidateHand::ConstPtr>;
    // (camera_id, chands)
    std::map<unsigned int, chands> cam_chands;
    // Established hands
    sensor_fusion::EstablishedHand ehand;

    void CHandCB(const sensor_fusion::CandidateFace::ConstPtr &msg){}
};

