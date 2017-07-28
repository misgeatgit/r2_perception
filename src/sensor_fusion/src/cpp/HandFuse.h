#include "Fusion.h"

#include "sensor_fusion/CandidateHand.h"
#include "sensor_fusion/EstablishedHand.h"

class HandFuse : public FusionModule {
    // (hands_id, saliency_data)
    using chands = std::map<unsigned int, sensor_fusion::CandidateHand::ConstPtr>;
    // (camera_id, chands)
    std::map<unsigned int, chands> cam_chands;
    // Established hands
    sensor_fusion::EstablishedHand ehand;

    void CHandCB(const sensor_fusion::CandidateFace::ConstPtr &msg){}

    public:
        void run(ros::NodeHandle* n);
        void publish (void);

};

