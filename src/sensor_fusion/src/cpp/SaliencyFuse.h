#include "Fusion.h"

#include "sensor_fusion/CandidateSaliency.h"
#include "sensor_fusion/EstablishedSaliency.h"

class SaliencyFuse : public FusionModule {
    // (saliency_id, saliency_data)
    using csaliencies = std::map<unsigned int, sensor_fusion::CandidateSaliency::ConstPtr>;
    // (camera_id, csaliencies)
    std::map<unsigned int, csaliencies> cam_csaliencies;
    // Estabilished saliency
    sensor_fusion::EstablishedSaliency esaliency;

    void CSaliencyCB(const sensor_fusion::CandidateSaliency::ConstPtr &msg){}

    public:
        void run(ros::NodeHandle* n);
        void publish (void);

};


