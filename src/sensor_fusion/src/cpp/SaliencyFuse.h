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


/* Saliency Fusion algo

     - Pa = Aa + ANa
       Pb = Bb + BNb

       find A and B then decide how close are Pa and Pb using the fitness
       function  Fc = 1/2(Pa+Pb)
       if Fc <= Threshold, then fuse them.
*/

