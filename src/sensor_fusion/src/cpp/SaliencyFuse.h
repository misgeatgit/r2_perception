#include "Fusion.h"

#include "sensor_fusion/CandidateSaliency.h"
#include "sensor_fusion/EstablishedSaliency.h"

class SaliencyFuse : public FusionModule {
    private:
        ros::Subscriber _sub_csaliency;
        ros::Subscriber _sub_rlsense_csaliency;
        std::vector<CandidateSaliency> csaliencyMsgBuffer; // Some Xsec duration  saliency message.
        std::queue<EstablishedSaliency> _esalencies;
        
        ros::Publisher * _pub_saliency;

        // (saliency_id, saliency_data)
        using csaliencies = std::map<unsigned int, sensor_fusion::CandidateSaliency::ConstPtr>;
        // (camera_id, csaliencies)
        std::map<unsigned int, csaliencies> cam_csaliencies;
        // Estabilished saliency
        sensor_fusion::EstablishedSaliency esaliency;

        void CSaliencyCB(const sensor_fusion::CandidateSaliency::ConstPtr &msg);

        // Fuses the candidate saliency vectors based on algorithm in the google
        // doc.gt
        void FuseCSalencyVec(void);

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

