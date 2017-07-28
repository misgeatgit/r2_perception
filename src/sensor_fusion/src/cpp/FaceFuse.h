#include "Fusion.h"

#include "ros/ros.h"
#include "ros/time.h"

#include "sensor_fusion/CandidateFace.h"
#include "sensor_fusion/EstablishedFace.h"

using namespace sensor_fusion;

class FaceFuse : virtual public FusionModule {
    private:
        // (face_id, saliency_data)
        using cfaces = std::map<unsigned int, CandidateFace::ConstPtr>;
        // (camera_id, cfaces)
        std::map<unsigned int, cfaces>  cam_cfaces;
        // Established face
        EstablishedFace eface;
        std::vector<EstablishedFace> efaces;
        struct FaceLink{
            unsigned int camera_id;
            unsigned int cface_id;
        };

        ros::Publisher * pub_eface;

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

        void processCFaces(void); //XXX When this function is invoked as a plugin how does it affect the control flow?
    public:
        // Candidate Face topic call back handler
        void CFaceCB(const CandidateFace::ConstPtr &msg);

        // Implement Publish

       void publish (void);
       void run(ros::NodeHandle* node);
};

