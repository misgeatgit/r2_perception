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


/*
 * Design Rationale
 * - Modularity - Need to adapt to the addition of new sensors with minimal
 *   configuration.
 * - Plugin architectures - Loading and unloading of fusers should be smooth.
 * - Opencog integration? TODO
 */
class Fusion {
    // Implements fusion algorithms
    void virtual run(ros::NodeHandle n);
    void virtual publish(void);
    // Load Fusion libraries
    // Unload Fusers
};

