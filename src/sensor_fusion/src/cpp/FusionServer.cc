#include "ros/ros.h"
#include "Fusion.h"

class FusionServer{
    private:
        std::vector<FusionModule*> _modules;
        ros::Rate _loop_rate;

    public:
        FusionServer(unsigned int loop_rate = 10);
        ~FusionServer();

        // Dynamical loader for ROS fusion nodes
        bool load(std::string path);
        void unload(std::string fusion);


        // This is where all the ros topics are published.
        void run(void){
            while (ros::ok()){
                for(const FusionModule* module : _modules){
                    module->publish();
                } 
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
};

FusionServer::FusionServer(unsigned int loop_rate/*=10*/) :_loop_rate(loop_rate){

}

bool FusionServer::load(std::string filename){
    void *dynLibrary = dlopen(filename.c_str(), RTLD_LAZY | RTLD_GLOBAL);
    const char* dlsymError = dlerror();
    if ((dynLibrary == NULL) || (dlsymError)) {
        //TODO ROS log
        // This is almost surely due to a user configuration error.
        // User errors are always logged as warnings.
        return false;
    }

    // reset error
    dlerror();

    // search for 'load' & 'unload' symbols
    FusionModule* module = (FusionModule* (*) (void)) dlsym(dynLibrary, "fusion_module_load");
    dlsymError = dlerror();
    if (dlsymError) {
        //TODO ROS log
        return false;
    }

    // push publish to the list of publishers
    //TODO make sure the module is not added more than once.
    modules.push_back(module);

    // load and init module
    module->run();

    return true;
}

void FusionServer::unload(std::string fusion){

}
