#ifndef PoseEstimation_H
#define PoseEstimation_H

#include "localization/common_include.h"
#include "localization/map.h"



namespace localization 
{
class PoseEstimation
{
public:
    typedef shared_ptr<PoseEstimation> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;        		// current VO status
        
    Map::Ptr    map_;       		// map with all frames and map points       
    Frame::Ptr  ref_;       // reference key-frame 
    Frame::Ptr  curr_;      // current frame 
     
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)
   
    SE3 T_c_w_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    

    float match_ratio_;     // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    double  map_point_erase_ratio_; // remove map point ratio
    
public: // functions 
    PoseEstimation();
    ~PoseEstimation();
    
    //bool addFrame( Frame::Ptr frame );      // add a new frame 
    
//protected:  
    // inner operation  
    bool featureMatching(const Frame::Ptr frame_curr,
						 const Frame::Ptr frame_ref); 
    void optimizeMap();
    

    bool checkEstimatedPose(); 
    bool checkKeyFrame();    
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
    void mapInitialization();
    void poseEstimationPnP();
};
}

#endif // PoseEstimation_H
