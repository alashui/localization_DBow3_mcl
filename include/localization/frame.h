#ifndef FRAME_H
#define FRAME_H

#include "localization/common_include.h"
#include "localization/camera.h"
#include "localization/config.h"

namespace localization 
{
    
// forward declare 
//class MapPoint;
class Frame
{
	public:
		typedef std::shared_ptr<Frame> Ptr;
		unsigned long                  id_;         // id of this frame
		double                         time_stamp_; // when it is recorded
		SE3                            T_c_w_;      // transform from world to camera
		Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
		Mat                        color_, depth_; // color and depth image 
		
		cv::Ptr<cv::ORB> 			   orb_;  // orb detector and computer
		std::vector<cv::KeyPoint>      keypoints_;  // key points in image
		Mat                     	   descriptors_;  // descriptors in image
		
		// parameters 
    	int num_of_features_;   // number of features
    	double scale_factor_;   // scale in image pyramid
    	int level_pyramid_;     // number of pyramid levels
		
		bool                           is_key_frame_;  // whether a key-frame
		
			
		// std::vector<MapPoint*>         map_points_; // associated map points
		//bool                           is_key_frame_;  // whether a key-frame
		
	public: // data members 
		Frame();
		Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
		~Frame();
		
		static Frame::Ptr createFrame(); 
		
		// find the depth in depth map
		double findDepth( const cv::KeyPoint& kp );
		
		void setPose( const SE3& T_c_w ){T_c_w_ = T_c_w;}
		void setKpts_Dpts(std::vector<cv::KeyPoint> keypoints,Mat descriptors)
			{keypoints_=keypoints;descriptors_=descriptors;}
	
		void extractKeyPoints();			
		void computeDescriptors();

				
		// Get Camera Center
		Vector3d getCamCenter() const;
				
		
		// check if a point is in this frame 
		bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
