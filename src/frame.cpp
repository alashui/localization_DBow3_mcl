#include "localization/frame.h"
#include <boost/timer.hpp>


namespace localization
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(new Camera), is_key_frame_(false)
{
	num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
{
	num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long id_ = 0;
    return Frame::Ptr( new Frame(id_++) );
}

void Frame::extractKeyPoints()
{
	boost::timer timer;
	orb_->detect ( color_, keypoints_ );
	cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}			
void Frame::computeDescriptors()
{
	boost::timer timer;
	orb_->compute ( color_, keypoints_, descriptors_ );
	cout<<"extract computeDescriptors cost time: "<<timer.elapsed() <<endl;
}




double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {    	
		return double(d)/camera_->depth_scale_;
    }

    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }    
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;  
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ ); 
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}

}
