#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "localization/config.h"
#include "localization/visual_odometry.h"
#include "localization/g2o_types.h"

namespace localization
{

PoseEstimation::PoseEstimation() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{

    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
    
}

PoseEstimation::~PoseEstimation()
{

}
//匹配当前帧和参考帧,实际处理是匹配当前帧的特征点和出现在参考帧中的地图点
void PoseEstimation::featureMatching(const Frame::Ptr frame_curr,
									 const Frame::Ptr frame_ref)
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // select the candidates in map 
    Mat desp_map;
    vector<MapPoint::Ptr> candidate;
    for ( auto& allpoints: map_->map_points_ )  //筛选出属于参考帧的地图点
    {
        MapPoint::Ptr& p = allpoints.second;
        // check if p in curr frame image 
        if ( frame_ref->isInFrame(p->pos_) )
        {
            // add to candidate 
            p->visible_times_++;
            candidate.push_back( p );
            desp_map.push_back( p->descriptor_ );
        }
    }
    
    matcher_flann_.match ( desp_map, frame_curr->descriptors_, matches );
    // select the best matches        
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;
    

    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back( candidate[m.queryIdx] );
            match_2dkp_index_.push_back( m.trainIdx );
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}

void PoseEstimation::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for ( int index:match_2dkp_index_ )
    {
        pts2d.push_back ( curr_->keypoints_[index].pt );
    }
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        pts3d.push_back( pt->getPositionCV() );
    }

    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_w_estimated_ = SE3 (
                           SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                       );

    // using bundle adjustment to optimize the pose
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
    ));
    optimizer.addVertex ( pose );

    // edges
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int> ( i,0 );
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId ( i );
        edge->setVertex ( 0, pose );
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        // set the inlier map points 
        match_3dpts_[index]->matched_times_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );

    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    
    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
}

bool PoseEstimation::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

void PoseEstimation::optimizeMap()
{
    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double PoseEstimation::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}

void PoseEstimation::mapInitialization()
{
    string map_dir_ = localization::Config::get<string> ( "map_dir_" );
	map_->load(map_dir_);
	if(map_->state_==EMPTY)
	{
		string image_database_dir = localization::Config::get<string> ( "image_database_dir" );
		//string dir_frames_rgb = localization::Config::get<string> ( "dir_frames_rgb_" );    
		//string dir_frames_depth = localization::Config::get<string> ( "dir_frames_depth_" );	
		string dir_pose = localization::Config::get<string> ( "dir_pose_" );			
		
		const char* dir_pose_cstr = dir_pose.c_str();  //ifstream fin(const char*)
	    ifstream fin(dir_pose_cstr);	    	//数据格式:frame_id              double[12]
	    										//		(数字1,2,3,...)对应帧id ;	R(3x3) t(3x1) 表示该帧的相机位姿     
		if (!fin)								
		{
		    cerr<<"cannot find pose file"<<endl;
		    //return 1;
		}
		else//文件存在
		{
		   vector<Frame::Ptr> frame_ptr_vec;
		   //读取所有的关键帧及每一帧相机位姿	
		   while(getline(fin,temp))		//获取一行,一行代表一条位姿记录及对应关键帧id
		   {
		   		//对每一行数据的读入一个frame对象
		   		Frame::Ptr frame(new Frame());
		   		vector<double> double_vec; 
		   		double num;
				istringstream iss(temp);
				iss >> frame->id_;  	//帧id
				
				string rgb_dir = image_database_dir+"/rgb/"+to_string(frame->id_)+".png";
				string depth_dir = image_database_dir+"/depth/"+to_string(frame->id_)+".png";                       
       		    frame->color_ = imread(color_dir);
       		    frame->depth_ = imread(depth_dir);   
        		frame->extractKeyPoints();
        		frame->computeDescriptors();
								
				while(iss >> num)  		//分别将这一行数据读入						
					double_vec.push_back(num);			
				Eigen::Matrix3d R;
				Eigen::Vector3d t;				
				R(0,0)=double_vec[0];  R(0,1)=double_vec[1];  R(0,2)=double_vec[2];  t(0)=double_vec[3];
				R(1,0)=double_vec[4];  R(1,1)=double_vec[5];  R(1,2)=double_vec[6];  t(1)=double_vec[7];
				R(2,0)=double_vec[8];  R(2,1)=double_vec[9];  R(2,2)=double_vec[10]; t(2)=double_vec[11];
				Sophus::SE3 T(R,t);
				frame->T_c_w_ = T;
				
				frame_ptr_vec.push_back(frame);																
		   }
		   Frame::Ptr frame_cur,frame_ref;
		   for (Frame::Ptr frame : frame_ptr_vec)
		   {
		   		if(map_->state_==EMPTY)
		   		{
		   			map_->addKeyFrame(frame);	//第一帧,添加所有关键点为地图点(函数addKeyFrame有处理)
		   			map_->state_=EXSIT;
		   			frame_cur=frame_ref=frame;
		   		}
		   		else
		   		{
		   			frame_cur=frame;
		   			featureMatching(frame_cur, frame_ref);
		   			map_->addKeyFrame(frame);
		   			map_->addMapPoints( frame,match_2dkp_index_);
		   		}
		   		
		   }		   
		}           	
	}	
}

}
