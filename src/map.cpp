#include "localization/map.h"

namespace localization
{

void Map::insertKeyFrame ( Frame::Ptr frame )
{
    cout<<"insertKeyFrame"<<frame->id_<<endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
    }
}

void Map::addKeyFrame(Frame::Ptr frame)
{
    if ( keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for ( size_t i=0; i < frame->keypoints_.size(); i++ )
        {
            double d = frame->findDepth ( frame->keypoints_[i] );
            if ( d < 0 ) 
                continue;
            Vector3d p_world = frame->camera_->pixel2world (
                Vector2d ( frame->keypoints_[i].pt.x, frame->keypoints_[i].pt.y ), frame->T_c_w_, d   );          
            Vector3d n = p_world - frame->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(  p_world, n,
              				 frame->descriptors_.row(i).clone(), frame.get()    );
            insertMapPoint( map_point );
        }
    }
    
    insertKeyFrame ( frame );
}

void Map::addMapPoints( const Frame::Ptr frame,
						const vector<int> match_2dkp_index_)
{
    // add the new map points into map
    vector<bool> matched(frame->keypoints_.size(), false); 
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i < frame->keypoints_.size(); i++ )
    {
        if ( matched[i] == true )   //匹配过得点之前已经添加过,只添加未匹配上的特征点
            continue;
        double d = frame->findDepth ( frame->keypoints_[i] );
        if ( d < 0 ) 
            continue;
        Vector3d p_world = frame->camera_->pixel2world (
            Vector2d ( frame->keypoints_[i].pt.x, frame->keypoints_[i].pt.y ), frame->T_c_w_, d   );
        
        Vector3d n = p_world - frame->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(   p_world, n,
            				      frame->descriptors_.row(i).clone(), frame.get()  );        
        insertMapPoint( map_point );
    }
}


void Map::save(string &filename)    //将map的实例保存到文件(只保存重要的数据)                 
{
	cout<<"map save..."<<endl;		

	cv::FileStorage fs(filename, FileStorage::WRITE);    //保存为yml格式
	//采用cv::mat的方式存储mappoint数据量超大	,只存keyframes数据
	
	//存储map_points_
	vector<int>point_id_all_vec;
	for ( auto iter =map_points_.begin(); iter !=map_points_.end();iter++ )
	{		
		point_id_all_vec.push_back(iter->second->id_);
		fs << "mappoint_id_"+ to_string(iter->second->id_) <<iter->second->id_;
		
		cv::Mat cvMat_pos_=(Mat_<double>(3, 1) <<iter->second->pos_[0],iter->second->pos_[1],iter->second->pos_[2]);//欧式矩阵转换为cv::Mat
		fs << "mappoint_pos_" + to_string(iter->second->id_) << cvMat_pos_;
		
		cv::Mat cvMat_norm_=(Mat_<double>(3, 1) <<iter->second->norm_[0],iter->second->norm_[1],iter->second->norm_[2]);
		fs << "mappoint_norm_" + to_string(iter->second->id_) << cvMat_norm_;
		
		fs << "mappoint_descriptors_" + to_string(iter->second->id_) << iter->second->descriptor_;
	}
	cv::Mat cvMat_point_id(point_id_all_vec);
	fs << "mappoint_id_all"<<cvMat_point_id;
	
	
	
	//存储keyframes_
	vector<int>frame_id_all_vec;
	for ( auto iter =keyframes_.begin(); iter !=keyframes_.end(); iter++)
	{
		cout<<"save map->keyframes_->frame "<<iter->second->id_<<endl;
		frame_id_all_vec.push_back(iter->second->id_);
		 //存储frame_id
		fs << "frame_id_" + to_string(iter->second->id_) << iter->second->id_;
		
		 //存储相机位姿
		Eigen::MatrixXd matrix4d_T_c_w_(4,4);
		matrix4d_T_c_w_= iter->second->T_c_w_.matrix();  //sophus::SE3转换为欧式矩阵
		cv::Mat cvMat_T_c_w_=(Mat_<double>(4, 4) <<  matrix4d_T_c_w_(0,0),matrix4d_T_c_w_(0,1),matrix4d_T_c_w_(0,2),matrix4d_T_c_w_(0,3),
													 matrix4d_T_c_w_(1,0),matrix4d_T_c_w_(1,1),matrix4d_T_c_w_(1,2),matrix4d_T_c_w_(1,3),
													 matrix4d_T_c_w_(2,0),matrix4d_T_c_w_(2,1),matrix4d_T_c_w_(2,2),matrix4d_T_c_w_(2,3),
													 matrix4d_T_c_w_(3,0),matrix4d_T_c_w_(3,1),matrix4d_T_c_w_(3,2),matrix4d_T_c_w_(3,3)  );
													 //欧式矩阵转换为cv::Mat
		fs << "frame_T_c_w_" + to_string(iter->second->id_) << cvMat_T_c_w_;  
		
		//存储关键点
		std::vector<cv::Point2f> points;
		std::vector<cv::KeyPoint>::iterator it;
		for( it= iter->second->keypoints_.begin(); it!= iter->second->keypoints_.end();it++)
		{
			points.push_back(it->pt);
		}
		cv::Mat keypointmatrix(points);
		fs << "frame_keypoints_" + to_string(iter->second->id_) << keypointmatrix;

		//存储描述子
		fs << "frame_descriptors_" + to_string(iter->second->id_) << iter->second->descriptors_;
		
	}
	cv::Mat cvMat_frame_id(frame_id_all_vec);
	fs << "frame_id_all"<<cvMat_frame_id;
	
	
	
	fs.release();
	cout<<"map data save in"<<filename<<endl;	
	cout<<"map save done. "<<endl;			
}



void Map::load(string &filename)	//读取太慢，，放弃。	
{

	cout<<"map load..."<<endl;	

	cv::FileStorage fs(filename,FileStorage::READ);  
    //fs.open(filename,FileStorage::READ);  
cout <<"db 00" <<endl;    
	// if failed to open a file  
    if(!fs.isOpened())
    {  
        cout<<"failed to open file "<< filename <<endl;          
    }  
	else
	{	
cout <<"db 0" <<endl;	
		//读入map_points_				
		cv::Mat cvMat_point_id;
cout <<"db 1" <<endl;
		fs["mappoint_id_all"] >> cvMat_point_id;
cout <<"db 2" <<endl;
		for (int i=0;i<=cvMat_point_id.rows;i++)
		{
			MapPoint::Ptr map_point( new MapPoint() );
			map_point->id_ = cvMat_point_id.at<int>(i,0);
cout <<"db 3" <<endl;			
			cv::Mat cvMat_point_pos;
			fs["mappoint_pos_"+ to_string(map_point->id_)] >> cvMat_point_pos;
			cout <<cvMat_point_pos <<endl;
		
		}
		
		
	
		cout<<"map load done. "<<endl;				
	}
	fs.release();		
		
}		
		
/*		
		    // read data using operator []  
        cv::FileStorage fs("test.yml", FileStorage::READ);  
        int width;  
        int height;  
        fs["imageWidth"]>>width;  
        fs["imageHeight"]>>height;  
        cout<<"width readed = "<<width<<endl;  
        cout<<"height readed = "<<height<<endl<<endl;  
      
    // read Mat  
        cv::Mat resultMatRead;  
        fs["resultMat"]>>resultMatRead;  
        cout<<"resultMat readed = "<<resultMatRead<<endl<<endl;  
          
        cv::Mat cameraMatrixRead,distCoeffsRead;  
        fs["cameraMatrix"]>>cameraMatrixRead;  
        fs["distCoeffs"]>>distCoeffsRead;  
        cout<<"cameraMatrix readed = "<<cameraMatrixRead<<endl;  
        cout<<"distCoeffs readed = "<<distCoeffsRead<<endl<<endl;  
      
    // read string  
        string timeRead;  
        fs["calibrationDate"]>>timeRead;  
        cout<<"calibrationDate readed = "<<timeRead<<endl<<endl;  
      
        fs.release();  
        
        
		
		
		
		for ( auto iter =map_points_.begin(); iter !=map_points_.end();iter++ )
		{
			//cv::write(fs,"mappoint_id_", iter->second->id_);
	
			fs << "mappoint_id_" <<iter->second->id_;
		
			cv::Mat cvMat_pos_=(Mat_<double>(3, 1) <<iter->second->pos_[0],iter->second->pos_[1],iter->second->pos_[2]);//欧式矩阵转换为cv::Mat
			fs << "mappoint_pos_" << cvMat_pos_;
		
			cv::Mat cvMat_norm_=(Mat_<double>(3, 1) <<iter->second->norm_[0],iter->second->norm_[1],iter->second->norm_[2]);
			fs << "mappoint_norm_" << cvMat_norm_;
		
			fs << "mappoint_descriptors_" << iter->second->descriptor_;
		}
	
		//存储keyframes_
		for ( auto iter =keyframes_.begin(); iter !=keyframes_.end(); iter++)
		{
			cout<<"save map->keyframes_->frame "<<iter->second->id_<<endl;
		
			 //存储frame_id
			fs << "frame_id_" << iter->second->id_;
		
			 //存储相机位姿
			Eigen::MatrixXd matrix4d_T_c_w_(4,4);
			matrix4d_T_c_w_= iter->second->T_c_w_.matrix();  //sophus::SE3转换为欧式矩阵
			cv::Mat cvMat_T_c_w_=(Mat_<double>(4, 4) <<  matrix4d_T_c_w_(0,0),matrix4d_T_c_w_(0,1),matrix4d_T_c_w_(0,2),matrix4d_T_c_w_(0,3),
														 matrix4d_T_c_w_(1,0),matrix4d_T_c_w_(1,1),matrix4d_T_c_w_(1,2),matrix4d_T_c_w_(1,3),
														 matrix4d_T_c_w_(2,0),matrix4d_T_c_w_(2,1),matrix4d_T_c_w_(2,2),matrix4d_T_c_w_(2,3),
														 matrix4d_T_c_w_(3,0),matrix4d_T_c_w_(3,1),matrix4d_T_c_w_(3,2),matrix4d_T_c_w_(3,3)  );
														 //欧式矩阵转换为cv::Mat
			fs << "frame_T_c_w_" << cvMat_T_c_w_;  
		
			//存储关键点
			std::vector<cv::Point2f> points;
			std::vector<cv::KeyPoint>::iterator it;
			for( it= iter->second->keypoints_.begin(); it!= iter->second->keypoints_.end();it++)
			{
				points.push_back(it->pt);
			}
			cv::Mat keypointmatrix(points);
			fs << "frame_keypoints_" << keypointmatrix;

			//存储描述子
			fs << "frame_descriptors_" << iter->second->descriptors_;
*/		









}
