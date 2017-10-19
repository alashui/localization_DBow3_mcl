#include "localization/pose_estimation.h"
#include "localization/image_retrieve.h"
#include "localization/config.h"


int main( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: parameter_file!"<<endl;
        return 1;
    }
    
    
	cout<<"initializing... "<<endl;
    localization::Config::setParameterFile ( argv[1] );    
    string map_dir = localization::Config::get<string> ( "map_dir" );
    string image_query_dir = localization::Config::get<string> ( "image_query_dir" );
    
    
	localization::PoseEstimation pose_estimation;    
	pose_estimation.mapInitialization();		//初始化地图
	//pose_estimation.map_->save(map_dir);
	//pose_estimation.map_->load(map_dir);

       
    //localization::ImageRetrieve::Ptr image_retrieve (new localization::ImageRetrieve );
    localization::ImageRetrieve image_retrieve;
    image_retrieve.map_ = pose_estimation.map_;  //数据库为地图中的关键帧
    image_retrieve.databaseInit(); 
    int num_result=10;
    image_retrieve.setResultNum(num_result);
    
    cout<<"initialization complete. "<<endl;
               
    //for ( int i=0; i<1; i++ )
    //{	 
        //image_retrieve.frame_query_=localization::Frame::createFrame();//不需要这句,frame_query_已经构造过
     //   string color_path = image_query_dir+"/"+to_string(i+1)+".png"; 
     //   string depth_path = image_query_dir+"/"+to_string(i+1)+".png"; 
     
        string color_path = image_query_dir+"/rgb452.png"; 
        string depth_path = image_query_dir+"/depth452.png";                   
        image_retrieve.frame_query_->color_ = imread(color_path);
        //image_retrieve->frame_query_->depth_ = imread(depth_path);   
        image_retrieve.frame_query_->extractKeyPoints();
        image_retrieve.frame_query_->computeDescriptors();
        
     	     	
     	
        image_retrieve.retrieve_result();         
    //    cout<<"searching for image "<<i<<" returns "<< num_result <<" results" <<endl;
        cout<<"searching for image_query "<<" returns "<< num_result <<" results" <<endl;
        
        pose_estimation.map_->map_points_.clear();	
        for(int k=0; k<num_result; k++)
        {
        	
        	int similar_frame_id = image_retrieve.EntryId_frame_id_[image_retrieve.result_[k].Id];
        	
        	
		    cout <<"entry_id: " << image_retrieve.result_[k].Id 
		    	 <<" frame_id: " << similar_frame_id
		    	 <<" score: "  << image_retrieve.result_[k].Score 
		    	 <<endl;
		                    
		                    			 				
			pose_estimation.curr_= image_retrieve.frame_query_;
			pose_estimation.ref_= pose_estimation.map_->keyframes_[ similar_frame_id ];
			

			//估计当前帧与参考帧的运动,3d点为参考帧上产生的地图点,2d点为当前帧的像素点
						
			for ( size_t i=0; i < pose_estimation.ref_->keypoints_.size(); i++ )   //计算参考帧上产生的地图点
		    {
		    	
		        double d = pose_estimation.ref_->findDepth ( pose_estimation.ref_->keypoints_[i] );
		        if ( d < 0 ) 
		            continue;
		        Vector3d p_world = pose_estimation.ref_->camera_->pixel2world (
		        	Vector2d ( pose_estimation.ref_->keypoints_[i].pt.x, pose_estimation.ref_->keypoints_[i].pt.y ), 
		        		pose_estimation.ref_->T_c_w_, d   );
		            		            	            		                      
		        Vector3d n = p_world - pose_estimation.ref_->getCamCenter();
		        n.normalize();
		        localization::MapPoint::Ptr map_point =localization::MapPoint::createMapPoint(  p_world, n,
		          				 pose_estimation.ref_->descriptors_.row(i).clone(), pose_estimation.ref_.get()    );
		        pose_estimation.map_->insertMapPoint( map_point ); 
		        							//计算这一参考帧产生的地图点,其实跟地图没有关系,只是计算它的三维点	   		       		        
		    }			
			pose_estimation.featureMatching(pose_estimation.curr_, pose_estimation.ref_);
			pose_estimation.poseEstimationPnP();
			
			pose_estimation.map_->map_points_.clear();	//清除地图点
		
		}
																			      
    //}


}
