#include "localization/common_include.h"
#include "localization/pose_estimation.h"
#include "localization/image_retrieve.h"
#include "localization/config.h"

#include<opencv2/opencv.hpp>
#include <math.h>
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
       
    //localization::ImageRetrieve::Ptr image_retrieve (new localization::ImageRetrieve );
    localization::ImageRetrieve image_retrieve;
    image_retrieve.map_ = pose_estimation.map_;  //数据库为地图中的关键帧
    image_retrieve.databaseInit(); 
    int num_result=20;
    image_retrieve.setResultNum(num_result);
    
    cout<<"initialization complete. "<<endl;
               
    //for ( int i=0; i<1; i++ )
    //{	 
        //image_retrieve.frame_query_=localization::Frame::createFrame();//不需要这句,frame_query_已经构造过
     //   string color_path = image_query_dir+"/"+to_string(i+1)+".png"; 
     //   string depth_path = image_query_dir+"/"+to_string(i+1)+".png"; 
     
        string color_path = image_query_dir+"/rgb260.png"; 
        string depth_path = image_query_dir+"/depth260.png";                   
        image_retrieve.frame_query_->color_ = imread(color_path);
        image_retrieve.frame_query_->depth_ = imread(depth_path);   
        image_retrieve.frame_query_->extractKeyPoints();
        image_retrieve.frame_query_->computeDescriptors();
        
        /*
		Mat picture(1000,1000,CV_8UC3,Scalar(255,255,255));      	     	
		Point center = Point(0,0); */

      	
        image_retrieve.retrieve_result();         
    //    cout<<"searching for image "<<i<<" returns "<< num_result <<" results" <<endl;
        cout<<"searching for image_query "<<" returns "<< num_result <<" results" <<endl;
        
        pose_estimation.map_->map_points_.clear();
        
        class PoseResult{public:double score, x, y,theta;int frame_id;bool state;};  //表示相机位姿
        
        
        
        vector<PoseResult>pose_result_vec;
        	
        for(int k=0; k<num_result; k++)
        {
        	PoseResult pose_result;
        	SE3 T_temp;
        	pose_estimation.T_c_w_estimated_ = T_temp;
        	
        	pose_result.frame_id = image_retrieve.EntryId_frame_id_[image_retrieve.result_[k].Id];        	
        	pose_result.score = image_retrieve.result_[k].Score;
        	
		    cout <<"result " << k 
		    	 //<<" entry_id: " << image_retrieve.result_[k].Id 
		    	 <<"   frame_id: " << pose_result.frame_id
		    	 <<"   score: "  << pose_result.score 
		    	 <<endl;
		   
		   
		    Mat K = ( Mat_<double> ( 3,3 ) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1 );                 
		                    			 				
			pose_estimation.curr_= image_retrieve.frame_query_;
			pose_estimation.ref_= pose_estimation.map_->keyframes_[ pose_result.frame_id ];
			

			//估计当前帧与参考帧的运动,3d点为参考帧上产生的地图点,2d点为当前帧的像素点
						
			for ( size_t i=0; i < pose_estimation.ref_->keypoints_.size(); i++ )   //计算参考帧上产生的地图点
		    {
		    	
		        double d = pose_estimation.ref_->findDepth ( pose_estimation.ref_->keypoints_[i] );
		        if ( d < 0 ) 
		            continue;
		        Vector3d p_world = pose_estimation.ref_->camera_->pixel2world (
		        	Vector2d ( pose_estimation.ref_->keypoints_[i].pt.x, pose_estimation.ref_->keypoints_[i].pt.y ), 
		        		pose_estimation.ref_->T_c_w_, d   );


		                      
		        Vector3d n;// = p_world - pose_estimation.ref_->getCamCenter();
		        n.normalize();
		        localization::MapPoint::Ptr map_point =localization::MapPoint::createMapPoint(  p_world, n,
		          				 pose_estimation.ref_->descriptors_.row(i).clone(), pose_estimation.ref_.get()    );
		        pose_estimation.map_->insertMapPoint( map_point ); 
		        							//计算这一参考帧产生的地图点,其实跟地图没有关系,只是计算它的三维点
		    }	
		    	   		
			if ( pose_estimation.featureMatching(pose_estimation.curr_, pose_estimation.ref_) ) //匹配成功才做运动估计
			{
				pose_estimation.poseEstimationPnP();
				pose_result.state = pose_estimation.checkEstimatedPose();				
			}
					
			pose_estimation.map_->map_points_.clear();	//清除地图点
		
			Sophus::SE3 Twc = pose_estimation.T_c_w_estimated_.inverse();			
			cout <<"Twc"<<endl<<Twc.matrix() << endl <<endl <<endl;	
			
			
			pose_result.x= Twc.matrix()(0,3);
			pose_result.y= Twc.matrix()(2,3);
			
			pose_result.theta= acos(0.5*((Twc.matrix()(0,0)+Twc.matrix()(1,1)+Twc.matrix()(2,2))-1)) ;
			pose_result_vec.push_back(pose_result);
			/*
			center.x =500+Twc.matrix()(0,3)*10;
			center.y =500+Twc.matrix()(2,3)*10;  
			cout <<"x:"<<center.x <<"  y:"<<center.y<<endl;
			//半径  
			int r = 10;  
			//承载图像  
			 
			//参数为：承载的图像、圆心、半径、颜色、粗细、线型  
			circle(picture,center,r,Scalar(0,0,250));  
			*/
			
										
		}
		
		cout <<"result"<<endl;
		for(int k=0; k<num_result; k++)
		{
			
			cout << "result " << k << ":   " 
				 << "frame_id: " << pose_result_vec[k].frame_id << "   "
				 << " score: "  << pose_result_vec[k].score << "   "
				 << "x:"<< pose_result_vec[k].x << "   "
				 << "y:"<< pose_result_vec[k].y << "   "
				 << "theta:"<<pose_result_vec[k].theta <<"   "
				 << "state:"<<pose_result_vec[k].state << endl;
		}
		
		
		
		
		/*imshow("pose",picture); 
		waitKey(0);*/																	      
    //}



}



/*

result for rgb300.png

result 0:    frame_id: 300    score: 1   		 x:13.9296   y:2.82265   theta:2.41964
result 1:    frame_id: 301    score: 0.102776    x:13.9793   y:2.78729   theta:2.42875
result 2:    frame_id: 299    score: 0.0693958   x:13.8875   y:2.86637   theta:2.41272
result 3:    frame_id: 302    score: 0.0536191   x:13.996    y:2.76473   theta:2.4303
result 4:    frame_id: 298    score: 0.0448145   x:112.629   y:-30.5906  theta:3.02878
result 5:    frame_id: 303    score: 0.0435787   x:14.9075   y:0.737955  theta:2.32366
result 6:    frame_id: 305    score: 0.037726    x:13.9842   y:2.84597   theta:2.43421
result 7:    frame_id: 304    score: 0.0310616   x:13.9858   y:2.81322   theta:2.43141
result 8:    frame_id: 306    score: 0.0286385   x:13.8923   y:2.84655   theta:2.40503
result 9:    frame_id: 307    score: 0.0264989   x:13.8643   y:2.86456   theta:2.3985
result 10:   frame_id: 308    score: 0.0244973   x:13.8881   y:2.82965   theta:2.40215
result 11:   frame_id: 297    score: 0.0135155   x:6.565     y:8.81041   theta:2.74595
result 12:   frame_id: 256    score: 0.0111524   x:10509.5   y:3854.42   theta:1.48
result 13:   frame_id: 311    score: 0.0108131   x:9.89627   y:0.878723  theta:1.96324
result 14:   frame_id: 110    score: 0.0106924   x:9.26337   y:16.8362   theta:0.807775
result 15:   frame_id: 317    score: 0.00989605  x:34147.5   y:57296.7   theta:2.33352
result 16:   frame_id: 340    score: 0.00939412  x:7.00126   y:-2.26659  theta:2.67488
result 17:   frame_id: 270    score: 0.00890024  x:16.3584   y:0.0297195 theta:2.63166
result 18:   frame_id: 370    score: 0.00875891  x:-20.455   y:12.4545   theta:3.06371
result 19:   frame_id: 267    score: 0.00873694  x:13.6721   y:-1.4608   theta:2.1875




result for rgb452.png

result 0:    frame_id: 452    score: 1           x:0.408557   	y:-2.40073   theta:0.782908
result 1:    frame_id: 453    score: 0.108716    x:0.356602   	y:-2.41706   theta:0.77187
result 2:    frame_id: 451    score: 0.106872    x:0.482839   	y:-2.3951    theta:0.796882
result 3:    frame_id: 454    score: 0.0810632   x:0.239636   	y:-2.41744   theta:0.748581
result 4:    frame_id: 450    score: 0.0662164   x:-4.04426   	y:4.84234    theta:3.10194
result 5:    frame_id: 455    score: 0.0624233   x:0.186706   	y:-2.44466   theta:0.735096
result 6:    frame_id: 449    score: 0.0553199   x:-2.57991   	y:5.7375     theta:3.11677
result 7:    frame_id: 456    score: 0.0416935   x:-5.16213   	y:-1.73001   theta:3.0485
result 8:    frame_id: 458    score: 0.0310182   x:-6.29236   	y:-1.30025   theta:3.06427
result 9:    frame_id: 448    score: 0.0299908   x:5.90433    	y:2.05365    theta:2.68338
result 10:   frame_id: 457    score: 0.0251521   x:-4.79126   	y:-2.17759   theta:2.94126
result 11:   frame_id: 461    score: 0.0208819   x:-10.8196   	y:-18.6922   theta:3.10637
result 12:   frame_id: 463    score: 0.0190736   x:1.74737    	y:-1.28862   theta:2.87178
result 13:   frame_id: 459    score: 0.0187747   x:24.6494    	y:16.2173    theta:1.99607
result 14:   frame_id: 460    score: 0.0185662   x:-2.16e+06    y:-1.591e+06 theta:2.45028
result 15:   frame_id: 462    score: 0.0173106   x:0.297174   	y:-2.19306   theta:0.779233
result 16:   frame_id: 447    score: 0.0129455   x:-1.36563   	y:1.73796    theta:2.05446
result 17:   frame_id: 110    score: 0.0110651   x:-67.6046   	y:-40.2864   theta:2.94448
result 18:   frame_id: 4      score: 0.0109389   x:-0.0288433   y:0.0145766  theta:1.09768
result 19:   frame_id: 127    score: 0.0104159   x:3.29576   	y:5.21191    theta:1.43948



result for rgb260.png 

result 0:    frame_id: 260    score: 1           x:9.88652   y:3.61278   theta:2.72927
result 1:    frame_id: 261    score: 0.0864794   x:10.3216   y:3.01969   theta:2.73475
result 2:    frame_id: 258    score: 0.0818935   x:10.3458   y:2.96634   theta:2.73824
result 3:    frame_id: 259    score: 0.0803074   x:10.3559   y:3.01768   theta:2.7419
result 4:    frame_id: 256    score: 0.0754513   x:10.3826   y:2.97796   theta:2.74815
result 5:    frame_id: 253    score: 0.0697977   x:10.3326   y:2.98229   theta:2.7439
result 6:    frame_id: 257    score: 0.0630406   x:10.3628   y:2.96788   theta:2.74328
result 7:    frame_id: 262    score: 0.061544    x:10.3165   y:3.0371    theta:2.73522
result 8:    frame_id: 254    score: 0.0599786   x:10.435    y:3.02145   theta:2.76374
result 9:    frame_id: 252    score: 0.0598985   x:10.4017   y:3.0538    theta:2.76079
result 10:   frame_id: 251    score: 0.0540477   x:10.3529   y:3.06178   theta:2.75177
result 11:   frame_id: 255    score: 0.05008     x:10.3326   y:2.9465    theta:2.73857
result 12:   frame_id: 263    score: 0.0360672   x:10.3219   y:3.04862   theta:2.73592
result 13:   frame_id: 264    score: 0.0319114   x:10.2862   y:3.08902   theta:2.73475
result 14:   frame_id: 249    score: 0.0304109   x:10.3726   y:3.08206   theta:2.7598
result 15:   frame_id: 248    score: 0.0262705   x:10.3619   y:3.05817   theta:2.75758
result 16:   frame_id: 265    score: 0.0260094   x:10.2952   y:3.09771   theta:2.73509
result 17:   frame_id: 250    score: 0.0258338   x:10.3628   y:3.01612   theta:2.75198
result 18:   frame_id: 247    score: 0.0215262   x:22531.4   y:1601.53   theta:1.60017
result 19:   frame_id: 267    score: 0.0186977   x:9.99037   y:2.99555   theta:2.68821




result  for rgb2.png 

result 0:   frame_id: 1    score: 0.144893   x:0.000482244   y:0.0234564   theta:0.028621
result 1:   frame_id: 477    score: 0.11729   x:0.0378861   y:0.0749989   theta:0.0359883
result 2:   frame_id: 475    score: 0.108709   x:0.0195725   y:0.00706001   theta:0.0421862
result 3:   frame_id: 4    score: 0.108444   x:0.0272674   y:0.0527278   theta:0.0332201
result 4:   frame_id: 3    score: 0.107565   x:0.0118323   y:0.0450191   theta:0.0297721
result 5:   frame_id: 478    score: 0.0983268   x:0.0470925   y:0.0818096   theta:0.0318857
result 6:   frame_id: 476    score: 0.0972937   x:0.0217826   y:0.00455701   theta:0.0367172
result 7:   frame_id: 6    score: 0.0917573   x:-1527.85   y:-9417.39   theta:2.73811
result 8:   frame_id: 474    score: 0.0913623   x:-0.0314661   y:0.00241921   theta:0.0312285
result 9:   frame_id: 479    score: 0.0876875   x:-1.22341   y:8.21544   theta:3.1272
result 10:   frame_id: 5    score: 0.0851114   x:0.0177218   y:0.0872163   theta:0.0342836
result 11:   frame_id: 482    score: 0.0825751   x:-1.21187   y:8.55739   theta:3.13638
result 12:   frame_id: 480    score: 0.073984   x:-1.26003   y:8.1173   theta:3.12611
result 13:   frame_id: 471    score: 0.0738817   x:0.0141057   y:-0.0605316   theta:0.0504018
result 14:   frame_id: 7    score: 0.0703707   x:0.26556   y:-0.159435   theta:0.0898385
result 15:   frame_id: 481    score: 0.0679087   x:0.885976   y:9.03687   theta:3.12507
result 16:   frame_id: 469    score: 0.0671866   x:-323377   y:-353985   theta:0.803534
result 17:   frame_id: 462    score: 0.0662981   x:0.0528013   y:-0.569841   theta:0.0689776
result 18:   frame_id: 8    score: 0.064611   x:0.206035   y:-0.144628   theta:0.0824043
result 19:   frame_id: 472    score: 0.0642401   x:86209.5   y:5929.47   theta:2.90297




result for rgb166.png 
result 0:   frame_id: 166    score: 1   x:5.93231   y:2.12083   theta:2.98002
result 1:   frame_id: 167    score: 0.0711954   x:5.89423   y:2.10534   theta:2.99095
result 2:   frame_id: 168    score: 0.05143   x:5.82774   y:2.02262   theta:3.00744
result 3:   frame_id: 162    score: 0.0433674   x:6.07323   y:2.24401   theta:2.94426
result 4:   frame_id: 172    score: 0.0425471   x:5.92175   y:1.8964   theta:2.98179
result 5:   frame_id: 163    score: 0.0415118   x:5.93876   y:2.29396   theta:2.98268
result 6:   frame_id: 169    score: 0.0397726   x:5.81286   y:2.00867   theta:3.01124
result 7:   frame_id: 161    score: 0.0377171   x:6.00542   y:2.44722   theta:2.97254
result 8:   frame_id: 164    score: 0.0367645   x:5.93752   y:2.26431   theta:2.98401
result 9:   frame_id: 170    score: 0.0272418   x:5.76781   y:1.92724   theta:3.02319
result 10:   frame_id: 173    score: 0.0248939   x:6.1099   y:2.30433   theta:2.96002
result 11:   frame_id: 158    score: 0.0214829   x:7.10162   y:-8.507   theta:2.89693
result 12:   frame_id: 174    score: 0.0213827   x:5.71859   y:2.4348   theta:3.02341
result 13:   frame_id: 157    score: 0.0207831   x:7.22937   y:-9.53613   theta:3.07671
result 14:   frame_id: 160    score: 0.0192237   x:5.8681   y:1.99376   theta:2.98528
result 15:   frame_id: 171    score: 0.0186299   x:5.75016   y:1.88338   theta:3.02816
result 16:   frame_id: 159    score: 0.0172483   x:5.17752   y:-9.54924   theta:3.04377
result 17:   frame_id: 156    score: 0.0139583   x:7.07287   y:-8.96258   theta:2.80645
result 18:   frame_id: 175    score: 0.0132031   x:5.64273   y:2.57497   theta:3.03079
result 19:   frame_id: 121    score: 0.0122519   x:0.802124   y:-1.1073   theta:1.27738


*/















