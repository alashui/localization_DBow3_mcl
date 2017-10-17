#include "localization/image_retrieve.h"
namespace localization
{

	ImageRetrieve::ImageRetrieve()
	{
		string database_dir = localization::Config::get<string> ( "database_dir" );		
	    database_.load( database_dir );
cout <<"db 1116" << endl;
    	if ( database_.size()==0 )
    	{
    	    state_ = EMPTY;   //数据库还未建立
   		}
   		else 
   		    state_ = EXIST;
	}
	
	
	ImageRetrieve::~ImageRetrieve(){}
        
	void ImageRetrieve::create_database()
	{
cout<<"db 0.1"<<endl;	
		string vocabulary_dir = localization::Config::get<string> ( "vocabulary_dir" );
		string database_dir = localization::Config::get<string> ( "database_dir" );
		string db_frame_id_dir = localization::Config::get<string> ( "db_frame_id_dir" );
		
		// read the vocabulary
		cout <<"reading vocabulary"<< endl;
		DBoW3::Vocabulary vocab( vocabulary_dir );
   	 	if ( vocab.empty() )
   	 	{
    	    cerr<<"Vocabulary does not exist."<<endl;
    	}
		//读取需要添加进数据库的图像,提取特征并计算描述子

		//直接从已生成的Map实例中获取
cout<<"db 0.2"<<endl;				
	   	//生成数据库
		DBoW3::Database db( vocab, true, 0);
		
		ofstream fout(db_frame_id_dir);
		for ( auto iter =map_->keyframes_.begin(); iter != map_->keyframes_.end(); iter++)
		{
			unsigned int EntryId = db.add((iter->second)->descriptors_);  //EntryId为这个描述子转换为词袋向量后存在database中的id
			EntryId_frame_id_.insert( make_pair(EntryId,iter->second->id_) );  
			//因为map->keyframes_是unordered_map类型，是无序的，所以这里将EntryId与frame_id_的对应关系保存
			fout << EntryId << " " << iter->second->id_ << endl;			
		}
		fout.close();
		//for ( int i=0; i<descriptors.size(); i++ )
		//    db.add(descriptors[i]);
		    		    
cout<<"db 0.3"<<endl;		    		    
		db.save(database_dir );

		cout<<"database info: "<<db<<endl;
		cout<<"database save in" << database_dir <<endl;
		cout<<"the database has been established."<<endl;
		
		database_=db;
		
		/*
		//从文件中读取，然后计算描述子
		string image_database_dir = localization::Config::get<string> ( "image_database_dir" );
		cout<<"reading images "<<endl;
		vector<Mat> images; 

		struct dirent *ptr;      
		DIR *dir;  
		dir=opendir((image_database_dir+"/rgb").c_str());   
		vector<string> dir_files;    
		while((ptr=readdir(dir))!=NULL)  
		{         
		    //跳过'.'和'..'两个目录  
		    if(ptr->d_name[0] == '.')  
		        continue;   
		    dir_files.push_back(ptr->d_name);  
		}                  
		closedir(dir);  

		for ( int i=0; i<dir_files.size(); i++ )
		{
		    string path = image_database_dir+"/rgb/rgb"+to_string(i+1)+".png";
		    images.push_back( imread(path) );
		}

		cout<<"detecting ORB features ... "<<endl;
		cv::Ptr< Feature2D > detector = ORB::create();
		vector<Mat> descriptors;
		int index = 1;
		for ( Mat& image:images )
		{
		    vector<KeyPoint> keypoints; 
		    Mat descriptor;
		    detector->detectAndCompute( image, Mat(), keypoints, descriptor );
		    descriptors.push_back( descriptor );
		    cout<<"extracting features from image " << index++ <<endl;
		}
		cout<<"extract total "<<descriptors.size()*500<<" features."<<endl;
		*/	
	}
	
	
	void ImageRetrieve::retrieve_result()
	{
		switch (state_)
		{
			case EMPTY:
			{
cout<<"db 0"<<endl;			
				cout<<"database does not exist (or it`s a empty database)."<<endl;
				cout<<"creating database now"<<endl;
				create_database();
				state_ = EXIST;
			}
			case EXIST:
			{
				cout<<"database existed,retrieving now"<<endl;
			
				// 在数据库中检索与待查询图像相似度最高的几帧图像,并计算得分
				cout<<"comparing image with database "<<endl;	
				database_.query( frame_query_->descriptors_, result_, num_result_);      						
			}
		
		}
	
	}
	
	
	
	
	

}




