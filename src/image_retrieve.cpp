#include "localization/image_retrieve.h"
namespace localization
{

	ImageRetrieve::ImageRetrieve():frame_query_(new Frame),map_(new Map) {}	
	ImageRetrieve::~ImageRetrieve(){}
        
	void ImageRetrieve::create_database()
	{
	
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
				
	   	//生成数据库
		//DBoW3::Database db( vocab, true, 0);
		database_.setVocabulary( vocab, true, 0);

		ofstream fout(db_frame_id_dir);	
		for ( auto iter = map_->keyframes_.begin(); iter != map_->keyframes_.end(); iter++)
		{			
			unsigned int EntryId = database_.add((iter->second)->descriptors_);  //EntryId为这个描述子转换为词袋向量后存在database中的id			
			EntryId_frame_id_.insert( make_pair(EntryId,iter->second->id_) ); 
			//因为map->keyframes_是unordered_map类型，是无序的，所以这里将EntryId与frame_id_的对应关系保存
			fout << EntryId << " " << iter->second->id_ << endl;		
		}
		fout.close();			   		    		    		    
		database_.save(database_dir );
			
		cout<<"database info: "<<database_<<endl;
		cout<<"database save in" << database_dir <<endl;
		cout<<"the database has been established."<<endl;
		
		/*
		//从文件中读取图像，然后计算描述子
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
	
	void ImageRetrieve::read_db_frame_id_()  //读取db_frame_id_.txt到 EntryId_frame_id_;
	{
		string db_frame_id_dir = localization::Config::get<string> ( "db_frame_id_dir" );			

	    ifstream fin(db_frame_id_dir,ios::in);	    		    								   
		if (!fin)								
		{
		    cerr<<"cannot find db_frame_id_.txt!"<<endl;
		    //return 1;
		}
		else//文件存在
		{

		   string temp;	

		   while(getline(fin,temp))		//获取一行,一行代表一条位姿记录及对应关键帧id
		   {
		   	
		   		//对每一行数据的读入unordered_map< unsigned int, int > EntryId_frame_id_;
		   		unsigned int EntryId;
		   		int frame_id;		   				   		
		   		vector<double> double_vec; 
		   		double num;	
		   		istringstream iss(temp);	
				while(iss >> num)  		//分别将这一行数据读入						
					double_vec.push_back(num);			
								
				EntryId=double_vec[0]; 
				frame_id=double_vec[1];	
				
				if ( EntryId_frame_id_.find(EntryId) == EntryId_frame_id_.end() )
					EntryId_frame_id_.insert( make_pair(EntryId, frame_id) );
				else			  
					EntryId_frame_id_[ EntryId] = frame_id;																			
		   }
		}
	}
	
	
	void ImageRetrieve::databaseInit()
	{
		cout<<"initializing database... "<<endl;
		string database_dir = localization::Config::get<string> ( "database_dir" );					    
		fstream fin;
		fin.open(database_dir,ios::in);
		if(!fin)
		{

		    cout<<"database does not exist "<<endl;
			cout<<"creating database now"<<endl;
			create_database();
		}
		else
		{
			cout<<"database file exist "<<endl;
		    cout<<"load database now..."<<endl;
		    database_.load( database_dir );
			    
			if ( database_.size()==0 )
			{			    
			    cout<<"it`s a empty database."<<endl;
				cout<<"creating database now"<<endl;
				create_database();
	   		}
	   		else 
	   		    read_db_frame_id_();
   		}		
	}
			
	void ImageRetrieve::retrieve_result()
	{	
		database_.query( frame_query_->descriptors_, result_, num_result_);
		cout<<"image retrieve done."<<endl;	
	}

}




