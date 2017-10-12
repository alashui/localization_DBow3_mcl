#include "localization/image_retrieve.h"
namespace localization
{

	ImageRetrieve::ImageRetrieve()
	{
		string database_dir = localization::Config::get<string> ( "database_dir" );
	    database_.load( database_dir );

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
		string vocabulary_dir = localization::Config::get<string> ( "vocabulary_dir" );
		string database_dir = localization::Config::get<string> ( "database_dir" );
		string image_database_dir = localization::Config::get<string> ( "image_database_dir" );
		
		// read the vocabulary
		cout <<"reading vocabulary"<< endl;
		DBoW3::Vocabulary vocab( vocabulary_dir );
   	 	if ( vocab.empty() )
   	 	{
    	    cerr<<"Vocabulary does not exist."<<endl;
    	}
		//读取需要添加进数据库的图像,提取特征并计算描述子
		
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

	   //生成数据库
		DBoW3::Database db( vocab, true, 0);
		for ( int i=0; i<descriptors.size(); i++ )
		    db.add(descriptors[i]);
		db.save(database_dir );

		cout<<"database info: "<<db<<endl;
		cout<<"database save in" << database_dir <<endl;
		cout<<"the database has been established."<<endl;
		
		database_=db;
		
	}
	
	
	void ImageRetrieve::retrieve_result()
	{
		switch (state_)
		{
			case EMPTY:
			{
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






/*
#include "localization/common_include.h"

#include "DBoW3/DBoW3.h"
#include "localization/config.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: parameter_file!"<<endl;
        return 1;
    }

    localization::Config::setParameterFile ( argv[1] );    
    string project_dir = localization::Config::get<string> ( "project_dir" );
    //string vocabulary_dir = localization::Config::get<string> ( "vocabulary_dir" );
    string database_dir = localization::Config::get<string> ( "database_dir" );
    string image_query_dir = localization::Config::get<string> ( "image_query_dir" );

    DBoW3::Database db( database_dir );


    if ( db.size()==0 )
    {
        cerr<<"database does not exist (or it`s a empty database)."<<endl;
        return 1;
    }
    cout<<"database info: "<<db<<endl;


    //读取待查询的图片,提取特征并计算描述子
    cout<<"reading images "<<endl;
    vector<Mat> images; 
    for ( int i=0; i<1; i++ )
    {
        string path = image_query_dir+"/"+to_string(i+1)+".png";
        images.push_back( imread(path) );
    }

    cout<<"detecting ORB features ... "<<endl;
    Ptr< Feature2D > detector = ORB::create();
    vector<Mat> descriptors;
    for ( Mat& image:images )
    {
        vector<KeyPoint> keypoints; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
    }


    // 在数据库中检索与待查询图像相似度最高的几帧图像,并计算得分
    cout<<"comparing images with database "<<endl;
    
    for ( int i=0; i<descriptors.size(); i++ )
    {
        DBoW3::QueryResults ret;
        db.query( descriptors[i], ret, 20);      // max result=4
        cout<<"searching for image "<<i<<" returns "<<ret<<endl;
    }
    cout<<"done."<<endl;

}
*/

