#include "localization/common_include.h"

#include "DBoW3/DBoW3.h" 
#include <dirent.h>   //用于遍历目录文件的库
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
    //string project_dir = localization::Config::get<string> ( "project_dir" );
    string vocabulary_dir = localization::Config::get<string> ( "vocabulary_dir" );
    string database_dir = localization::Config::get<string> ( "database_dir" );
    string image_database_dir = localization::Config::get<string> ( "image_database_dir" );

    //string image_query_dir = localization::Config::get<string> ( "image_query_dir" );

    // read the vocabulary
    cout <<"reading vocabulary"<< endl;
    DBoW3::Vocabulary vocab( vocabulary_dir );
    if ( vocab.empty() )
    {
        cerr<<"Vocabulary does not exist."<<endl;
        return 1;
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
    Ptr< Feature2D > detector = ORB::create();
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
    cout<<"done."<<endl;
}
    /*
    cout<<"image_database_dir: "<< image_database_dir <<endl;

    ifstream fin ( image_database_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    //读取需要添加进数据库的图像
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( image_database_dir+"/"+rgb_file );
        depth_files.push_back ( image_database_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }
    fin.close();
    //计算描述子 
    cout<<"generating features ... "<<endl;
    vector<Mat> descriptors;
    Ptr< Feature2D > detector = ORB::create();
    int index = 1;
    for ( string rgb_file:rgb_files )
    {
        Mat image = imread(rgb_file);
        vector<KeyPoint> keypoints; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
        cout<<"extracting features from image " << index++ <<endl;
    }
    cout<<"extract total "<<descriptors.size()*500<<" features."<<endl;
    */
    



   
    


