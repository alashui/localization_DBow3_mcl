#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

#include "localization/config.h"

#include <dirent.h>  

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
    string image_database_dir = localization::Config::get<string> ( "image_database_dir" );
    string vocabulary_dir = localization::Config::get<string> ( "vocabulary_dir" );

    cout<<"database: "<<image_database_dir<<endl;


    //读取用于训练词典的图像,提取特征并计算描述子
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


/*
    //读取用于训练词典的图像,提取特征并计算描述子
    ifstream fin ( image_database_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
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
    // create vocabulary 
    cout<<"creating vocabulary, please wait ... "<<endl;
    DBoW3::Vocabulary vocab;
    vocab.create( descriptors );
    cout<<"vocabulary info: "<<vocab<<endl;
    vocab.save( vocabulary_dir,true );
    cout<<"vocabulary save in" <<vocabulary_dir<<endl;
    cout<<"done"<<endl;
    
    return 0;
}
