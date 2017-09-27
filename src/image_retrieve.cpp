#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

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
    string vocabulary_dir = localization::Config::get<string> ( "vocabulary_dir" );
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
        cout<<"searching for image "<<i<<" returns "<<ret<<endl<<endl;
    }
    cout<<"done."<<endl;

}


