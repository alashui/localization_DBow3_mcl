#include "localization/common_include.h"

#include "DBoW3/DBoW3.h"
#include "localization/config.h"
#include <dirent.h>  //用于遍历目录文件的库

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
