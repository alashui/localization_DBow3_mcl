#include "localization/image_retrieve.h"
#include "localization/config.h"
#include "localization/frame.h"

int main( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: parameter_file!"<<endl;
        return 1;
    }

    localization::Config::setParameterFile ( argv[1] );    
    string image_query_dir = localization::Config::get<string> ( "image_query_dir" );

    localization::ImageRetrieve::Ptr image_retrieve (new localization::ImageRetrieve ); 
    int num_result=20;
    image_retrieve->setResultNum(num_result);
             
    for ( int i=0; i<1; i++ )
    {
        image_retrieve->frame_query_=localization::Frame::createFrame();
        string path = image_query_dir+"/"+to_string(i+1)+".png";                       
        image_retrieve->frame_query_->color_ = imread(path);   
        image_retrieve->frame_query_->extractKeyPoints();
        image_retrieve->frame_query_->computeDescriptors();
     
        image_retrieve->retrieve_result();
        cout<<"searching for image "<<i<<" returns "<<image_retrieve->result_<<endl;
		cout<<"image retrieve done."<<endl;        
    }

}


