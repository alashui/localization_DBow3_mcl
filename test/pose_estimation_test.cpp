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

    localization::Config::setParameterFile ( argv[1] );    
    string map_dir = localization::Config::get<string> ( "map_dir" );
    string image_query_dir = localization::Config::get<string> ( "image_query_dir" );
    
    
	localization::PoseEstimation pose_estimation;    
	pose_estimation.mapInitialization();
	//pose_estimation.map_->save(map_dir);
	//pose_estimation.map_->load(map_dir);
cout <<"db 111" << endl;

       
    localization::ImageRetrieve::Ptr image_retrieve (new localization::ImageRetrieve ); 
    int num_result=20;
cout <<"db 112" << endl;
    image_retrieve->setResultNum(num_result);
cout <<"db 113" << endl;
    image_retrieve->map_ = pose_estimation.map_;
 cout <<"db 1111" << endl;           
    for ( int i=0; i<1; i++ )
    {	 cout <<"db 2" << endl; 
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
