#include "localization/pose_estimation.h"
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
	localization::PoseEstimation pose_estimation;
    
	pose_estimation.mapInitialization();
	//pose_estimation.map_->save(map_dir);
	//pose_estimation.map_->load(map_dir);



}
