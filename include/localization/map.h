#ifndef MAP_H
#define MAP_H

#include "localization/common_include.h"
#include "localization/frame.h"
#include "localization/mappoint.h"

namespace localization
{
class Map
{
	public:
		typedef shared_ptr<Map> Ptr;
		unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks
		unordered_map<unsigned long, Frame::Ptr >     keyframes_;         // all key-frames

		enum Map_state 
			{
				EMPTY=0,
				EXIST=1
			};
		MapState    state_;			//status of Map ,exist or empty
		Map():map_state_(EMPTY) {}
		
		void insertKeyFrame( Frame::Ptr frame );
		void insertMapPoint( MapPoint::Ptr map_point );
		void save(string &filename);
		void load(string &filename);
};
}

#endif // MAP_H
