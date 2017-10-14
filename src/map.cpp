#include "localization/map.h"

namespace localization
{

void Map::insertKeyFrame ( Frame::Ptr frame )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
    }
}

void Map::addKeyFrame(Frame::Ptr frame)
{
    if ( keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for ( size_t i=0; i < frame->keypoints_.size(); i++ )
        {
            double d = frame->findDepth ( keypoints_curr_[i] );
            if ( d < 0 ) 
                continue;
            Vector3d p_world = frame->camera_->pixel2world (
                Vector2d ( frame->keypoints_[i].pt.x, frame->keypoints_[i].pt.y ), frame->T_c_w_, d   );           
            Vector3d n = p_world - frame->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(  p_world, n,
              				 frame->descriptors_.row(i).clone(), frame.get()    );
            insertMapPoint( map_point );
        }
    }
    
    insertKeyFrame ( frame );
}

void Map::addMapPoints( const Frame::Ptr frame,
						const vector<int>* match_2dkp_index_)
{
    // add the new map points into map
    vector<bool> matched(frame_curr->keypoints_.size(), false); 
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i < frame_curr->keypoints_.size(); i++ )
    {
        if ( matched[i] == true )   //匹配过得点之前已经添加过,只添加未匹配上的特征点
            continue;
        double d = frame->findDepth ( keypoints_curr_[i] );
        if ( d < 0 ) 
            continue;
        Vector3d p_world = frame->camera_->pixel2world (
            Vector2d ( frame->keypoints_[i].pt.x, frame->keypoints_[i].pt.y ), frame->T_c_w_, d   );
        
        Vector3d n = p_world - frame->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(   p_world, n,
            				      frame->descriptors_.row(i).clone(), frame.get()  );        
        insertMapPoint( map_point );
    }
}




void save(string &filename)
{

}
void load(string &filename)
{

}




}
