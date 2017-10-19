#ifndef _IMAGE_RETRIEVE_H
#define _IMAGE_RETRIEVE_H

#include "localization/common_include.h"

#include "DBoW3/DBoW3.h"
#include <dirent.h>   //用于遍历目录文件的库
#include "localization/config.h"

#include "localization/frame.h"
#include "localization/map.h"

namespace localization
{
class Map;
class ImageRetrieve
{
    public:
		typedef shared_ptr<ImageRetrieve> Ptr;
		
		//给定frame和map,运行retrieve_result(),得到result;
		Frame::Ptr frame_query_;
		Map::Ptr map_;	
		DBoW3::QueryResults result_;   //DBoW3::QueryResults的数据成员std::vector<Result>
									   //DBoW3::Result 的数据成员public: EntryId Id; double Score;	
									   		//EntryId为unsigned int	的宏定义		
						 
		unordered_map< unsigned int, int > EntryId_frame_id_;  //将database中的EntryId和frame_id一一对应，
					    							  //便于从database.query()后的ret.Id得到frame_id
		double result_score_;
		int result_frame_id_;			    							  
					    							  
					    							  
		ImageRetrieve();
		~ImageRetrieve();
        
		void create_database();
		void read_db_frame_id_();
		void databaseInit();
		void retrieve_result();
        void setResultNum(int num){num_result_=num;}
        
    private:
		DBoW3::Database database_;		
		int num_result_;	

};

}

#endif // IMAGE_RETRIEVE

