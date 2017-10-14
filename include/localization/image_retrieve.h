#ifndef _IMAGE_RETRIEVE_H
#define _IMAGE_RETRIEVE_H

#include "localization/common_include.h"

#include "DBoW3/DBoW3.h"
#include <dirent.h>   //用于遍历目录文件的库
#include "localization/config.h"

#include "localization/frame.h"

namespace localization
{

class ImageRetrieve
{
    public:
		typedef shared_ptr<ImageRetrieve> Ptr;
		enum Database_state 
		{
		    EMPTY=0,
		    EXIST=1
		};
		
		//cv::Mat image_query;
		Frame::Ptr frame_query_;
		DBoW3::QueryResults result_;
					    
		ImageRetrieve();
		~ImageRetrieve();
        
		void create_database();
		void retrieve_result();
        void setResultNum(int num){num_result_=num;}
        
    private:
		DBoW3::Database database_;		
		Database_state state_;        //数据库存在状态
		int num_result_;	

};

}

#endif // IMAGE_RETRIEVE

