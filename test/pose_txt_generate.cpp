//通过result_after.g2o文件生成pose.txt文件
#include "localization/config.h"

using namespace localization;

int main(int argc, char** argv)
{
    if ( argc != 2 )
    {
        cout<<"usage: parameter_file!"<<endl;
        return 1;
    }

    Config::setParameterFile ( argv[1] );    
    string image_database_dir = Config::get<string> ( "image_database_dir" );

	string dir_result_after_g2o(image_database_dir + "/result_after.g2o");
	string dir_pose(image_database_dir +  "/pose.txt");						

    ifstream fin(dir_result_after_g2o);	    	//dir_result_after_g2o
	ofstream fout(dir_pose);
	    										
	if (!fin)								
	{
	    cerr<<"cannot find result_after.g2o file"<<endl;
		    //return 1;
	}
	else//文件存在
	{
	   string temp;	
	   int num_line=0;
	   while(getline(fin,temp))		//获取一行,一行数据格式为  VERTEX_SE3:QUAT 1   0 0 0 0 0 0 1  
										//八位数字分别表示 x y z qx qy qz qw(前三位为位置，后四位为四元数表示的旋转角)
	   {	
	   	//对每一行数据的读入
			num_line++;
			if (num_line==2)	//忽略第二行 数据为 FIX 1
				continue;
			string temp_str;  //忽略每行前面的字符串 VERTEX_SE3:QUAT
		   	vector<double> double_vec; 
		   	double num;
			istringstream iss(temp);
			iss >> temp_str;
			if (temp_str!="VERTEX_SE3:QUAT") break;  	
				
			while(iss >> num)  		//分别将这一行数据读入						
				double_vec.push_back(num);

			//将x y z q1 q2 q3 q4 转换为sophus::se3 表示
			Eigen::Vector3d t(double_vec[1],double_vec[2],double_vec[3]);
		 	Eigen::Quaterniond q(double_vec[7],double_vec[4],double_vec[5],double_vec[6]);
											//Eigen::Quaterniond里使用顺序qw qx qy qz 
			Sophus::SE3 T(q,t);
			cout<< double_vec[0] <<endl;
			cout<< T.matrix()<<endl;

			Eigen::MatrixXd m(4,4);
			m=T.matrix();	

			fout << double_vec[0] <<" " 
				 << m(0,0) <<" " << m(0,1)<<" " << m(0,2)<<" " << m(0,3) <<" "
				 << m(1,0) <<" " << m(1,1)<<" " << m(1,2)<<" " << m(1,3) <<" "
				 << m(2,0) <<" " << m(2,1)<<" " << m(2,2)<<" " << m(2,3) <<" "
				 <<endl;																		
	   }
	   fout.close();
	   fin.close();
	}           	


}


