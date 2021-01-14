#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>


#include <sstream>

#include <opencv2/calib3d.hpp>

#include <iostream>

namespace std{

	cv::Mat img_in;

	img_in = cv::imread("imagine.png");	
	cv::imshow(img_in);
	//cv::cvtColor(img_in, )

}