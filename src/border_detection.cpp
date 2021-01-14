#include "border_detection.h"

bool border_detection(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){

  //Variables
  cv::Mat border_mask;
  std::vector<std::vector<cv::Point>> border_contours;
  std::vector<cv::Point> approx_border;
  //--------------------------------------------------//
  std::cout << "===================================BORDERS===================================" << std::endl;

  //Setting the black mask
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), border_mask);

  //Filtering
  cv::Mat kernel_b = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(1, -1));
  cv::erode(border_mask, border_mask, kernel_b);
  cv::dilate(border_mask, border_mask, kernel_b);

  //FIND countours
  cv::findContours(border_mask, border_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  //cv::drawContours(hsv_img, border_contours, -1, cv::Scalar(140,190,40), 4, cv::LINE_AA);
  //cv::imshow("border", hsv_img);
  //int kk = cv::waitKey(0);

  //We go through all the contours
  for (int i = 0; i < border_contours.size(); ++i)
  {
    //we exclude the small areas, the wheels of the robot and the digits on the victims
    if (cv::contourArea(border_contours[i]) > 3000)
    {
      //we reduce the number of vertices , 3 was the best option 
      cv::approxPolyDP(border_contours[i], approx_border, 3, true);

      std::cout << "We have a border with a total number of vertices: " << approx_border.size() << std::endl;
      //we scale each obstacle and put it in the obstacle list vector
      Polygon border;
      for (const auto &point : approx_border)
      {
        border.emplace_back(point.x / scale, point.y / scale);
      }
      obstacle_list.push_back(border);
    }
  }
  
}
