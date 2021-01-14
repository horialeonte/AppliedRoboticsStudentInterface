#include "obstacle_detection.h"
bool obstacle_detection(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){

  //Variables
  cv::Mat obstacles_mask;
  std::vector<std::vector<cv::Point>> obstacles_contours;
  std::vector<cv::Point> approx_obstacles;
  cv::Mat red_mask_low, red_mask_high;
  std::cout << " -------------------OBSTACLES-------------" << std::endl;

  //for the red mask we need both sides of the now 180 degrees cilinder. so thats why wehave 2 functions.
  cv::inRange(hsv_img, cv::Scalar(0, 102, 86), cv::Scalar(40, 255, 255), red_mask_low);
  cv::inRange(hsv_img, cv::Scalar(164, 102, 86), cv::Scalar(180, 255, 255), red_mask_high);
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, obstacles_mask);

  //Filtering
  cv::Mat kernel_obst = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, -1));
  cv::erode(obstacles_mask, obstacles_mask, kernel_obst);
  cv::dilate(obstacles_mask, obstacles_mask, kernel_obst);
  
  //Finding the vertices
  cv::findContours(obstacles_mask, obstacles_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  //std::cout << "Contours OBSTACLES list ---" << " THERE ARE  " << obstacles_contours.size() << std::endl;
  //cv::Mat contours_img;
  //contours_img = hsv_img.clone();
  //cv::drawContours(contours_img, obstacles_contours, -1, cv::Scalar(140,190,40), 4, cv::LINE_AA);
  //imshow("win",contours_img);

  //We approximate vertices and write the obstacles in the obstacle_list
  for (int i = 0; i < obstacles_contours.size(); ++i)
  {
    //std::cout << obstacles_contours[i] << std::endl;
    cv::approxPolyDP(obstacles_contours[i], approx_obstacles, 5, true);
    std::cout << "Obstacle number " << i+1 << " with a total nr of vertices of " << approx_obstacles.size() << std::endl;
              
    //we scale each obstacle and put it in the obstacle list vector
    Polygon obstacle;
    for (const auto &point : approx_obstacles)
    {
      obstacle.emplace_back(point.x / scale, point.y / scale);
    }
    obstacle_list.push_back(obstacle);
  }
  std::cout << "Obstacles found!" << " There are " << obstacle_list.size() << std::endl;
}
