#include "robot_finder.h"

bool robot_finder(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta){


//Variables
    std::vector<std::vector<cv::Point>> robot_contours;
    cv::Mat robot_mask, hsv_img;   
    std::vector<cv::Point> approx_robot;
    //Image transform
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    //Setting the blue mask 
    cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), robot_mask);

    //Filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(1,-1));
    cv::erode(robot_mask,robot_mask, kernel);
    cv::dilate(robot_mask,robot_mask, kernel);

    //FIND countours
    cv::findContours(robot_mask, robot_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool robot_found = false;
    //Approximate the contours
    for(int i=0;i<robot_contours.size();++i){
      cv::approxPolyDP(robot_contours[i], approx_robot, 25, true);
      //we have to be sure the robot was found, aka the triangle
      if (approx_robot.size() != 3) continue;
      robot_found = true;
      //std::cout << approx_robot.size() << std::endl << approx_robot << std::endl << "-------------------------------------------" <<std::endl;
      //it should be only one shape. so if we found the correct one, we end the for loop. 
      break;
    }
    //std::cout << "Robot found" << std::endl;

    //compute the pose
    if(robot_found){

      for (const auto& pt: approx_robot) {
        triangle.emplace_back(pt.x/scale, pt.y/scale);
      }

      double cx = 0, cy = 0;
      for (auto item: triangle) {
        cx += item.x;
        cy += item.y;
      }
      cx /= triangle.size();
      cy /= triangle.size();

      double dst = 0;
      Point vertex;
      for (auto& item: triangle)
      {
        double dx = item.x-cx;      
        double dy = item.y-cy;
        double curr_d = dx*dx + dy*dy;
        if (curr_d > dst)
        { 
          dst = curr_d;
          vertex = item;
        }
      }
      double dx = cx-vertex.x;
      double dy = cy-vertex.y;

      x = cx;
      y = cy;
      theta = std::atan2(dy, dx);

    }
    float maxt = 0;
    for (auto item: triangle) {
        float dist = std::sqrt(std::pow(x-item.x,2) + std::pow(y-item.y,2)*1.0);
        if(dist > maxt) maxt = dist;
        //std::cout << dist << " " << item.x << " " << item.y << maxt <<std::endl;
      }
    std::cout <<"Robot at " << x << " , " << y << " With orientation " << theta <<  " and inflation " << maxt << std::endl;
    //std::cout << "Find Robot ran"  << std::endl;

    return true;
}

