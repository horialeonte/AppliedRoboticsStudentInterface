#include "victim_gate_detection.h"
bool victim_gate_detection(const cv::Mat& img_in, const double scale,  std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate){
  
  std::cout << " -------------------VICTIMS AND GATE-------------" << std::endl;
  //-----------------------------------VARIABLES ----------------------------------------------//
  cv::Mat hsv_img, victims_mask, gate_mask;
  std::vector<std::vector<cv::Point>> victims_contours, gate_contours;
  std::vector<cv::Point> approx_victims, approx_gate;

  cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
     
  //Get the green mask
  cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(110, 255, 255), victims_mask);

  //Filtering
  cv::Mat kernel_vict = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9), cv::Point(1, -1));
  cv::erode(victims_mask, victims_mask, kernel_vict);
  cv::dilate(victims_mask, victims_mask, kernel_vict);

  //Finding the vertices
  cv::findContours(victims_mask, victims_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  //----------------------------DIGIT RECOGNITION PREPARATION-------------------------------------------------------------
  cv::Mat victims_mask_inv, filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::bitwise_not(victims_mask, victims_mask_inv);

  //Load the templates from file
  std::vector<cv::Mat> templates;
  for (int i = 0; i <= 9; ++i){
    templates.emplace_back(cv::imread("../imgs/template/" + std::to_string(i) + ".png"));
  }
  //we get rid of the green spaces
  img_in.copyTo(filtered, victims_mask_inv);
  //----------------------------------------------------------------------------------------------------------------------

  //DETECTING GATE AND VICTIMS
  cv::Rect boundRect; //(victims_contours.size());

  for (int i = 0; i < victims_contours.size(); ++i)
  {
    //computing the contour area so we separate gate and victims
    int area_test = cv::contourArea(victims_contours[i]);
    
    //DETECTING THE GATE
    if (area_test < 5500 && area_test > 2500)
    {
      cv::approxPolyDP(victims_contours[i], approx_victims, 8, true);
      std::cout << "GATE found with area  " << area_test << " with number of vertices " << approx_victims.size() << " iteration " << i << std::endl;
      for (const auto &point : approx_victims)
      {
        gate.emplace_back(point.x / scale, point.y / scale);
      }
    }

    //DETECTING THE VICTIMS
    if (area_test > 5500)
    {
      cv::approxPolyDP(victims_contours[i], approx_victims, 2, true);

      //draw rectangle
      boundRect = boundingRect(cv::Mat(approx_victims));

      //actual digit recognition
      cv::Mat process_roi(filtered, boundRect);

      if (process_roi.empty())
        continue;

      //we are processing the rectangle of interest so it matches the pattern
      cv::resize(process_roi, process_roi, cv::Size(200, 200));
      cv::threshold(process_roi, process_roi, 100, 255, 0);

      //TODO---UNCOMMENT FOR FIRST RUN ON NEW MAP---
      imshow("win1", process_roi);
      int kk = cv::waitKey(0);

      //image flipper
      switch(i){
        case 1: {
          cv::flip(process_roi, process_roi, 1); //0 on x, 1 on y, -1 on z
          cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);
          break;
        }

        case 2: {
          cv::flip(process_roi, process_roi, 1); //0 on x, 1 on y, -1 on z
          cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);
          break;
        }

        case 3: {
          cv::flip(process_roi, process_roi, 1); //0 on x, 1 on y, -1 on z
          cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);
          break;
        }

        case 4:{
          cv::flip(process_roi, process_roi, 1); //0 on x, 1 on y, -1 on z
          cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);
          cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);
          break;
        }

        case 5:{
          cv::flip(process_roi, process_roi, 1); //0 on x, 1 on y, -1 on z
          //cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);
          break;
        }
      }

      //actual image recognition
      int best_score = 0;
      int match = -1;

      for (int j = 0; j < templates.size(); ++j)
      {
        cv::Mat result;
        cv::matchTemplate(process_roi, templates[j], result, cv::TM_CCOEFF);
        double score;
        cv::minMaxLoc(result, nullptr, &score);

        if (score > best_score)
        {
          best_score = score;
          match = j;
        }
      }
      //std::cout << "Match is ---" << match << std::endl;

      Polygon victim;
      for (const auto &point : approx_victims)
      {
        victim.emplace_back(point.x / scale, point.y / scale);
      }
      std::pair<int, Polygon> victim_paired;
      victim_paired.first = match;
      victim_paired.second = victim;

      victim_list.push_back(victim_paired);
      std::cout << "Victim number " << match << " found with area  " << area_test << " with number of vertices " << approx_victims.size() << " at iteration " << i << std::endl;
    }
  } 

  std::cout <<"Victims and Gate found! " << "We have " << victim_list.size() << " victims" <<std::endl;
}
