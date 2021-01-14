#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include <opencv2/calib3d.hpp>

#include <iostream>

#include "mission.h"


namespace student {



 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
   throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
    
    
    static size_t id = 0;
  	static bool init = false;
  	static std::string folder_path;

  	if(!init){      
      bool exist = true;
      int i = 0;
      while(exist && i < 1000){
        std::stringstream ss;
        ss << config_folder << "/camera_image" << std::setw(3) << std::setfill('0') << i << "/";
  		  folder_path = ss.str();

        exist = std::experimental::filesystem::exists(folder_path);

        i++;        
      }
      
      if(i > 999 || !std::experimental::filesystem::create_directories(folder_path)){
        throw std::logic_error( "NO EMTY FOLDER" );
      }

  		init = true;
  	}
    	    
    cv::imshow( topic, img_in);
    char c;
    c = cv::waitKey(30);
    
    
    /*
    //Mat image; 
    //image = imread();

    if(img_in.empty()){
      cout<<"Couldnt load image"<<std::endl;
      return -1;
    }
    */
    cv::imshow(topic, img_in);

    std::stringstream img_file;
    switch (c) {    	
		case 's':		
			img_file << folder_path << std::setfill('0') << std::setw(3)  << (id++) << ".jpg";
		 	cv::imwrite( img_file.str(), img_in );
		 	std::cout << "Saved image " << img_file.str() << std::endl;
		 	break;

		default:
				break;
    }
    

    //namedWindow("Win1", WINDOW_AUTOSIZE);
    //imshow( "Win1", image);
    //waitKey(0); 
    
  }
  //THIS IS THE STUFF FOR PICKING THE 4 REAL WORLD 3D POINTS FOR EXTRINSIC CALIB
  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;

  void mouseCallback(int event, int x, int y, int, void* p)
  {
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;
    
    result.emplace_back(x*show_scale, y*show_scale);
    cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
      usleep(500*1000);
      done.store(true);
    }

    //added for debug 
    std::cout <<"mouseCallback just ran" << std::endl;
  }

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
  {
    result.clear();
    cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
      cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());

    //added for debug
    std::cout << "pickNpoints done" << std::endl;
    return result;
  }
  //UNTIL HERE 

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" ); 
    std::string file_path = config_folder + "/extrinsicCalib.csv";

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)){
          
      std::experimental::filesystem::create_directories(config_folder);
      
      image_points = pickNPoints(4, img_in);
      // SAVE POINT TO FILE
      // std::cout << "IMAGE POINTS: " << std::endl;
      // for (const auto pt: image_points) {
      //   std::cout << pt << std::endl;
      // }
      std::ofstream output(file_path);
      if (!output.is_open()){
        throw std::runtime_error("Cannot write file: " + file_path);
      }
      for (const auto pt: image_points) {
        output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
    }else{
      // LOAD POINT FROM FILE
      std::ifstream input(file_path);
      if (!input.is_open()){
        throw std::runtime_error("Cannot read file: " + file_path);
      }
      while (!input.eof()){
        double x, y;
        if (!(input >> x >> y)) {
          if (input.eof()) break;
          else {
            throw std::runtime_error("Malformed file: " + file_path);
          }
        }
        image_points.emplace_back(x, y);
      }
      input.close();
    }
    
    cv::Mat dist_coeffs;
    //HERE we have to put the actual parameters --------------------------------------------------------------------------------------------------------------------
    dist_coeffs   = (cv::Mat1d(1,4) << -1.5513759107742700e+00, 2.4365397169165259e+00, 1.1853422826599072e-01  , 5.3510161123244407e-04, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    // cv::Mat Rt;
    // cv::Rodrigues(rvec_, Rt);
    // auto R = Rt.t();
    // auto pos = -R * tvec_;

    if (!ok) {
      std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    std::cout << "extrinsic calbi done" << std::endl;
    std::cout << "rvec is" << std::endl << rvec << "tvec is" <<std::endl << tvec << std::endl;

    return ok; 
    

  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    
    //THIS IS THE FIRST METHOD, BUT THE UNDISTPRTION IS NOT SO VISIBLE
    //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
    //cv::imshow("win1", img_out);
    //std::cout << "undistortion done";
    //int key = cv::waitKey(0);

    //throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );  
    //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs, config_folder);
    
    
    //THIS IS THE SECOND METHOD, BUT THE IMSHOW KEEPS CRASHING
    static bool maps_done = false;
    static cv::Mat mapx, mapy;

    if(!maps_done){

      cv::Mat R;
      cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, mapx, mapy);

      maps_done = true;
    }

    cv::remap(img_in, img_out, mapx, mapy, cv::INTER_LINEAR);

    //std::cout << "TEST";

    //cv::imshow("win2", img_in);
    //cv::imshow("win1", img_out);
    //int key = cv::waitKey(0);
    std::cout << "image undistortion done" <<std::endl;
    
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
   // throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );  
    
    cv::Mat corner_pixels;

    //uses rvec and tvec in order to give us the 2d image of the view
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), corner_pixels);

    //we get a matrix which will allow us to transform from 3d real world to 2d camera view.
    plane_transf = cv::getPerspectiveTransform(corner_pixels, dest_image_points_plane);
    std::cout << "perspective transform done" << std::endl;
  }


void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    //we unwarp every image we pass to it. we use the plane_transf. The node calls twice, one for ground once for robot plane. 
    cv::warpPerspective(img_in, img_out, transf, img_in.size());  
    //imshow("win1", img_out);
    //int key = cv::waitKey(0);
    std::cout << "image unwarp done" <<std::endl;
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    
    //-----------------------------------VARIABLES ----------------------------------------------//
    
    cv::Mat hsv_img, obstacles_mask, victims_mask, gate_mask;
    std::vector<std::vector<cv::Point>> obstacles_contours, victims_contours, gate_contours;
    std::vector<cv::Point> approx_obstacles, approx_victims, approx_gate;
    cv::Mat red_mask_low, red_mask_high;
    
    
    //----------------------------------IMAGE2HSV----------------------------------------//
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    //-----------------------------------OBSTACLES-------------------------------------------------//
    std::cout << " -------------------OBSTACLES-------------" << std::endl;
    //for the red mask we need both sides of the now 180 degrees cilinder. so thats why wehave 2 functions.
    cv::inRange(hsv_img, cv::Scalar(0, 102, 86), cv::Scalar(40, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(164, 102, 86), cv::Scalar(180, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, obstacles_mask); 
    //Filtering
    cv::Mat kernel_obst = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(1,-1));
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
    for(int i=0;i<obstacles_contours.size();++i){
      //std::cout << obstacles_contours[i] << std::endl;
      cv::approxPolyDP(obstacles_contours[i], approx_obstacles, 5, true);
      std::cout << approx_obstacles.size() << std::endl << approx_obstacles << std::endl << "------------------------------------------------------------------" <<std::endl;
      //we scale each obstacle and put it in the obstacle list vector
      Polygon obstacle;
      for(const auto& point:approx_obstacles){
        obstacle.emplace_back(point.x/scale, point.y/scale);
      }
      obstacle_list.push_back(obstacle);
    }
    std::cout <<"Obstacles found!" << " There are " << obstacle_list.size() << std::endl;

    //-----------------------------------VICTIMS-AND-GATE------------------------------------------------//
    std::cout << " -------------------VICTIMS AND GATE-------------" << std::endl;
    //Get the green mask
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(110, 255, 255), victims_mask);

    //Filtering
    cv::Mat kernel_vict = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9), cv::Point(1,-1));
    cv::erode(victims_mask, victims_mask, kernel_vict);
    cv::dilate(victims_mask, victims_mask, kernel_vict);

    //Finding the vertices 
    cv::findContours(victims_mask, victims_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //----------------------------DIGIT RECOGNITION PREPARATION-------------------------------------------------------------
    cv::Mat victims_mask_inv, filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(victims_mask, victims_mask_inv);
    //Load the templates from file
    std::vector<cv::Mat> templates;
    for(int i =0; i<=9; ++i){
      templates.emplace_back(cv::imread("../imgs/template/"+std::to_string(i)+ ".png"));
    }
    //we get rid of the green spaces
    img_in.copyTo(filtered, victims_mask_inv);
    //----------------------------------------------------------------------------------------------------------------------

    //DETECTING GATE AND VICTIMS
    cv::Rect boundRect;//(victims_contours.size());

    for(int i = 0 ;i < victims_contours.size();++i){

      int area_test = cv::contourArea(victims_contours[i]);
      //DETECTING THE GATE
      if(area_test < 5500 && area_test > 2500){
        cv::approxPolyDP(victims_contours[i], approx_victims, 8, true);
        std::cout << "GATE found with area  " << area_test << " with number of vertices " << approx_victims.size() << " iteration " << i <<  std::endl;
        for(const auto& point:approx_victims){
          gate.emplace_back(point.x/scale, point.y/scale);
        }
      }

      //DETECTING THE VICTIMS
      if(area_test > 5500){

        cv::approxPolyDP(victims_contours[i], approx_victims, 2, true);
        
        //draw rectangle
        boundRect = boundingRect(cv::Mat(approx_victims));

        //actual digit recognition
        cv::Mat process_roi(filtered, boundRect);

        if(process_roi.empty()) continue;

        cv::resize(process_roi, process_roi, cv::Size(200,200));
        cv::threshold(process_roi, process_roi, 100,255,0);

        cv::flip(process_roi,process_roi,1); //0 on x, 1 on y, -1 on z
        //cv::rotate(process_roi, process_roi, cv::ROTATE_90_CLOCKWISE);

        //cv::imshow("roi", process_roi);
        //int key = cv::waitKey(0);

        //actual image recognition
        int best_score = 0;
        int match = -1;

        for(int j=0; j<templates.size();++j){
          cv::Mat result;
          cv::matchTemplate(process_roi, templates[j], result, cv::TM_CCOEFF);
          double score;
          cv::minMaxLoc(result, nullptr, &score);

            
          if(score > best_score){
            best_score = score;
            match = j;
          }
        }
        //std::cout << "Match is ---" << match << std::endl;

        Polygon victim;
        for(const auto& point:approx_victims){
          victim.emplace_back(point.x/scale, point.y/scale);
        }
        std::pair<int,Polygon> victim_paired;
        victim_paired.first = match;
        victim_paired.second = victim;

        victim_list.push_back(victim_paired);
        std::cout << "Victim number " << match << " found with area  " << area_test << " with number of vertices " << approx_victims.size() << " at iteration " << i << std::endl;
      }
    } 

    std::cout <<"Victims and Gate found! " << "We have " << victim_list.size() << " victims" <<std::endl;
    std::cout<<"Process Map ran" << std::endl;

    return true;
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
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

/*
  // See: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
  std::vector<Polygon> inflate(std::vector<Polygon> obs, double R){
    // The Clipper library only works with int, therefore our doubles are scaled to int trying not to lose much precision.
    double scale_int = 10000;
    int R_int = int ( R * scale_int );
    ClipperLib::Paths solution;
    ClipperLib::ClipperOffset co;
    for ( unsigned int i = 0; i < obs.size(); i++ ){
        ClipperLib::Path temp_path;
        for ( unsigned int j = 0; j < obs[i].size(); j++ ){
            temp_path << ClipperLib::IntPoint( int( obs[i][j].x * scale_int ), int( obs[i][j].y * scale_int) );
        }
        // NOTE: jtSquare is selected in order to have simple polygons which are fast to compute
        // Other slower, more precise solutions are jtRound, jtMiter
        // See: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Types/JoinType.htm
        co.AddPath(temp_path, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    }
    co.Execute(solution, R_int);

    // Scale back to double and return
    vector<Polygon> obs_infl;
    for ( unsigned i = 0; i < solution.size(); i++ ){
        Polygon temp_pol;
        for ( unsigned j = 0; j < solution[i].size(); j++ ){
            temp_pol.push_back(Point{solution[i][j].X/scale_int, solution[i][j].Y/scale_int});
        }
        obs_infl.push_back(temp_pol);
    }
    return obs_infl;
  }

  // Compute the centroid of a polygon. See: https://bell0bytes.eu/centroid-convex/
  Point centroid(const Polygon pol){
    float centroidX = 0, centroidY = 0;
    float det = 0, tempDet = 0;
    unsigned int j = 0;
    unsigned int nVertices = (unsigned int)pol.size();

    for (unsigned int i = 0; i < nVertices; i++){

      // closed polygon - last vertex connects with the first one
      if ( i + 1 == nVertices ){ j = 0; }
      else { j = i + 1; }

      // compute the determinant
      tempDet = pol[i].x * pol[j].y - pol[j].x*pol[i].y;
      det += tempDet;

      centroidX += (pol[i].x + pol[j].x)*tempDet;
      centroidY += (pol[i].y + pol[j].y)*tempDet;
    }

    // divide by the total mass of the polygon
    centroidX /= 3*det;
    centroidY /= 3*det;

    return Point{centroidX, centroidY};
  }

  */

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    //throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );   

    std::cout << "PLan path started" << std::endl;

    RRTS_params RRTS_pars;
    RRTS_pars.maxIt = 5000;
    RRTS_pars.tol = 0.02;
    RRTS_pars.d_lim = 0.07;
    RRTS_pars.b = 0.30;

    Dubins_params Dubins_pars;
    Dubins_pars.Kmax = 99.0;
    Dubins_pars.k = 4;
    Dubins_pars.M = 16;

    path = mission1(RRTS_pars, Dubins_pars, borders, obstacle_list, victim_list, gate, x,y, theta);

    return true;


  
  }  


  


}

