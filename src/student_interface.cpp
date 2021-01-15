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

//#include "mission.hpp"
#include "robot_finder.h"
#include "obstacle_detection.h"
#include "victim_gate_detection.h"
#include "border_detection.h"

//#include "dubins.hpp"
#include "mission.h"
//#include "rrt_star.hpp"
//#include "clipper.hpp"


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

    std::cout << "extrinsic calib done" << std::endl;
    std::cout << "rvec is" << std::endl << rvec << "tvec is" <<std::endl << tvec << std::endl;

    return ok; 
    

  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    
    //THIS IS THE FIRST METHOD
    //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
    //cv::imshow("win1", img_out);
    //std::cout << "undistortion done";
    //int key = cv::waitKey(0);

    //throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );  
    //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs, config_folder);
    
    
    //THIS IS THE SECOND METHOD
    static bool maps_done = false;
    static cv::Mat mapx, mapy;

    if(!maps_done){
      cv::Mat R;
      cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, mapx, mapy);
      maps_done = true;
    }
    cv::remap(img_in, img_out, mapx, mapy, cv::INTER_LINEAR);
    //std::cout << "image undistortion done" <<std::endl;
    
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
    //std::cout << "image unwarp done" <<std::endl;
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    
    //-----------------------------------VARIABLES ----------------------------------------------//
    cv::Mat hsv_img;
    //-----------------------------------IMAGE2HSV-----------------------------------------------//
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    //-----------------------------------OBSTACLES-----------------------------------------------//
    bool ok_obs = obstacle_detection(hsv_img, scale, obstacle_list);
    //-----------------------------------VICTIMS-AND-GATE----------------------------------------//
    bool ok_vict = victim_gate_detection(img_in, scale, victim_list, gate);
    //-----------------------------------BORDERDS------------------------------------------------//
    //bool ok_border = border_detection(hsv_img, scale, obstacle_list);

    std::cout<<"Process Map ran" << std::endl;

    return ok_obs && ok_vict;
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    //we call the function in robot_finder.cpp
    return robot_finder(img_in, scale, triangle, x,  y, theta);
    std::cout << scale << std::endl;
  }
/*

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
  }*/

  

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
    
    std::cout << "Plan path started" << std::endl;
    
    // Parameters RRT*
    RRTS_params RRTS_pars;
    RRTS_pars.maxIt = 5000;
    RRTS_pars.tol = 0.02;
    RRTS_pars.d_lim = 0.07;
    RRTS_pars.b = 0.30;
    
    // Parameters Dubins
    Dubins_params Dubins_pars;
    Dubins_pars.Kmax = 50.0;
    Dubins_pars.k = 4;
    Dubins_pars.M = 16;

    // Radius of the robot [m]
    const float R = 0.11;

    // Speed of the robot [m/s]
    const float v = 0.25;

    // Reward per rescued victim [s]
    const float zz = 0;

    // Choose mission. Valid missions are 1 and 2.
    const int mission = 1;

    // The path is returned.
    path = missionPlanner(RRTS_pars, Dubins_pars, borders, obstacle_list, victim_list, gate, x, y, theta, R, v, zz, mission);
    return true;


    return true;


  
  }  


  


}

