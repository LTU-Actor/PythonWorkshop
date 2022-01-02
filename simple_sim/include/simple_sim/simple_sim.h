#ifndef SIMPLE_SIM_H_
#define SIMPLE_SIM_H_

#include <ros/ros.h>

// Include sensor messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

// Include CV headers
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include math headers
#include <vector>
#include <cmath>
#include <math.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <simple_sim/SimpleSimConfig.h>

// Constants
#define DIFF_DRIVE_RECT 0
#define DIFF_DRIVE_CIRC 1
#define ACKER_STEER_RECT 2

#define COLLISION_NONE 0
#define COLLISION_SOFT 1
#define COLLISION_HARD 2

class SimpleSim
{
 public:
  SimpleSim();
  ~SimpleSim();
  bool init();
  void loadFieldImage();
  void timeStep();
  void displayLocation();
  double delta_t_target_;  // The targetted time step in milliseconds

    
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  dynamic_reconfigure::Server<simple_sim::SimpleSimConfig> server_;
  simple_sim::SimpleSimConfig config_;

  // ROS Time

  // ROS Topic Publishers
  image_transport::Publisher camera_view_pub_;
  image_transport::ImageTransport it_;
  ros::Publisher odom_pub_;
  ros::Publisher lidar_scan_pub_;
  ros::Publisher gps_pub_;
  
  // ROS Topic Subscribers
  ros::Subscriber cmd_vel_sub_;

  // ROS topics
  nav_msgs::Odometry odom_;
  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped transformStamped_;
 
  //////////////
  // Variables
  //////////////
  // Source image filename
  std::string fname_field_img_;

  // Field Image
  cv::Mat img_field_master_;
  
  // Set image scaling
  double x_scale_field_;
  double y_scale_field_; 
  double img_view_scale_;
  
  // Coordinate system location on field image
  int x_img_cg_;
  int y_img_cg_;

  // Robot type
  int robot_type_;
  
  // Robot location variables
  bool init_flag_;
  double Xr_init_;
  double Yr_init_;
  double Theta_init_;
  double Xr_;
  double Yr_;
  double Theta_;
  cv::Matx22f Rz_;

  // GPS variables
  double latitude_base_;
  double longitude_base_;
  double latitude_;
  double longitude_;
  cv::Point2f north_unit_vec_;
  double earth_radius_;
  
  // Camera parameters and location
  double camera_phi_;                   // Rotation about local y-axis
  int camera_f_;                        // Focal length
  bool camera_border_;                  // Flag for camera view border
  double Zc_;                           // Height
  cv::Point2f camera_corners_ref_[4];   // View corners @Xr_=Yr_=Theta_=0
  double camera_width_;                 // Physical dimension
  double camera_length_;                // Physical dimension
  double camera_x_ref_;                 // Postion reference
  double camera_y_ref_;                 // Postion reference
  cv::Point2f camera_pts_ref_[4];       // Drawing reference

  // Lidar locations and parameters
  double Xl_;
  double Yl_;
  double lidar_x_ref_;
  double lidar_y_ref_;
  double lidar_radius_ref_;

  // Define twist inputs and angular velocity
  double linear_x_;
  double angular_z_;
  double omega_;
  double steer_angle_;

  // Define robot dimensions
  double robot_width_;
  double robot_length_;
  double robot_radius_;
  int robot_width_img_;
  int robot_length_img_;
  double rwheel_;
  double wheel_width_;
  double trackwidth_;
  double axle_dst_ = 0;
  double wheelbase_;
  double max_steer_;
  cv::Point2f body_corners_ref_[4];
  cv::Point2f lr_whl_ref_[4];
  cv::Point2f rr_whl_ref_[4];
  cv::Point2f lf_whl_center_ref_;
  cv::Point2f rf_whl_center_ref_;
  cv::Point2f lf_whl_ref_[4];
  cv::Point2f rf_whl_ref_[4];

  // Define camera view
  cv::Mat img_cam_;
  cv::Point2f camera_corners_[4];
  cv::Point2i camera_corners_img_[4];

  // Lidar scan
  bool lidar_active_ = false;
  float lidar_max_dist_;
  float scan_data_[360] = { 0.0 };
  cv::Point2i scan_loc_img_[360];
  int lidar_marker_size_ = 4;

  // Obstruction information
  bool exterior_walls_ = false;
  bool obstructions_ = false;
  int collision_model_ = COLLISION_NONE;
  bool obs_sim_stopped_ = false;
  std::vector<std::vector<cv::Point2f>> rect_obs_corners_;
  std::vector<cv::Point2f> circle_obs_center_;
  std::vector<double> circle_obs_radius_;
  
  // Odometry variables
  double pose_cov_[36];

  // Colors - BGR
  cv::Scalar robot_color_;
  cv::Scalar tire_color_;
  cv::Scalar camera_color_;
  cv::Scalar lidar_color_;
  cv::Scalar wall_color_;
  cv::Scalar obs_color_;
  
  // Parameter functions
  void setDefaultParameters();
  bool loadParameters();
  bool loadObstructions(int echo_on);
  bool checkParameters();
    
  // Callback function prototypes
  void configCallback(simple_sim::SimpleSimConfig &config, uint32_t level);
  void cmdVelocityCb(const geometry_msgs::Twist::ConstPtr &msg);

  // Publish functions
  void publishLidarScan();
  void publishGPS();
  void publishTransform( const ros::Time time_now);
  
  // Coordinate mapping functions
  void real_to_image_coords( const cv::Point2f &point,
			     cv::Point2i &img_point );

  // Geometric support functions
  double angle2D(const cv::Point2f p1, const cv::Point2f p2);
  bool pointInsidePolygon(const cv::Point2f *polygon,
			  const int n, const cv::Point2f p);
  bool robotCollision( );

  // Drawing functions
  void updateFieldImage(const bool update_time_disp,
			const double delta_t);
  void drawFieldCoord(cv::Mat &img_field );
  void drawRobot(cv::Mat &img_field);
  void drawRobotOLD(cv::Mat &img_field);
  void drawTire(cv::Mat &img_field, cv::Point2f *whl_ref);
  void drawOutlineCameraView(cv::Mat &img_field);
  void drawFilledPoly(cv::Mat &img, const cv::Point2i vertices[],
		      const cv::Scalar color );
  void drawPolygonObstruction(cv::Mat &img_field,
			      const std::vector<cv::Point2f> pts);
  void drawCircularObstruction(cv::Mat &img_field,
			       const cv::Point2f center,
			       const double radius); 
  void drawLidarScan(cv::Mat &img_field);

  // Reference frame functions
  void setRotationZ();
  void setRobotRefPoints();
  void setLidarGlobalLocation();
  
  // Camera functions
  void getCameraImage(cv::Mat &img_field);
  void getCameraCornersOLD(cv::Mat &img_field);
  void getCameraCorners();
  void getRefCameraCorners();
  
  // Lidar functions
  void initializeLidarScan();
  void fillLidarScanWalls();
  void fillLidarScanPolygonObstruction(const std::vector<cv::Point2f> pts);
  void fillLidarScanLine(const cv::Point2f pta, const cv::Point2f ptb );
  void fillLidarScanCircularObstruction(const cv::Point2f center,
					const double radius );

  // GPS functions
  void computeGPSLocation();
  
};
#endif // SIMPLE_SIM_H_
