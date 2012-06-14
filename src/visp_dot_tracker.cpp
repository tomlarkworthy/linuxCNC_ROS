#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDot2.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>

#include <ros/ros.h>
#include <ros/param.h>

#include <image_transport/image_transport.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/version.hpp>

#include <opencv-2.3.1/opencv2/opencv.hpp>

#include <visp_camera_calibration/CalibPointArray.h>

#include "conversion.hh"
#include "callbacks.hh"

/* 
  // read 3D model from parameters
  XmlRpc::XmlRpcValue modelpoints__x_list;
  XmlRpc::XmlRpcValue modelpoints__y_list;
  XmlRpc::XmlRpcValue modelpoints__z_list;
  ros::param::get(visp_camera_calibration::modelpoints__x_param,modelpoints__x_list);
  ros::param::get(visp_camera_calibration::modelpoints__y_param,modelpoints__y_list);
  ros::param::get(visp_camera_calibration::modelpoints__z_param,modelpoints__z_list);
  ROS_ASSERT(modelpoints__x_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(modelpoints__x_list.size() == modelpoints__y_list.size() && modelpoints__x_list.size()==modelpoints__z_list.size());
  for(int i=0;i<modelpoints__x_list.size();i++){
    ROS_ASSERT(modelpoints__x_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(modelpoints__y_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(modelpoints__z_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    double X = static_cast<double>(modelpoints__x_list[i]);
    double Y = static_cast<double>(modelpoints__y_list[i]);
    double Z = static_cast<double>(modelpoints__z_list[i]);
    vpPoint p;
    p.setWorldCoordinates(X,Y,Z);
    modelpoints__.push_back(p);
  }
 **/


class DotTracker
  {
  public:
    typedef vpImage<unsigned char> image_t;
    typedef std::vector<vpPoint> points_t;
    typedef std::vector<vpImagePoint> imagePoints_t;
    
    DotTracker(unsigned queueSize = 5u);

    void spin();
    void waitForImage();
    void waitForInit();
    void pointCorrespondenceCallback(const visp_camera_calibration::CalibPointArrayPtr& msg);

    unsigned queueSize_;
		
	ros::NodeHandle nodeHandle_;
	image_transport::ImageTransport imageTransport_;
	
	image_t image_;

	std::string rectifiedImageTopic_;
	std::string cameraInfoTopic_;
	std::string pointCorrespondenceTopic_	;
	
	std::string model_prefix_;

	image_transport::CameraSubscriber cameraSubscriber_;
	ros::Subscriber pointCorrespondenceSubscriber_;

	std_msgs::Header header_;
	sensor_msgs::CameraInfoConstPtr info_;
	
	visp_camera_calibration::CalibPointArrayPtr pointMsg_;
	
	std::vector<visp_camera_calibration::CalibPoint> points_;
	std::vector<boost::shared_ptr<vpDot2> > trackers_;
	
	CvMat * homography_;
  };

void printMat(CvMat *A)
{
	int i, j;
	for (i = 0; i < A->rows; i++)
	{
		printf("\n"); 
		switch (CV_MAT_DEPTH(A->type))
		{
			case CV_32F:
			case CV_64F:
			for (j = 0; j < A->cols; j++)
			printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
			break;
			case CV_8U:
			case CV_16U:
			for(j = 0; j < A->cols; j++)
			printf ("%6d",(int)cvGetReal2D(A, i, j));
			break;
			default:
			break;
		}
	}
	printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dot_tracker");
  try
    {
      DotTracker tracker(1);
      if (ros::ok())
		tracker.spin();
	  ROS_INFO_STREAM("Finishing spinning");
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}


DotTracker::DotTracker(unsigned queueSize)
    : queueSize_(queueSize),
      nodeHandle_(),
      imageTransport_(nodeHandle_),
      image_(),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      pointCorrespondenceTopic_(),
      model_prefix_(),
      cameraSubscriber_(),
      pointCorrespondenceSubscriber_(),
      pointMsg_(),
	  points_(),
	  trackers_(),
	  homography_()
      
{
	
	homography_ = cvCreateMat(3, 3, CV_64FC1);
	// find camera prefix name
	ros::Rate rate (1);
    while (rectifiedImageTopic_.empty ())
      {
	ros::param::get ("DotTracker/camera_topic", rectifiedImageTopic_);
	ros::param::get ("DotTracker/camera_info_topic", cameraInfoTopic_);
	ros::param::get ("DotTracker/point_correspondence_topic", pointCorrespondenceTopic_);
	ros::param::get ("DotTracker/model_prefix", model_prefix_);
	if (!ros::param::has ("DotTracker/camera_topic"))
	  {
	    ROS_WARN
	      ("the camera_prefix parameter does not exist.\n"
	       "This may mean that:\n"
	       "- the tracker is not launched,\n"
	       "- the tracker and viewer are not running in the same namespace."
	       );
	  }
	else if (rectifiedImageTopic_.empty ())
	  {
	    ROS_INFO
	      ("tracker is not yet initialized, waiting...\n"
	       "You may want to launch the client to initialize the tracker.");
	  }
	if (!ros::ok ())
	  return;
	rate.sleep ();
      }
	      
	ROS_INFO_STREAM("camera_topic is " << rectifiedImageTopic_);
	ROS_INFO_STREAM("camera_info_topic is " << cameraInfoTopic_);
	ROS_INFO_STREAM("point_correspondence_topic is " << pointCorrespondenceTopic_);
	
	
	cameraSubscriber_ = imageTransport_.subscribeCamera
      (rectifiedImageTopic_, queueSize_,
       bindImageCallback(image_, header_, info_));
    //cameraSubscriber_ = imageTransport_.subscribe(rectifiedImageTopic_, queueSize_, imageCallback);
    // Wait for the image to be initialized.
     
    //register callback for pointCorrespondance 
    pointCorrespondenceSubscriber_ = nodeHandle_.subscribe(pointCorrespondenceTopic_, 1, &DotTracker::pointCorrespondenceCallback, this);
    //nodeHandle_.subscribe(pointCorrespondenceTopic_, 1, pointCorrespondenceCallbackGlobal);
    
     
    waitForImage();
    
    waitForInit();
}

bool epsilonEquals(double a, double b, double e){
		return (a + e > b) && (a - e < b);
}

void DotTracker::pointCorrespondenceCallback(const visp_camera_calibration::CalibPointArrayPtr& msg){
	
	ROS_INFO_STREAM("pointCorrespondenceCallback");
	points_.clear();
	trackers_.clear();
	
	
	std::string modelpoints__x_param(model_prefix_ + "/model_points_x");
	std::string modelpoints__y_param(model_prefix_ + "/model_points_y");
	std::string modelpoints__z_param(model_prefix_ + "/model_points_z");
	
	XmlRpc::XmlRpcValue modelpoints__x_list;
	XmlRpc::XmlRpcValue modelpoints__y_list;
	XmlRpc::XmlRpcValue modelpoints__z_list;
	ros::param::get(modelpoints__x_param,modelpoints__x_list);
	ros::param::get(modelpoints__y_param,modelpoints__y_list);
	ros::param::get(modelpoints__z_param,modelpoints__z_list);
	
	ROS_INFO_STREAM(modelpoints__x_param << " value " << modelpoints__x_list);
	ROS_INFO_STREAM(modelpoints__y_param << " value " << modelpoints__y_list);
	ROS_INFO_STREAM(modelpoints__z_param << " value " << modelpoints__z_list);
	
	ROS_ASSERT(modelpoints__x_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(modelpoints__x_list.size() == modelpoints__y_list.size() && modelpoints__x_list.size()==modelpoints__z_list.size());
	for(int i=0;i<modelpoints__x_list.size();i++){
		ROS_ASSERT(modelpoints__x_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(modelpoints__y_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(modelpoints__z_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				
		visp_camera_calibration::CalibPoint element = visp_camera_calibration::CalibPoint();
		
		
		element.X = modelpoints__x_list[i];
		element.Y = modelpoints__y_list[i];
		element.Z = modelpoints__z_list[i];
		
		ROS_INFO_STREAM("initialized"<< element.X << "," << element.Y << "," << element.Z); 
		
		points_.push_back(element);
		
		
		//ROS_INFO_STREAM("element i=" << element.i << " j=" << element.j);
		
		boost::shared_ptr<vpDot2> tracker(new vpDot2());
		
		tracker->setSizePrecision(.1);
		tracker->setEllipsoidShapePrecision(.1);
		tracker->setGrayLevelPrecision(.65);
		trackers_.push_back(tracker);
	}
	
	//now we have all the trackers for each model point, let us initialate trackers based on 
	//suplied calibration data
	
	for(uint i=0;i<msg->points.size();i++){
		visp_camera_calibration::CalibPoint element = msg->points[i];
		
		ROS_INFO_STREAM("looking for i=" << element.i << " j=" << element.j << "," << element.X << "," << element.Y << "," << element.Z); 
		
		for(uint j=0;j<points_.size();j++){
			visp_camera_calibration::CalibPoint element2 = points_[j];
			
			if( epsilonEquals(element.X, element2.X, 0.001) && 	
				epsilonEquals(element.Y, element2.Y, 0.001) &&
				epsilonEquals(element.Z, element2.Z, 0.001)){
					//we found (epsilon) identical point
					trackers_[j]->initTracking(image_, vpImagePoint(element.i, element.j) , 10);//dodgy???
					
					
				ROS_INFO_STREAM("found " << element2.i << " j=" << element2.j << "," << element2.X << "," << element2.Y << "," << element2.Z); 
			}
		}
	}
}


void
DotTracker::spin()
{
	int count = 0;
	boost::format fmtWindowTitle ("ViSP Dot tracker - [ns: %s]");
    fmtWindowTitle % ros::this_node::getNamespace ();

    vpDisplayX d(image_, image_.getWidth(), image_.getHeight(),
		 fmtWindowTitle.str().c_str());

	ros::Rate loop_rate_tracking(200);
	bool ok = false;
	vpHomogeneousMatrix cMo;
	vpImagePoint point (10, 10);
	while (!ok && ros::ok())
	{
		
		try
		  {
			//ROS_INFO_STREAM("spin once area");
			vpDisplay::display(image_);
			
			for(uint i=0;i<trackers_.size();i++){
				try{
					trackers_[i]->track(image_);
					trackers_[i]->display(image_, vpColor::red);
				}catch(...){
					//ROS_ERROR_STREAM("failed to track dot " << i);
				}
				
			}
			
			int N = points_.size();
			
			double srcData[N*2];
			double dstData[N*2];
			CvMat * src = cvCreateMat( N, 3, CV_64FC1);
			CvMat * dst = cvCreateMat( N, 3, CV_64FC1);
			
			CvMat * mask= cvCreateMat(1, N, CV_8UC1);
			for(int i = 0; i < N; i++ ){
				//model is src
				src->data.db[i*3] = points_[i].X;
				src->data.db[i*3+1] = points_[i].Y;
				src->data.db[i*3+2] = 1;
				
				//screen is dst
				dst->data.db[i*3] = trackers_[i]->getCog().get_i();
				dst->data.db[i*3+1] = trackers_[i]->getCog().get_j();
				dst->data.db[i*3+2] = 1;
				
				//ROS_INFO_STREAM("trackers_[i]->getCog() =" << trackers_[i]->getCog().get_i() << ", " << trackers_[i]->getCog().get_j());	
				//ROS_INFO_STREAM("points_[i] =" << points_[i].X << ", " << points_[i].Y);	
			}
			
				
			
			
			cvFindHomography(src, dst, homography_, CV_LMEDS, 0, mask);
			
			if(count++ % 10 == 0){
					count =0;
					printMat(homography_);
			}	
						
			for(int i = 0; i < N; i++ ){ 
				
				if(((int)(mask->data.ptr[i])) == 0){
					//note we have to transpose
					CvMat * dst2 = cvCreateMat( 3, 1, CV_64FC1); //screen (unkown)
					CvMat * src2 = cvCreateMat( 3, 1, CV_64FC1); //model (known)
					
					src2->data.db[0] = points_[i].X;
					src2->data.db[1] = points_[i].Y;
					src2->data.db[2] = 1.0;
					
					cvMatMul(homography_, src2, dst2);
					
					dst2->data.db[0] /= dst2->data.db[2];
					dst2->data.db[1] /= dst2->data.db[2];
					dst2->data.db[2] /= dst2->data.db[2];
					
					//ROS_INFO_STREAM("trackers_[i]->getCog() =" << trackers_[i]->getCog().get_i() << ", " << trackers_[i]->getCog().get_j());	
					//ROS_INFO_STREAM("for point number: " << i << " model x = " << points_[i].X << " model y = " << points_[i].Y);
					//ROS_INFO_STREAM("setting tracker "<< i << " to x = "<< dst2->data.db[0] << ", y = " << dst2->data.db[1] << "," << dst2->data.db[2]); 	
					
					try{
						//trackers_[i]->initTracking(image_, vpImagePoint(dst2->data.db[0], dst2->data.db[1]) , 15);//dodgy???	
						//trackers_[i]->getCog().set_i(dst2->data.db[0]);
						//trackers_[i]->getCog().set_j(dst2->data.db[1]);
						
						//ROS_INFO_STREAM("setting tracker "<< i << " to x = "<< dst2->data.db[0] << ", y = " << dst2->data.db[1] << "," << dst2->data.db[2]); 	
						
						boost::shared_ptr<vpDot2> tracker(new vpDot2());
		
						tracker->setSizePrecision(.1);
						tracker->setEllipsoidShapePrecision(.1);
						tracker->setGrayLevelPrecision(.65);
						
						trackers_[i] = tracker;
						trackers_[i]->initTracking(image_, vpImagePoint(dst2->data.db[0], dst2->data.db[1]) , 15);
						
						trackers_[i]->track(image_);
						trackers_[i]->display(image_, vpColor::green);
					}catch(...){
						ROS_ERROR_STREAM("failed to track dot " << i);
					}
					
					//trackers_[i]->getCog().set_i(dst2->data.db[0]);
					//trackers_[i]->getCog().set_j(dst2->data.db[1]);
				}
			}
			/*
			ROS_INFO_STREAM("mask=");
			for(int i = 0; i < N; i++ ){
					ROS_INFO_STREAM((int)mask->data.ptr[i]<<" ");
			}
			ROS_INFO("\n");
			for(int i = 0; i < 3; i++ ){
				for(int j = 0; j < 3; j++ ){
				
					ROS_INFO_STREAM(hom_ret->data.db[i + j*3]<<" ");
				}
				ROS_INFO("\n");
			}
			ROS_INFO("\n");
			*/
			vpDisplay::flush(image_);
			
			ros::spinOnce();
			loop_rate_tracking.sleep();
		  }
		catch(const std::runtime_error& e)
		  {
			ROS_ERROR_STREAM("C failed to initialize: "
					 << e.what() << ", retrying...");
		  }
		catch(const std::string& str)
		  {
			ROS_ERROR_STREAM("B failed to initialize: "
					 << str << ", retrying...");
		  }
		catch(...)
		  {
			ROS_ERROR("A failed to initialize, retrying...");
		  }
	}
}

void
  DotTracker::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (ros::ok()
	   && (!image_.getWidth() || !image_.getHeight()))
      {
	ROS_INFO_THROTTLE(1, "waiting for an image...");
	ros::spinOnce();
	loop_rate.sleep();
      }
  }
  
void
  DotTracker::waitForInit()
  {
    ros::Rate loop_rate(10);
    while (ros::ok()
	   && (points_.size() == 0))
      {
	ROS_INFO_THROTTLE(1, "waiting for initial points ...");
	ros::spinOnce();
	loop_rate.sleep();
      }
  }
