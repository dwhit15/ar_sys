/**
* @file single_board.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief ROS version of the example named "simple_board" in the Aruco software package.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include <ar_sys/utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace aruco;

class ArSysSingleBoard
{
	private:
		cv::Mat inImage, resultImg;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
    bool publish_tf;
		bool publish_pose;
		bool publish_corners;
		bool calculate_pose;
		MarkerDetector mDetector;
		vector<Marker> markers;
		BoardConfiguration the_board_config;
		BoardDetector the_board_detector;
		Board the_board_detected;
		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher debug_pub;
		ros::Publisher pose_pub;
		ros::Publisher transform_pub;
		ros::Publisher position_pub;
		// ros::Publisher corner_measure_uv_pub;
		// ros::Publisher corner_board_pos_pub;
		ros::Publisher corner_pub;
		std::string board_frame;

		double marker_size;
		std::string board_config;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		tf::TransformListener _tfListener;

	public:
		ArSysSingleBoard()
			: cam_info_received(false),
			nh("~"),
			it(nh)
		{
			image_sub = it.subscribe("/image", 1, &ArSysSingleBoard::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysSingleBoard::cam_info_callback, this);

			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
			// corner_measure_uv_pub = nh.advertise<std_msgs::Float32MultiArray>("corners/uv_camera", 100);
			// corner_board_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("corners/xyz_board", 100);
			corner_pub = nh.advertise<std_msgs::Float32MultiArray>("corners", 100);

			nh.param<double>("marker_size", marker_size, 0.05);
			nh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
			nh.param<std::string>("board_frame", board_frame, "");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, false);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, false);
      nh.param<bool>("publish_tf", publish_tf, false);
      nh.param<bool>("publish_pose", publish_pose, true);
      nh.param<bool>("publish_corners", publish_corners, false);
      nh.param<bool>("calculate_pose", calculate_pose, true);

			the_board_config.readFromFile(board_config.c_str());
			ROS_INFO_STREAM(calculate_pose);
			if (!calculate_pose)
				the_board_config.mInfoType=BoardConfiguration::NONE;

			ROS_INFO("ArSys node started with marker size of %f m and board configuration: %s",
					 marker_size, board_config.c_str());
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
      static tf::TransformBroadcaster br;

			if(!cam_info_received) return;

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
				resultImg = cv_ptr->image.clone();

				//detection results will go into "markers"
				markers.clear();
				//Ok, let's detect
				mDetector.detect(inImage, markers, camParam, marker_size, false);
				//Detection of the board

				float probDetect=the_board_detector.detect(markers, the_board_config, the_board_detected, camParam, marker_size);

				if (calculate_pose) {

					if (probDetect > 0.0 && publish_pose)
					{
						tf::Transform transform = ar_sys::getTf(the_board_detected.Rvec, the_board_detected.Tvec);

						tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, board_frame);

	                    if (publish_tf)
	                        br.sendTransform(stampedTransform);

						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(transform, poseMsg.pose);
						poseMsg.header.frame_id = msg->header.frame_id;
						poseMsg.header.stamp = msg->header.stamp;
						pose_pub.publish(poseMsg);

						geometry_msgs::TransformStamped transformMsg;
						tf::transformStampedTFToMsg(stampedTransform, transformMsg);
						transform_pub.publish(transformMsg);

						geometry_msgs::Vector3Stamped positionMsg;
						positionMsg.header = transformMsg.header;
						positionMsg.vector = transformMsg.transform.translation;
						position_pub.publish(positionMsg);
					}
					//for each marker, draw info and its boundaries in the image
					for(size_t i=0; draw_markers && i < markers.size(); ++i)
					{
						markers[i].draw(resultImg,cv::Scalar(0,0,255),2);
					}

					if(camParam.isValid() && marker_size != -1)
					{
						//draw a 3d cube in each marker if there is 3d info
						for(size_t i=0; i<markers.size(); ++i)
						{
							if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
							if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
						}
						//draw board axis
						if (probDetect > 0.0) CvDrawingUtils::draw3dAxis(resultImg, the_board_detected, camParam);
					}

				}

				//for each marker, publish it's id and coordinate points
				if (publish_corners && markers.size() != 0)
				{
					std_msgs::Float32MultiArray corner_msg;
					std_msgs::MultiArrayLayout corner_msg_layout;
					size_t coordinate_length = 5;
					corner_msg_layout.dim.push_back(std_msgs::MultiArrayDimension());
					corner_msg_layout.dim[0].label="corner";
					corner_msg_layout.dim[0].size=markers.size();
					corner_msg_layout.dim[0].stride=markers.size()*coordinate_length;
					corner_msg_layout.dim.push_back(std_msgs::MultiArrayDimension());
					corner_msg_layout.dim[1].label="coordinates";
					corner_msg_layout.dim[1].size=coordinate_length;
					corner_msg_layout.dim[1].stride=coordinate_length;
					corner_msg.layout=corner_msg_layout;


					for(size_t marker_index=0; marker_index < markers.size(); ++marker_index)
					{
						Marker measured_marker = markers[marker_index];
						aruco::MarkerInfo reference_marker =
							the_board_detected.conf.getMarkerInfo ( measured_marker.id );
						const double edge_length = cv::norm ( reference_marker[0]-reference_marker[1] );
						const double marker_meter_per_pix = marker_size / edge_length;

						for (int corner_index=0;corner_index<4;corner_index++)
						{
							cv::Point2f corner_uv = measured_marker[corner_index];
							corner_msg.data.push_back(corner_uv.x);
							corner_msg.data.push_back(corner_uv.y);

							cv::Point3f corner_xyz = reference_marker[corner_index]*marker_meter_per_pix;
							corner_msg.data.push_back(corner_xyz.x);
							corner_msg.data.push_back(corner_xyz.y);
							corner_msg.data.push_back(corner_xyz.z);
						}
					}

					corner_pub.publish(corner_msg);
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0)
				{
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.frame_id = msg->header.frame_id;
					debug_msg.header.stamp = msg->header.stamp;
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector.getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_single_board");

	ArSysSingleBoard node;

	ros::spin();
}
