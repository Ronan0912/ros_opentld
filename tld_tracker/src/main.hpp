/*  Copyright 2012 UdeS University of Sherbrooke
 *
 *   This file is part of ROS_OpenTLD.
 *
 *   ROS_OpenTLD is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ROS_OpenTLD is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with ROS_OpenTLD. If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 * main.hpp
 *
 *  Created on: June 8, 2012
 *      Author: Ronan Chauvin
 */

//* Main
/**
* State machine of OpenTLD
* INIT
* TRACKER_INIT
* TRACKING
* STOPPED
*  
*/

#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <tld/TLD.h>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <string>

class Main
{
	public:
		Main()
		{
			tld = new tld::TLD();
			state = INIT;

			ros::NodeHandle np("~");
			np.param("showOutput", showOutput, true);
			np.param("loadModel", loadModel, false);
			np.param("autoFaceDetection", autoFaceDetection, false);
			np.param("exportModelAfterRun", exportModelAfterRun, false);
			np.param("modelImportFile", modelImportFile, std::string("model"));
			np.param("modelExportFile", modelExportFile, std::string("model"));
			np.param("cascadePath", face_cascade_path, 
					std::string("haarcascade_frontalface_alt.xml"));

			np.param("x", target_bb.x, 100);
			np.param("y", target_bb.y, 100);
			np.param("width", target_bb.width, 100);
			np.param("height", target_bb.height, 100);
			np.param("correctBB", correctBB, false);

			pub1 = n.advertise<tld_msgs::BoundingBox>(
                    "tld_tracked_object", 1000, true);
			pub2 = n.advertise<std_msgs::Float32>(
                    "tld_fps", 1000, true);
			sub1 = n.subscribe(
                    "image", 1000, &Main::imageReceivedCB, this);
			sub2 = n.subscribe(
                    "bounding_box", 1000, &Main::targetReceivedCB, this);
			sub3 = n.subscribe(
                    "cmds", 1000, &Main::cmdReceivedCB, this);

			semaphore.lock();
		}

		~Main()
		{
			delete tld;
		}

		void process();

	private:
		tld::TLD * tld; 
		bool showOutput;
		bool exportModelAfterRun;
		bool loadModel;
		bool autoFaceDetection;
		std::string modelImportFile;
		std::string modelExportFile;

		enum
		{
			INIT,
			TRACKER_INIT,
			TRACKING,
			STOPPED
		} state;

		bool correctBB;
		cv::Rect target_bb;
		cv::Mat target_image;

		std_msgs::Header img_header;
		cv::Mat img;
		cv_bridge::CvImagePtr img_buffer_ptr;
		cv::Mat gray;
		boost::interprocess::interprocess_mutex mutex;
		boost::interprocess::interprocess_mutex semaphore;
		ros::NodeHandle n;
		ros::Publisher pub1;
		ros::Publisher pub2;
		ros::Subscriber sub1;
		ros::Subscriber sub2;
		ros::Subscriber sub3;

		std::string face_cascade_path;
		cv::CascadeClassifier face_cascade;

        /*!
        * \brief This function return a new image has been received
        */
		bool newImageReceived();
		void getLastImageFromBuffer();

        /*!
        * \brief ROS image callback.
        */
		void imageReceivedCB(const sensor_msgs::ImageConstPtr & msg);

        /*!
        * \brief ROS target callback.
        */
		void targetReceivedCB(const tld_msgs::TargetConstPtr & msg);

        /*!
        * \brief ROS command callback.
        */
		void cmdReceivedCB(const std_msgs::CharConstPtr & cmd);

        /*!
        * \brief This function sends the tracked object as a BoudingBox message.
        *
        * \param x
        * \param y
        * \param width
        * \param height
        * \param confidence
        */
		void sendTrackedObject(int x, int y, int width, int height, float conf);

		void clearBackground();
		void stopTracking();
		void toggleLearning();
		void alternatingMode();
		void exportModel();
		void importModel();
		void reset();

        /*!
        * \brief This function allows to automatically initialize the target 
        * using the OpenCV Haar-cascade detector.
        */
		cv::Rect faceDetection();
};

#endif /* MAIN_HPP_ */
