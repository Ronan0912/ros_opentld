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
 * base_frame.hpp
 *
 *  Created on: May 17, 2012
 *      Author: Ronan Chauvin
 */

#ifndef BASE_FRAME_H_
#define BASE_FRAME_H_

#include <QtGui/QWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QObject>
#include <QtGui/QImage>
#include <QtCore/QRectF>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>

#include "ui_baseFrame.h"

class BaseFrame : public QWidget, private Ui::BaseFrame
{
	Q_OBJECT

	public:
		ros::NodeHandle n;
		ros::Publisher pub1;
		ros::Publisher pub2;
		ros::Subscriber sub1;
		ros::Subscriber sub2;
		ros::Subscriber sub3;

		BaseFrame();
		virtual ~BaseFrame();

	protected:
		void keyPressEvent(QKeyEvent * event);

	private:
		cv_bridge::CvImageConstPtr cv_ptr;
		bool first_image;

		void image_receivedCB(const sensor_msgs::ImageConstPtr & msg);
		void tracked_objectCB(const tld_msgs::BoundingBoxConstPtr & msg);
		void fps_trackerCB(const std_msgs::Float32ConstPtr & msg);

	signals:
		void sig_image_received(const QImage & image);
		void sig_tracked_object_changed(const QRectF & bb);
		void sig_fps_tracker_changed(int fps);
		void sig_confidence_changed(int confidence);

		public slots:
		void clear_background();
		void clear_and_stop_tracking();
		void toggle_learning();
		void alternating_mode();
		void export_model();
		void import_model();
		void reset();
};

#endif
