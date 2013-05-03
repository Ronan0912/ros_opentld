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
 * base_frame.cpp
 *
 *  Created on: May 17, 2012
 *      Author: Ronan Chauvin
 */

#include "base_frame.hpp"
#include <sensor_msgs/image_encodings.h>

#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtGui/QFileDialog>

namespace enc = sensor_msgs::image_encodings;

BaseFrame::BaseFrame()
{   
	setupUi(this);

	QObject::connect(this,SIGNAL(sig_image_received(const QImage &)),base_frame_graphics_view,SLOT(image_received(const QImage &)));    
	QObject::connect(this,SIGNAL(sig_tracked_object_changed(const QRectF &)),base_frame_graphics_view,SLOT(tracked_objet_changed(const QRectF &)));
	QObject::connect(this,SIGNAL(sig_fps_tracker_changed(int)),lcd_fps_tracker,SLOT(display(int)));  
	QObject::connect(this,SIGNAL(sig_confidence_changed(int)),confidence_bar,SLOT(setValue(int)));  
	QObject::connect(background_reset_button,SIGNAL(clicked()),this,SLOT(clear_background()));
	QObject::connect(learning_button,SIGNAL(clicked()),this,SLOT(toggle_learning()));      
	QObject::connect(alternating_button,SIGNAL(clicked()),this,SLOT(alternating_mode()));
	QObject::connect(stop_tracking_button,SIGNAL(clicked()),this,SLOT(clear_and_stop_tracking()));    
	QObject::connect(importing_button,SIGNAL(clicked()),this,SLOT(import_model()));
	QObject::connect(exporting_button,SIGNAL(clicked()),this,SLOT(export_model()));
	QObject::connect(reset_button,SIGNAL(clicked()),this,SLOT(reset()));

	sub1 = n.subscribe("image", 1000, &BaseFrame::image_receivedCB, this);
	sub2 = n.subscribe("tracked_object", 1000, &BaseFrame::tracked_objectCB, this);
	sub3 = n.subscribe("fps_tracker", 1000, &BaseFrame::fps_trackerCB, this);
	pub1 = n.advertise<tld_msgs::Target>("tld_gui_bb", 1000, true);
	pub2 = n.advertise<std_msgs::Char>("tld_gui_cmds", 1000, true);

	first_image = true;
}

BaseFrame::~BaseFrame() 
{

}

void BaseFrame::keyPressEvent(QKeyEvent * event)
{
	switch (event->key()) 
	{
		case Qt::Key_Return:
		case Qt::Key_Enter:
		case Qt::Key_Backspace:
			qDebug() << "Enter";
			if(!base_frame_graphics_view->get_bb()->rect().isEmpty())
			{
				tld_msgs::Target msg;
				msg.bb.x = (int)base_frame_graphics_view->get_bb()->rect().x();
				msg.bb.y = (int)base_frame_graphics_view->get_bb()->rect().y();
				msg.bb.width = (int)base_frame_graphics_view->get_bb()->rect().width();
				msg.bb.height = (int)base_frame_graphics_view->get_bb()->rect().height();
				msg.bb.confidence = 1.0;
				cv_ptr->toImageMsg(msg.img);			
				pub1.publish(msg);
			}
			break;
		case Qt::Key_Q:
			qDebug() << "Quitting";
			close();
			break;
		case Qt::Key_B:
			clear_background();
			break;
		case Qt::Key_C:
			clear_and_stop_tracking();
			break;
		case Qt::Key_L:
			toggle_learning();
			break;
		case Qt::Key_A:
			alternating_mode();
			break;
		case Qt::Key_E:
			export_model();
			break;
		case Qt::Key_I:
			import_model();
			break;
		case Qt::Key_R:
			reset();
			break;
		case Qt::Key_F5:
			first_image = true;
		default:
			event->ignore();
			break;
	}
}

void BaseFrame::image_receivedCB(const sensor_msgs::ImageConstPtr & msg)
{
	if(first_image || base_frame_graphics_view->get_correct_bb())
	{
		try
		{
			if (enc::isColor(msg->encoding))
				cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
			else
				cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		QImage image;

		if (enc::isColor(msg->encoding))
		{
			image = QImage((const unsigned char*)(cv_ptr->image.data),cv_ptr->image.cols,cv_ptr->image.rows,QImage::Format_RGB888);
			//image.rgbSwapped();
		}
		else
		{
			image = QImage((const unsigned char*)(cv_ptr->image.data),cv_ptr->image.cols,cv_ptr->image.rows,QImage::Format_Indexed8);
		}

		emit sig_image_received(image);

		if(first_image)
			first_image = false;
	}
}

void BaseFrame::tracked_objectCB(const tld_msgs::BoundingBoxConstPtr & msg)
{
    if(msg->width && msg->height)
        first_image = true;
    else
        first_image = false;

	QRectF rect(msg->x,msg->y,msg->width,msg->height);
	emit sig_tracked_object_changed(rect);
	emit sig_confidence_changed((int)(msg->confidence*100));
}

void BaseFrame::fps_trackerCB(const std_msgs::Float32ConstPtr & msg)
{
	emit sig_fps_tracker_changed((int)msg->data);
}

void BaseFrame::clear_background()
{
	std_msgs::Char cmd;
	cmd.data = 'b';
	pub2.publish(cmd);
	qDebug() << "Clearing Background";
}

void BaseFrame::clear_and_stop_tracking()
{
	std_msgs::Char cmd;
	cmd.data = 'c';
	pub2.publish(cmd);
	qDebug() << "Clearing and stop tracking";
}

void BaseFrame::toggle_learning()
{
	std_msgs::Char cmd;
	cmd.data = 'l';
	pub2.publish(cmd);
	qDebug() << "Toggle learning";
}

void BaseFrame::alternating_mode()
{
	std_msgs::Char cmd;
	cmd.data = 'a';
	pub2.publish(cmd);            
	qDebug() << "Alternating mode";
}

void BaseFrame::export_model()
{
	QString res = QFileDialog::getSaveFileName(this, tr("Choose a model file name"), "/", tr("All (*)"));
	if(!res.isNull())
	{
		ros::NodeHandle nh;
		nh.setParam("/ros_tld_tracker_node/modelExportFile", res.toStdString());
	}
	else
	{
		// The user pressed cancel or closed the dialog.
	}

	std_msgs::Char cmd;
	cmd.data = 'e';
	pub2.publish(cmd);
	qDebug() << "Exporting model : " << res;
}

void BaseFrame::import_model()
{
	QString res = QFileDialog::getOpenFileName(this, tr("Choose a model file to open"), "/", tr("All (*)"));
	if(!res.isNull())
	{
		ros::NodeHandle nh;
		nh.setParam("/ros_tld_tracker_node/modelImportFile", res.toStdString());
	}
	else
	{
		// The user pressed cancel or closed the dialog.
	}

	std_msgs::Char cmd;
	cmd.data = 'i';
	pub2.publish(cmd);
	qDebug() << "Importing model : " << res;
}

void BaseFrame::reset()
{
	std_msgs::Char cmd;
	cmd.data = 'r';
	pub2.publish(cmd);
	qDebug() << "Reset";
}
