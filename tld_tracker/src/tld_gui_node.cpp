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

#include "base_frame.hpp"
#include <signal.h>
#include <QtGui/QApplication>
#include <ros/ros.h>

void termination_handler(int signum);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_tld_gui_node");
	ros::AsyncSpinner spinner(1);
	struct sigaction action;

	/* Set up the structure to specify the action */
	action.sa_handler = termination_handler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = 0;
	sigaction(SIGINT, &action, NULL);

	QApplication * app = new QApplication(argc, argv);
	BaseFrame * gui = new BaseFrame();
	gui->setMinimumSize(640,360);
	gui->show();

	spinner.start();
	app->exec();
	spinner.stop();

	delete gui;
	delete app;

	return 0;
}

void termination_handler(int signum)
{
	QApplication::quit();
}
