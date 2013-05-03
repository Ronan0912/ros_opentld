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
 * base_frame_graphics_view.hpp
 *
 *  Created on: May 17, 2012
 *      Author: Ronan Chauvin
 */

#ifndef BASE_FRAME_GRAPHICS_VIEW_H
#define BASE_FRAME_GRAPHICS_VIEW_H

#include <QtGui/QApplication>

#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QGraphicsItem>

#include <QtGui/QMouseEvent>
#include <QtGui/QResizeEvent>

#include <QtGui/QImage>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QColor>
#include <QtCore/QPoint>
#include <QtCore/QRectF>
#include <QtCore/QDebug>

class BaseFrameGraphicsView : public QGraphicsView
{   
	Q_OBJECT

	public:
		BaseFrameGraphicsView(QWidget * parent);
		BaseFrameGraphicsView(QGraphicsScene * scene = NULL, QWidget * parent = NULL);
		~BaseFrameGraphicsView();
		QGraphicsRectItem * get_bb() const;
		bool get_correct_bb();
		QPen * get_pen() const;
		QBrush * get_brush() const;

	protected:
		void resizeEvent(QResizeEvent * event);
		void mousePressEvent(QMouseEvent * event);
		void mouseMoveEvent(QMouseEvent * event);
		void mouseReleaseEvent(QMouseEvent * event);

	private:
		QPoint point;
		bool correct_bb;
		bool drag;
		QGraphicsPixmapItem * m_item_pixmap;
		QGraphicsRectItem * m_item_rect;
		QGraphicsScene * m_scene;
		QPen * m_pen;
		QBrush * m_brush;

		/* Mathieu's function */
		void computeScaleOffsets(float & scale, float & offsetX, float & offsetY) const;

	public slots:
		void image_received(const QImage & img);
		void tracked_objet_changed(const QRectF & rect);

signals:
		void sig_bb_set(QRectF rect);
};

#endif
