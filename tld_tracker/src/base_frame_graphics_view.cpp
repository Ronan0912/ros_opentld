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
 * base_frame_graphics_view.cpp
 *
 *  Created on: May 17, 2012
 *      Author: Ronan Chauvin
 */

#include <iostream>

#include "base_frame_graphics_view.hpp"

BaseFrameGraphicsView::BaseFrameGraphicsView(QWidget * parent) : QGraphicsView(parent)
{
	m_scene = new QGraphicsScene(this);
	this->setScene(m_scene);

	m_item_pixmap = m_scene->addPixmap(QPixmap("../../res/opentld.png"));
	m_item_rect = new QGraphicsRectItem();
	m_item_rect->setFlag(QGraphicsItem::ItemIsMovable);
	m_item_rect->setFlag(QGraphicsItem::ItemIsSelectable);

	m_pen = new QPen();
	m_brush = new QBrush(Qt::SolidPattern);

	m_pen->setColor(QColor(0,0,255));
	m_brush->setColor(QColor(255,255,255,64));
	m_item_rect->setPen(*m_pen);
	m_item_rect->setBrush(*m_brush);

	m_scene->addItem(m_item_rect);

	this->setCursor(Qt::CrossCursor);

	correct_bb = false;
	drag = false;
}

BaseFrameGraphicsView::BaseFrameGraphicsView(QGraphicsScene * scene, QWidget * parent) : QGraphicsView(scene,parent)
{
	m_scene = scene;
	this->setScene(m_scene);

	m_item_pixmap = m_scene->addPixmap(QPixmap("../../res/opentld.png"));
	m_item_rect = new QGraphicsRectItem(0,0,0,0);
	m_item_rect->setFlag(QGraphicsItem::ItemIsMovable);
	m_item_rect->setFlag(QGraphicsItem::ItemIsSelectable);

	m_pen = new QPen();
	m_brush = new QBrush(Qt::SolidPattern);

	m_pen->setColor(QColor(0,0,255));
	m_brush->setColor(QColor(255,255,255,64));
	m_item_rect->setPen(*m_pen);
	m_item_rect->setBrush(*m_brush);

	m_scene->addItem(m_item_rect);

	correct_bb = false;
	drag = false;
}

BaseFrameGraphicsView::~BaseFrameGraphicsView() 
{
	delete m_pen;
	delete m_brush;
}

QGraphicsRectItem * BaseFrameGraphicsView::get_bb() const
{
	return m_item_rect;
}

bool BaseFrameGraphicsView::get_correct_bb()
{
	return correct_bb;
}

QPen * BaseFrameGraphicsView::get_pen() const
{
	return m_pen;
}

QBrush * BaseFrameGraphicsView::get_brush() const
{
	return m_brush;
}

void BaseFrameGraphicsView::image_received(const QImage & img)
{
	m_item_pixmap->setPixmap(QPixmap::fromImage(img));
}

void BaseFrameGraphicsView::tracked_objet_changed(const QRectF & rect)
{
  //The tracker node sent a bounding box
  if(rect.width() || rect.height())
  {
    if(!this->correct_bb)
    {
      this->correct_bb = true;
      this->setCursor(Qt::ArrowCursor);
    } 
  }
  //The tracker node sent a bad bouding box
  else
  {
    if(this->correct_bb)
    {
      this->correct_bb = false;
      this->setCursor(Qt::CrossCursor);  
    }
  }

  this->m_item_rect->setRect(rect);        
}

void BaseFrameGraphicsView::resizeEvent(QResizeEvent * event)
{
  fitInView(this->sceneRect(), Qt::KeepAspectRatio);
  QGraphicsView::resizeEvent(event);
}

void BaseFrameGraphicsView::mousePressEvent(QMouseEvent * event)
{
  float scale, offset_x, offset_y;
  this->computeScaleOffsets(scale, offset_x, offset_y);

  qDebug() << "Press X : " << (event->pos().x()-offset_x)/scale << " Y : " << (event->pos().y()-offset_y)/scale;

  if(event->button() == Qt::LeftButton)
  {
    if(!correct_bb && !drag && !m_item_rect->isSelected())
    {
      point.setX((event->pos().x()-offset_x)/scale);
      point.setY((event->pos().y()-offset_y)/scale);

      drag = true;
    }
  }
  QGraphicsView::mousePressEvent(event);
}

void BaseFrameGraphicsView::mouseMoveEvent(QMouseEvent * event)
{
  float scale, offset_x, offset_y;
  this->computeScaleOffsets(scale, offset_x, offset_y);

  //    qDebug() << "Move X : " << (event->pos().x()-offset_x)/scale << " Y : " << (event->pos().y()-offset_y)/scale;

  if(!correct_bb && drag && !m_item_rect->isSelected())
  {
    m_item_rect->setRect(point.x(), point.y(), ((event->pos().x()-offset_x)/scale) - point.x(), ((event->pos().y()-offset_y)/scale) - point.y());
    this->update();
  }
  QGraphicsView::mouseMoveEvent(event);
}

void BaseFrameGraphicsView::mouseReleaseEvent(QMouseEvent * event)
{
  float scale, offset_x, offset_y;
  this->computeScaleOffsets(scale, offset_x, offset_y);

  qDebug() << "Release X : " << (event->pos().x()-offset_x)/scale << " Y : " << (event->pos().y()-offset_y)/scale;

  if(event->button() == Qt::LeftButton)
  {
    if(!correct_bb && drag  && !m_item_rect->isSelected())
    {
      QRectF rect = m_item_rect->rect();
      emit sig_bb_set(rect);
      rect.normalized();
      drag = false;
    }
  }
  QGraphicsView::mouseReleaseEvent(event);
}

/* Mathieu's function */
void BaseFrameGraphicsView::computeScaleOffsets(float & scale, float & offsetX, float & offsetY) const
{
  scale = 1.0f;
  offsetX = 0.0f;
  offsetY = 0.0f;

  float w = m_item_pixmap->pixmap().width();
  float h = m_item_pixmap->pixmap().height();
  float widthRatio = float(this->rect().width()) / w;
  float heightRatio = float(this->rect().height()) / h;

  //printf("w=%f, h=%f, wR=%f, hR=%f, sW=%d, sH=%d\n", w, h, widthRatio, heightRatio, this->rect().width(), this->rect().height());
  if(widthRatio < heightRatio)
  {
    scale = widthRatio;
  }
  else
  {
    scale = heightRatio;
  }

  //printf("ratio=%f\n",ratio);

  w *= scale;
  h *= scale;

  if(w < this->rect().width())
  {
    offsetX = (this->rect().width() - w)/2.0f;
  }
  if(h < this->rect().height())
  {
    offsetY = (this->rect().height() - h)/2.0f;
  }
  //printf("offsetX=%f, offsetY=%f\n",offsetX, offsetY);
}

