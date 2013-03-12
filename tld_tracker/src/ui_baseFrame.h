/********************************************************************************
** Form generated from reading UI file 'BaseFrame.ui'
**
** Created: Tue Sep 4 16:38:50 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASEFRAME_H
#define UI_BASEFRAME_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLCDNumber>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "base_frame_graphics_view.hpp"

QT_BEGIN_NAMESPACE

class Ui_BaseFrame
{
public:
    QHBoxLayout *horizontalLayout;
    QHBoxLayout *horizontalLayout_1;
    BaseFrameGraphicsView *base_frame_graphics_view;
    QVBoxLayout *verticalLayout;
    QLabel *confidence_label;
    QLabel *fps_acq_label;
    QLabel *fps_tracker_label;
    QProgressBar *confidence_bar;
    QLCDNumber *lcd_fps_acq;
    QLCDNumber *lcd_fps_tracker;    
    QSpacerItem *verticalSpacer;
    QPushButton *learning_button;
    QPushButton *importing_button;
    QPushButton *exporting_button;
    QPushButton *alternating_button;
    QPushButton *stop_tracking_button;
    QPushButton *background_reset_button;
    QPushButton *reset_button;

    void setupUi(QWidget *BaseFrame)
    {
        if (BaseFrame->objectName().isEmpty())
            BaseFrame->setObjectName(QString::fromUtf8("BaseFrame"));
        BaseFrame->setLocale(QLocale(QLocale::English, QLocale::Canada));
        horizontalLayout = new QHBoxLayout(BaseFrame);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        base_frame_graphics_view = new BaseFrameGraphicsView(BaseFrame);
        base_frame_graphics_view->setMinimumSize(640, 480);
        base_frame_graphics_view->setObjectName(QString::fromUtf8("base_frame_graphics_view"));

        horizontalLayout->addWidget(base_frame_graphics_view);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        confidence_label = new QLabel(BaseFrame);
        confidence_label->setObjectName(QString::fromUtf8("confidence_label"));
        confidence_label->setLocale(QLocale(QLocale::English, QLocale::Canada));

        verticalLayout->addWidget(confidence_label);

        horizontalLayout_1 = new QHBoxLayout();
        horizontalLayout_1->setObjectName(QString::fromUtf8("horizontalLayout_1"));

        confidence_bar = new QProgressBar(BaseFrame);
        confidence_bar->setObjectName(QString::fromUtf8("confidence_bar"));
        confidence_bar->setMaximum(100);
        confidence_bar->setValue(50);
        confidence_bar->setTextVisible(false);
        confidence_bar->setOrientation(Qt::Vertical);

        horizontalLayout_1->addWidget(confidence_bar);
        verticalLayout->addLayout(horizontalLayout_1);

        fps_acq_label = new QLabel(BaseFrame);
        fps_acq_label->setObjectName(QString::fromUtf8("fps_acq_label"));
        fps_acq_label->setLocale(QLocale(QLocale::English, QLocale::Canada));

        verticalLayout->addWidget(fps_acq_label);

        lcd_fps_acq = new QLCDNumber(BaseFrame);
        lcd_fps_acq->setObjectName(QString::fromUtf8("lcd_fps_acq"));
        lcd_fps_acq->setSegmentStyle(QLCDNumber::Flat);

        verticalLayout->addWidget(lcd_fps_acq);

        fps_tracker_label = new QLabel(BaseFrame);
        fps_tracker_label->setObjectName(QString::fromUtf8("fps_tracker_label"));
        fps_tracker_label->setLocale(QLocale(QLocale::English, QLocale::Canada));

        verticalLayout->addWidget(fps_tracker_label);

        lcd_fps_tracker = new QLCDNumber(BaseFrame);  
        lcd_fps_tracker->setObjectName(QString::fromUtf8("lcd_fps_tracker"));
        lcd_fps_tracker->setSegmentStyle(QLCDNumber::Flat);
  
        verticalLayout->addWidget(lcd_fps_tracker);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        learning_button = new QPushButton(BaseFrame);
        learning_button->setObjectName(QString::fromUtf8("learning_button"));

        verticalLayout->addWidget(learning_button);

        importing_button = new QPushButton(BaseFrame);
        importing_button->setObjectName(QString::fromUtf8("importing_button"));

        verticalLayout->addWidget(importing_button);

        exporting_button = new QPushButton(BaseFrame);
        exporting_button->setObjectName(QString::fromUtf8("exporting_button"));

        verticalLayout->addWidget(exporting_button);

        alternating_button = new QPushButton(BaseFrame);
        alternating_button->setObjectName(QString::fromUtf8("alternating_button"));

        verticalLayout->addWidget(alternating_button);

        stop_tracking_button = new QPushButton(BaseFrame);
        stop_tracking_button->setObjectName(QString::fromUtf8("stop_tracking_button"));

        verticalLayout->addWidget(stop_tracking_button);

        background_reset_button = new QPushButton(BaseFrame);
        background_reset_button->setObjectName(QString::fromUtf8("background_reset_button"));

        verticalLayout->addWidget(background_reset_button);

        reset_button = new QPushButton(BaseFrame);
        reset_button->setObjectName(QString::fromUtf8("reset_button"));

        verticalLayout->addWidget(reset_button);


        horizontalLayout->addLayout(verticalLayout);


        retranslateUi(BaseFrame);

        QMetaObject::connectSlotsByName(BaseFrame);
    } // setupUi

    void retranslateUi(QWidget *BaseFrame)
    {
        BaseFrame->setWindowTitle(QApplication::translate("BaseFrame", "TLD Gui", 0, QApplication::UnicodeUTF8));
        confidence_label->setText(QApplication::translate("BaseFrame", "Tracker's confidence :", 0, QApplication::UnicodeUTF8));
        fps_acq_label->setText(QApplication::translate("BaseFrame", "FPS acquisition :", 0, QApplication::UnicodeUTF8));
        fps_tracker_label->setText(QApplication::translate("BaseFrame", "FPS tracker :", 0, QApplication::UnicodeUTF8));
        learning_button->setText(QApplication::translate("BaseFrame", "Toggle learning", 0, QApplication::UnicodeUTF8));
        importing_button->setText(QApplication::translate("BaseFrame", "Importing model", 0, QApplication::UnicodeUTF8));
        exporting_button->setText(QApplication::translate("BaseFrame", "Exporting model", 0, QApplication::UnicodeUTF8));
        alternating_button->setText(QApplication::translate("BaseFrame", "Alternating mode", 0, QApplication::UnicodeUTF8));
        stop_tracking_button->setText(QApplication::translate("BaseFrame", "Start/Stop tracking", 0, QApplication::UnicodeUTF8));
        background_reset_button->setText(QApplication::translate("BaseFrame", "Reset background", 0, QApplication::UnicodeUTF8));
        reset_button->setText(QApplication::translate("BaseFrame", "Reset", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class BaseFrame: public Ui_BaseFrame {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASEFRAME_H
