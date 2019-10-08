//
// Created by zhihui on 8/5/19.
//
#include <iostream>

#include <QObject>
#include <QThread>
#include <QMessageBox>

#include "ClearIrrelevantPoints.h"

ClearIrrelevantPoints::~ClearIrrelevantPoints()
{
    std::cout<<"~ClearIrrelevantPoints"<<std::endl;
}

void ClearIrrelevantPoints::clearIrrelevantPointsSlot(osg::ref_ptr<osg::Switch> rootnode, osg::ref_ptr<osgViewer::View> mainview, bool isactive) {
    this->mainView = mainview;
    this->rootNode = rootnode;

    clearPointsEvents = new ClearPointsEvents(rootNode);

    std::cout << "ClearIrrelevantPoints thread: " << QThread::currentThreadId() << std::endl;

    std::cout << "ClearIrrelevantPoints :success" << std::endl;
    if (rootNode == nullptr) {
        std::cout << "get failed" << std::endl;
    }
    else{

        std::cout<<"isActive: "<<isactive<<std::endl;

        mainview->setEventQueue(new osgGA::EventQueue);

        if(isactive)
        {
            std::cout<<"addEvent success"<<std::endl;

            mainView->addEventHandler(clearPointsEvents);
        }
    }
}



void ClearIrrelevantPoints::removeEvent()
{
    if (rootNode == nullptr) {
        std::cout << "get failed" << std::endl;
    }
    else {

        std::cout << "remove clearPointsEvents success" << std::endl;

        mainView->removeEventHandler(clearPointsEvents);
    }
}

void ClearIrrelevantPoints::PolygonclearIrrelevantPointsSlot(osg::ref_ptr<osg::Switch> rootnode, osg::ref_ptr<osgViewer::View> mainview, bool isactive) {
    this->mainView = mainview;
    this->rootNode = rootnode;

    polygonClearPointsEvents = new PolygonClearPointsEvents(rootNode);

    std::cout << "PolygonClearPointsEvents thread: " << QThread::currentThreadId() << std::endl;

    std::cout << "PolygonClearPointsEvents :success" << std::endl;
    if (rootNode == nullptr) {
        std::cout << "get failed" << std::endl;
    }
    else{

        std::cout<<"isActive: "<<isactive<<std::endl;

        mainview->setEventQueue(new osgGA::EventQueue);

        if(isactive)
        {
            std::cout<<"addEvent success"<<std::endl;

            mainView->addEventHandler(polygonClearPointsEvents);
        }
    }
}


void ClearIrrelevantPoints::removePolygonEvent()
{
    if (rootNode == nullptr) {
        std::cout << "get failed" << std::endl;
    }
    else {
        std::cout << "remove polygonClearPointsEvents success" << std::endl;

        mainView->removeEventHandler(polygonClearPointsEvents);
    }
}


