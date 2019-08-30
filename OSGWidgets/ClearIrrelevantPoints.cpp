//
// Created by zhihui on 8/5/19.
//
#include <iostream>

#include <QObject>
#include <QThread>

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

        // osg::ref_ptr<ClearPointsEvents> clearPointsEvents = new ClearPointsEvents(rootNode);

        if(isactive)
        {
            std::cout<<"addEvent success"<<std::endl;

            mainView->addEventHandler(clearPointsEvents);
        }
//        else
//        {
//            std::cout<<"----------------"<<std::endl;
//
//            std::cout<<"remove the event"<<std::endl;
//
//            std::cout<<"----------------"<<std::endl;
//
//            mainView->removeEventHandler(clearPointsEvents);
//        }
    }
}

void ClearIrrelevantPoints::removeEvent()
{
    if (rootNode == nullptr) {
        std::cout << "get failed" << std::endl;
    }
    else {

        std::cout << "removeEvent success" << std::endl;



        // clearPointsEvents = new ClearPointsEvents(rootNode);

        mainView->removeEventHandler(clearPointsEvents);
    }
}


