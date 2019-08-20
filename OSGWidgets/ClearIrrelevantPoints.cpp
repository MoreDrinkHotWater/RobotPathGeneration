//
// Created by zhihui on 8/5/19.
//
#include <iostream>

#include <QObject>
#include <QThread>

#include "ClearIrrelevantPoints.h"

void ClearIrrelevantPoints::clearIrrelevantPointsSlot(osg::Switch *rootnode, osgViewer::View *mainview, bool isactive) {
    this->mainView = mainview;
    this->rootNode = rootnode;

    std::cout << "ClearIrrelevantPoints thread: " << QThread::currentThreadId() << std::endl;

    std::cout << "ClearIrrelevantPoints :success" << std::endl;
    if (rootNode == nullptr) {
        std::cout << "get failed" << std::endl;
    } else {

        osg::ref_ptr<ClearPointsEvents> clearPointsEvents = new ClearPointsEvents(rootNode);

        std::cout<<"isActive: "<<isactive<<std::endl;

        if(isactive)
        {
            mainView->addEventHandler(clearPointsEvents);
        }
        else
        {
            std::cout<<"----------------"<<std::endl;

            std::cout<<"remove the event"<<std::endl;

            std::cout<<"----------------"<<std::endl;

            // clearPointsEvents->clean();

            mainView->removeEventHandler(clearPointsEvents);
        }
    }
}
