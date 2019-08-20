//
// Created by zhihui on 8/13/19.
//

#ifndef HDMAPSFORROBOT_JUDGEGROUNDPOINT_H
#define HDMAPSFORROBOT_JUDGEGROUNDPOINT_H

#include <QObject>

#include <iostream>

#include <osg/Vec3>

#include <QThread>

#include "VertexVisitor.h"

class JudgeGroundPoint : public QObject
{
    Q_OBJECT

public:

    explicit JudgeGroundPoint(std::vector<osg::Vec3>::iterator iter, std::vector<osg::Vec3>::iterator iterat, int size_tempNode, int size_groundNode, VertexVisitor vtea);

public:
    osg::ref_ptr<osg::Vec3Array> return_Vec3Array;
};


#endif //HDMAPSFORROBOT_JUDGEGROUNDPOINT_H
