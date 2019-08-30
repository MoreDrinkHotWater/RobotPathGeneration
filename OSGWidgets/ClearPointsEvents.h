//
// Created by zhihui on 8/6/19.
//

#ifndef DEMO_TWO_CLEARPOINTSEVENTS_H
#define DEMO_TWO_CLEARPOINTSEVENTS_H

#include <osgGA/GUIEventHandler>
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "JudgeGroundPoint.h"
#include "VertexVisitor.h"

#include <osg/Switch>
#include <osgViewer/View>
#include <osgDB/ReadFile>
#include <QThread>

#include <iostream>
#include <stack>

namespace osgViewer{
    class View;
}
class ClearPointsEvents : public osgGA::GUIEventHandler, public QObject{

public:
    explicit ClearPointsEvents(osg::ref_ptr<osg::Switch> &rootNode, QObject *parent = nullptr);
    ~ClearPointsEvents() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

    void clean();

    std::vector<osg::Vec3>::iterator iter_temp;

    std::vector<osg::Vec3>::iterator iter_ground;

    int size_tempNode;

    int size_groundNode;

    int size_buildingNode;

private:
    // 根结点
    osg::ref_ptr<osg::Switch> rootNode;

    osg::ref_ptr<osg::Switch> pointCloudNode;

    osg::ref_ptr<osg::Switch> virtualPlaneNode;
    // 地面结点
    osg::ref_ptr<osg::Switch> groundNode;
    // 非地面结点
    osg::ref_ptr<osg::Switch> buildingNode;
    // 其他结点
    osg::ref_ptr<osg::Switch> otherNode;
    // 临时结点
    osg::ref_ptr<osg::Switch> tempNode;

//    // 回退结点
//    osg::ref_ptr<osg::Switch> roolbackNode;

    // 清除的叶子结点
    osg::ref_ptr<osg::Geode> tempIrrelevantGeode;
    // 清除的叶子结点之后 剩下的点数据
    osg::ref_ptr<osg::Geode> otherPointsGeode;

    // 选择的点
    std::vector<std::pair<size_t, osg::Vec3d>> selectedPoints;

    float x, y;

    double vpW,vpH;

    std::vector<osg::Vec3d> points;

    std::stack<osg::ref_ptr<osg::Switch>> savetempNodeStack;

    // std::stack<QMap<int, osg::Vec3>> MapStack;

    osg::ref_ptr<osg::Switch> node = new osg::Switch;

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::ref_ptr<osg::Geometry> test_geom = new osg::Geometry;


private:

    double calculateApproximateZ(const std::vector<osg::Vec3d> &points) const;

    void roolback();

//    JudgeGroundPoint *judgeGroundPoint;
//
//    QThread judgeGroundPointThread;

};
#endif //DEMO_TWO_CLEARPOINTSEVENTS_H
