//
// Created by zhihui on 9/3/19.
//
#include <osgGA/GUIEventHandler>
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "JudgeGroundPoint.h"
#include "VertexVisitor.h"

#include <osg/Switch>
#include <osgViewer/View>
#include <osgDB/ReadFile>
#include <QThread>
#include <opencv/cv.hpp>

#include <iostream>
#include <stack>

namespace osgViewer{
    class View;
}
class PolygonClearPointsEvents : public osgGA::GUIEventHandler, public QObject{

public:
    explicit PolygonClearPointsEvents(osg::ref_ptr<osg::Switch> &rootNode, QObject *parent = nullptr);
    ~PolygonClearPointsEvents() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

    void clean();

    void drawDone();

    int pnpoly(int nvert, std::vector<double >::iterator vertx, std::vector<double >::iterator verty, double testx, double testy);

    std::vector<osg::Vec3>::iterator iter_ground;

    std::vector<osg::Vec3>::iterator iter_saveRect;

    int size_tempNode;

    int size_groundNode;

    int size_lastRectNode;

    int size_buildingNode;

    int size_saveRectNode;

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
    // 保存用户选择的顶点和画的线段
    osg::ref_ptr<osg::Switch> clearLineNode;

    // 保存用户要删除的矩形框数据
    osg::ref_ptr<osg::Switch> saveRectNode;

    // 清除的叶子结点
    osg::ref_ptr<osg::Geode> tempIrrelevantGeode;
    // 清除的叶子结点之后 剩下的点数据
    osg::ref_ptr<osg::Geode> otherPointsGeode;

    // 选择的点
    std::deque<osg::Vec3> selectedPoints;

    float x, y;

    osg::Vec3 tempPoint;

    osg::Vec3 firstVertex;

    std::vector<osg::Vec3> saveVertex;

    std::stack<std::vector<osg::Vec3>> stack;

    osg::ref_ptr<osg::Switch> node = new osg::Switch;

private:

    double calculateApproximateZ(const std::vector<osg::Vec3d> &points) const;

    void roolback();


};

