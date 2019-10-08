//
// Created by zhihui on 9/3/19.
//

#include "PolygonClearPointsEvents.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VertexVisitor.h"
#include "JudgeGroundPoint.h"
#include <cmath>
#include "../Common/DataStructure.h"
#include "../Common/VectorMapSingleton.h"

#include <osg/Switch>
#include <osgViewer/View>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/NodeVisitor>
#include <QObject>
#include <QtCore/QMap>
#include <opencv2/opencv.hpp>
#include <QMessageBox>


PolygonClearPointsEvents::PolygonClearPointsEvents(osg::ref_ptr<osg::Switch> &rootNode, QObject *parent):
        QObject(parent),
        rootNode(rootNode),
        pointCloudNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointCloudNodeName))),
        tempNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, tempNodeName))),
        groundNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, groundNodeName))),
        buildingNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, buildingNodeName))),
        otherNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, otherNodeName))),
        virtualPlaneNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,virtualPlaneNodeName))),
        clearLineNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, clearLineNodeName))),
        saveRectNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, saveRectNodeName))),

        tempIrrelevantGeode(nullptr),
        otherPointsGeode(nullptr),

        size_tempNode(0),
        size_groundNode(0),
        size_buildingNode(0),
        size_lastRectNode(0),
        size_saveRectNode(0),

        firstVertex(0,0,0),
        tempPoint(0,0,0),

        x(0),
        y(0)
{

}

PolygonClearPointsEvents::~PolygonClearPointsEvents()
{
    std::cout<<"~ClearPointsEvents"<<std::endl;

    clean();
}

bool PolygonClearPointsEvents::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {


        case(osgGA::GUIEventAdapter::KEYDOWN):
        {

            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_R)
            {
                std::cout<<"user click key: R "<<std::endl;

                roolback();
            }

            // ctrl 生成 map.png
            if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Control_L)
            {
                std::cout<<"user click KEY_Control_L"<<std::endl;

                buildingNode->setNodeMask(1);

                std::cout<<"-----------------------------the bulidingNode is open!-------------------------"<<std::endl;

                QMap<int, osg::Vec3> GroundNode_map;
                QMap<int, osg::Vec3> BulidingNode_map;

                std::vector<osg::Vec3>::iterator Iter_ground;
                std::vector<osg::Vec3>::iterator Iter_buliding;
                std::vector<osg::Vec3>::iterator Iter_saveRect;

                VertexVisitor Vtea_ground;
                VertexVisitor Vtea_buliding;
                VertexVisitor Vtea_saveRect;

                otherNode->accept(Vtea_ground);

                buildingNode->accept(Vtea_buliding);

                size_groundNode = Vtea_ground.extracted_verts->size();
                Iter_ground = Vtea_ground.extracted_verts->begin();

                size_buildingNode = Vtea_buliding.extracted_verts->size();
                Iter_buliding = Vtea_buliding.extracted_verts->begin();

                for (int j = 0; j < size_groundNode; ++j) {
                    osg::Vec3 temp_point = osg::Vec3(Iter_ground->x(),Iter_ground->y(),Iter_ground->z());

                    GroundNode_map.insert(j,temp_point);

                    ++Iter_ground;
                }

                if(saveRectNode && saveRectNode->getNumChildren() != 0)
                {
                    saveRectNode->accept(Vtea_saveRect);

                    size_saveRectNode = Vtea_saveRect.extracted_verts->size();
                    Iter_saveRect = Vtea_saveRect.extracted_verts->begin();

                    for (int j = size_groundNode; j < size_groundNode + size_saveRectNode; ++j) {
                        osg::Vec3 temp_point = osg::Vec3(Iter_saveRect->x(),Iter_saveRect->y(),Iter_saveRect->z());

                        GroundNode_map.insert(j,temp_point);

                        ++Iter_saveRect;
                    }

                    std::cout<<"saveRect.size: "<<GroundNode_map.size()<<std::endl;
                }

                std::cout<<"groundNode_map.size: "<<GroundNode_map.size()<<std::endl;

                for (int j = 0; j < size_buildingNode; ++j) {
                    osg::Vec3 temp_point = osg::Vec3(Iter_buliding->x(),Iter_buliding->y(),Iter_buliding->z());

                    BulidingNode_map.insert(j,temp_point);

                    ++Iter_buliding;
                }

                std::cout<<"BulidingNode_map.size: "<<BulidingNode_map.size()<<std::endl;

                QMap<int, osg::Vec3>::iterator Iter_groundMap = GroundNode_map.begin();
                QMap<int, osg::Vec3>::iterator Iter_bulidingMap = BulidingNode_map.begin();

                double min_x = FLT_MAX;
                double max_x = FLT_MIN;
                double min_y = FLT_MAX;
                double max_y = FLT_MIN;

                for( ; Iter_bulidingMap != BulidingNode_map.end();) {

                    if(Iter_bulidingMap == BulidingNode_map.end())
                    {
                        std::cout<<"--------------Iter_bulidingMap == BulidingNode_map.end()-------------"<<std::endl;
                    }

                    if (min_x > Iter_bulidingMap.value().x()) {
                        min_x = Iter_bulidingMap.value().x();
                    }
                    if (max_x < Iter_bulidingMap.value().x()) {
                        max_x = Iter_bulidingMap.value().x();
                    }
                    if (min_y > Iter_bulidingMap.value().y()) {
                        min_y = Iter_bulidingMap.value().y();
                    }
                    if (max_y < Iter_bulidingMap.value().y()) {
                        max_y = Iter_bulidingMap.value().y();
                    }
                    ++Iter_bulidingMap;
                }

                min_x = int(min_x) - 1;
                min_y = int(min_y) - 1;

                float dim = 0.2;
                int Col = std::ceil((max_x - min_x) / dim);
                int Row = std::ceil((max_y - min_y) / dim);

                std::cout<<"Row: "<<Row<<std::endl;
                std::cout<<"Col: "<<Col<<std::endl;

                // 单通道： CV_8UC1 可以创建----8位无符号的单通道---灰度图片
                cv::Mat img = cv::Mat::zeros(Row, Col, CV_8UC1);
                for (int i = 0; i < Row; i++) {
                    for (int j = 0; j < Col; j++) {
                        img.at<uchar>(i, j) = 127;
                    }
                }

                // 非地面设置成黑色
                Iter_bulidingMap = BulidingNode_map.begin();

                for (; Iter_bulidingMap != BulidingNode_map.end() ;) {

                    if(Iter_bulidingMap == BulidingNode_map.end())
                    {
                        std::cout<<"--------------Iter_bulidingMap == BulidingNode_map.end()-------------"<<std::endl;
                    }

                    int lx = static_cast<int>((Iter_bulidingMap.value().x() - min_x) / dim);
                    int ly = static_cast<int>((Iter_bulidingMap.value().y() - min_y) / dim);

                    img.at<uchar>(ly, lx) = 0;

                    ++Iter_bulidingMap;
                }

                // 地面设置为白色
                for ( ; Iter_groundMap != GroundNode_map.end() ; )
                {
                    if(Iter_groundMap == GroundNode_map.end())
                    {
                        std::cout<<"--------------Iter_groundMap == GroundNode_map.end()-------------"<<std::endl;
                    }

                    int lx = static_cast<int>((Iter_groundMap.value().x() - min_x) / dim);
                    int ly = static_cast<int>((Iter_groundMap.value().y() - min_y) / dim);

                    // std::cout<<"ly: "<<ly<<"  "<<"lx: "<<lx<<std::endl;

                    img.at<uchar>(ly, lx) = 255;

                    ++Iter_groundMap;
                }

                cv::flip(img, img, 0);

                std::string map_filename = "map.png";
                std::string map_filename2 = "map.yaml";
                std::string map_folder = "/home/zhihui/TempPointCloud";

                cv::imwrite(map_folder + "/" + map_filename, img);

                std::ofstream ofs;
                ofs.open(map_folder + "/" + map_filename2, std::ios::out);
                ofs.setf(std::ios::fixed, std::ios::floatfield);
                ofs.precision(2);
                ofs << "image: " << map_filename << std::endl;
                ofs << "resolution: " << dim << std::endl;
                ofs << "origin: [" << min_x << ", " << min_y << ", 0.0]" << std::endl;
                ofs << "occupied_thresh: 0.1" << std::endl;
                ofs << "free_thresh: 0.05" << std::endl;
                ofs << "negate: 0" << std::endl;
                ofs.close();

                buildingNode->setNodeMask(0);

                std::cout << min_x << " " << min_y << std::endl;
            }

            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape) {
                clean();
                return true;
            }

            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_End)
            {

                std::cout << "user click the key End" << std::endl;

                drawDone();

                return true;
            }

            return false;

        }
        // 鼠标移动 画线
        case (osgGA::GUIEventAdapter::MOVE): {

            x = ea.getX();
            y = ea.getY();

            if (selectedPoints.empty()) {
                return false;
            }

            // 打开 tempNode 方便下次继续编辑
            tempNode->setNodeMask(1);

            // 先判断当前选点是否是已有的点或者点云上的点
            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);
            bool hasAlready = false;
            osg::Vec3d curPoint = osg::Vec3d(0,0,0);
            if (picker->containsIntersections()) {

                auto intersections = picker->getIntersections();
                for (const auto &intersection: intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        curPoint = intersection.localIntersectionPoint;
                        hasAlready = true;
                        break;
                    }
                    if (childNode->getName() == "OtherPoints") {
                        curPoint = intersection.localIntersectionPoint;
                        hasAlready = true;
                        break;
                    }
                    if (childNode->getName() == "Sphere") {
                        hasAlready = true;
                        break;
                    }
                }
            }

            if (!hasAlready) {
                // 生成虚拟平面用于选点
                double vpW = 80, vpH = 80;
                osg::ref_ptr<osgUtil::PolytopeIntersector> vpPicker =
                        new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                                         x - vpW,
                                                         y - vpH,
                                                         x + vpW,
                                                         y + vpH);
                osgUtil::IntersectionVisitor vpIv(vpPicker);
                view->getCamera()->accept(vpIv);

                // 平面四边形顶点
                osg::ref_ptr<osg::Vec3Array> quVertices = new osg::Vec3Array;
                osg::ref_ptr<osg::Geode> quGeode = new osg::Geode;
                osg::ref_ptr<osg::Geometry> quGeom = new osg::Geometry;
                osg::ref_ptr<osg::Vec4Array> quColors = new osg::Vec4Array;

                if (vpPicker->containsIntersections()) {
                    std::vector<osg::Vec3d> points;
                    double approximateZ = 0;
                    auto intersections = vpPicker->getIntersections();
                    for (const auto &intersection : intersections) {
                        auto childNode = intersection.nodePath.back();
                        if (childNode->getName() == "CloudPoints") {
                            points.push_back(intersection.localIntersectionPoint);
                        }
                        if (childNode->getName() == "OtherPoints") {
                            points.push_back(intersection.localIntersectionPoint);
                        }
                    }
                    // 一个点不好画平面
                    if (points.size() < 2) {
                        return false;
                    }

//                    std::cout << "------------Debug-----------------" << std::endl;
//                    std::cout << "points'size: " << points.size() << std::endl;
//                    approximateZ = calculateApproximateZ(points);
//                    std::cout << "approximateZ: " << approximateZ << std::endl;
//                    std::cout << "------------Debug-----------------" << std::endl;

                    // 原始点X-Y坐标值
                    std::vector<std::pair<double, double>> vertices;
                    for (const auto &point : points) {
                        vertices.emplace_back(std::make_pair(point.x(), point.y()));
                    }

                    double minX = vertices[0].first;
                    double maxX = vertices[0].first;
                    double minY = vertices[0].second;
                    double maxY = vertices[0].second;

                    for (const std::pair<double, double> pair : vertices) {
                        if (pair.first < minX) {
                            minX = pair.first;
                        }
                        if (pair.first > maxX) {
                            maxX = pair.first;
                        }
                        if (pair.second < minY) {
                            minY = pair.second;
                        }
                        if (pair.second > maxY) {
                            maxY = pair.second;
                        }
                    }

//                    std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY
//                              << std::endl;

                    quVertices->push_back(osg::Vec3(minX, minY, approximateZ));
                    quVertices->push_back(osg::Vec3(minX, maxY, approximateZ));
                    quVertices->push_back(osg::Vec3(maxX, maxY, approximateZ));
                    quVertices->push_back(osg::Vec3(maxX, minY, approximateZ));


                    // add red color : 1.0f, 0.0f, 0.0f, 1.0f  white color : 1.0f, 0.0f, 0.0f, 1.0f
                    quColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));

                    quGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
                    quGeom->setVertexArray(quVertices);
                    quGeom->setColorArray(quColors);
                    quGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
                    quGeode->setName("Rect");
                    quGeode->addDrawable(quGeom);

                    // 设置透明度
                    osg::ref_ptr<osg::StateSet> stateset = quGeode->getOrCreateStateSet();
                    stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
                    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                    virtualPlaneNode->addChild(quGeode);

                    // 当前鼠标所选的点
                    w = 1.5, h = 1.5;
                    osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                            osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
                    osgUtil::IntersectionVisitor iv(picker);
                    view->getCamera()->accept(iv);
                    if (picker->containsIntersections()) {
                        intersections = picker->getIntersections();

                        for (const auto &intersection: intersections) {
                            curPoint = intersection.localIntersectionPoint;
                            break;
                        }
                        curPoint.z() = approximateZ;
                    } else {
                        virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
                        return false;
                    }
                } else {
                    return false;
                }
            }

            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

            if(curPoint == osg::Vec3d(0,0,0))
            {
                return false;
            }

            vertices->push_back(tempPoint);
            vertices->push_back(curPoint);

//            std::cout<<"--------------------------_Debug-------------------------"<<std::endl;
//            std::cout<<"tempPoint: "<<tempPoint.x()<<"  "<<tempPoint.y()<<"  "<<tempPoint.z()<<std::endl;
//            std::cout<<"curPoint: "<<curPoint.x()<<"  "<<curPoint.y()<<"  "<<curPoint.z()<<std::endl;
//            std::cout<<"--------------------------_Debug-------------------------"<<std::endl;

            if (tempIrrelevantGeode == nullptr) {
                tempIrrelevantGeode = new osg::Geode;
                tempIrrelevantGeode->setName("tempIrrelevantGeode");
                tempNode->addChild(tempIrrelevantGeode);
            } else {
                tempIrrelevantGeode->removeDrawables(0, tempIrrelevantGeode->getNumDrawables());
            }

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setName("tempIrrelevantGeode");
            tempIrrelevantGeode->addDrawable(geom);

            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));
            osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
            colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

            geom->setColorArray(colors.get());
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->setVertexArray(vertices.get());

            virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());

            return true;

        }

        case (osgGA::GUIEventAdapter::RELEASE): {

            // std::cout<<"the mouse: RELEASE"<<std::endl;

            if (x == ea.getX() && y == ea.getY()) {

                pick(ea, view);

            }
            return true;
        }

        default:return false;
    }
}

void PolygonClearPointsEvents::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {


    // 选点
    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

        std::cout<<"user click the Left Mouse! "<<std::endl;

        std::cout<<"x: "<<x<<"  "<<"y: "<<y<<std::endl;

        // 第一次求交得到点
        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);
        osg::Vec3d localPoint = osg::Vec3d(0,0,0);
        if (picker->containsIntersections()) {

            auto intersections = picker->getIntersections();
            for (const auto &intersection : intersections) {
                auto childNode = intersection.nodePath.back();

                if (childNode->getName() == "CloudPoints") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (selectedPoints.front() == localPoint) {
                            return;
                        }
                    }
                }
                if (childNode->getName() == "OtherPoints") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (selectedPoints.front() == localPoint) {
                            return;
                        }
                    }
                }

                if (childNode->getName() == "AddPoints") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (selectedPoints.front() == localPoint) {
                            return;
                        }
                    }
                }

                if (childNode->getName() == "Sphere") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (selectedPoints.front() == localPoint) {
                            return;
                        }
                    }
                }
            }
        }

        std::cout<<"localPoint: "<<localPoint.x()<<"  "<<localPoint.y()<<"  "<<localPoint.z()<<std::endl;

        if(localPoint != osg::Vec3d(0,0,0))
        {
            selectedPoints.push_back(localPoint);
        }
        else
        {
            return;
        }

        if(selectedPoints.size() == 1)
        {
            firstVertex = localPoint;
        }

        // 加入顶点到容器中
//        if(selectedPoints.size() != 1)
//        {
//            saveVertex.push_back(localPoint);
//        }

        saveVertex.push_back(localPoint);

        // std::cout<<"------------------------------------: "<<selectedPoints.size()<<std::endl;

        // 画点
        osg::ref_ptr<osg::Geode> nodeGeode = new osg::Geode;
        nodeGeode->setName("Sphere");

        osg::ref_ptr<osg::ShapeDrawable> nodeSphere = new osg::ShapeDrawable(
                new osg::Sphere(localPoint, 0.1f));
        // 蓝色
        nodeSphere->setColor(osg::Vec4(0.6, 0.3, 0.9, 1.0));
        nodeGeode->addDrawable(nodeSphere.get());
        clearLineNode->addChild(nodeGeode.get());

        tempPoint = localPoint;
        // 画线
        if(selectedPoints.size() == 2)
        {
                auto sPoint = selectedPoints.front();
                auto ePoint = tempPoint;

                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(osg::Vec3(sPoint.x(), sPoint.y(), sPoint.z()));
                vertices->push_back(osg::Vec3(ePoint.x(), ePoint.y(), ePoint.z()));

                osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
                // 蓝色
                colors->push_back(osg::Vec4(0.6, 0.3, 0.9, 1.0));

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setVertexArray(vertices.get());
                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));

                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->setName("Line");
                geode->addDrawable(geom);

                clearLineNode->addChild(geode);

                selectedPoints.pop_front();
        }
    }

    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {

        if(saveVertex.empty())
        {
            return;
        }

        std::cout << "user click the Right Mouse!" << std::endl;

        // 清除已画的点和线
        clearLineNode->removeChild(0,clearLineNode->getNumChildren());

        double minX, maxX;
        double minY, maxY;
        std::vector<double> compX;
        std::vector<double> compY;
        for(auto temp : saveVertex)
        {
//        std::cout<<"Vertex: "<<temp.x()<<"  "<<temp.y()<<"  "<<temp.z()<<std::endl;
            compX.push_back(temp.x());
            compY.push_back(temp.y());
        }

        minX = maxX = compX[0];
        minY = maxY = compY[0];

        for ( auto verX : compX) {
            if (verX < minX) {
                minX = verX;
            }
            if (verX > maxX) {
                maxX = verX;
            }
        }

        for( auto verY : compY)
        {
            if (verY < minY) {
                minY = verY;
            }
            if (verY > maxY) {
                maxY = verY;
            }
        }

        std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY
                  << std::endl;


        QMap<int, osg::Vec3> groundNode_map;
        QMap<int, osg::Vec3> saveRectNode_map;
        QMap<int, osg::Vec3> lastsaveRectNode_map;

        VertexVisitor vtea_ground;
        VertexVisitor vtea_lastRect;

        if(otherNode->getNumChildren() == 0)
        {
            groundNode->accept(vtea_ground);
        }
        else
        {
            otherNode->accept(vtea_ground);
            saveRectNode->accept(vtea_lastRect);
        }

        size_groundNode = vtea_ground.extracted_verts->size();
        size_lastRectNode = vtea_lastRect.extracted_verts->size();

        iter_ground = vtea_ground.extracted_verts->begin();
        iter_saveRect = vtea_lastRect.extracted_verts->begin();

        int number = 0;

        for (int j = 0; j < size_groundNode; ++j) {

            osg::Vec3 temp_point = osg::Vec3(iter_ground->x(),iter_ground->y(),iter_ground->z());

            // 不在矩形内
            if (iter_ground->x() < minX || iter_ground->x()> maxX || iter_ground->y() < minY || iter_ground->y() > maxY) {

                groundNode_map.insert(j,temp_point);

            }
            // 矩形内的点
            else
            {
                saveRectNode_map.insert(number, temp_point);

                ++number;
            }
            ++iter_ground;
        }

        // 地面点加入 saveRectNode 结点
        if(otherNode->getNumChildren() != 0)
        {
            for (int j = 0; j < size_lastRectNode; ++j) {
                osg::Vec3 temp_point = osg::Vec3(iter_saveRect->x(),iter_saveRect->y(),iter_saveRect->z());

                // 不在矩形内
                if (iter_saveRect->x() < minX || iter_saveRect->x()> maxX || iter_saveRect->y() < minY || iter_saveRect->y() > maxY) {

                    groundNode_map.insert(size_groundNode,temp_point);
                    ++size_groundNode;

                }
                // 矩形内的点
                else
                {
                    saveRectNode_map.insert(number, temp_point);

                    ++number;
                }
                ++iter_saveRect;
            }
        }

        std::cout<<"groundNode_map.size(): "<<groundNode_map.size()<<std::endl;

        std::cout<<"saveRectNode_map.size(): "<<saveRectNode_map.size()<<std::endl;

        std::vector<double >::iterator iter_compX;
        std::vector<double >::iterator iter_compY;

        QMap<int, osg::Vec3>::iterator iter_saveRectmap = saveRectNode_map.begin();

        std::vector<osg::Vec3> add_map;

        std::vector<osg::Vec3> roolback_vector;

        for (; iter_saveRectmap != saveRectNode_map.end() ;)
        {
            iter_compX = compX.begin();
            iter_compY = compY.begin();

            // 在矩形但  没在多边形的点
            if(pnpoly(saveVertex.size(),iter_compX,iter_compY,iter_saveRectmap->x(),iter_saveRectmap->y()) == 0)
            {
                add_map.push_back(iter_saveRectmap.value());
            } else
            {
                roolback_vector.push_back(iter_saveRectmap.value());
            }

            ++iter_saveRectmap;
        }

        // 防止用户疯狂按右键 所以加一个判断。 至少 3 个点才有平面
        if(saveVertex.size() > 2)
        {
            // 要回退的点云数据数组 进栈
            stack.push(roolback_vector);
        }
        else
        {
            return;
        }

        std::cout<<"add_map size: "<<add_map.size()<<std::endl;

        osg::ref_ptr<osg::Geode> addPointsGeode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> add_geom = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> add_coords = new osg::Vec3Array();
        addPointsGeode->setName("AddPoints");

        std::vector<osg::Vec3>::iterator iter_addMap ;

        for (iter_addMap = add_map.begin();  iter_addMap != add_map.end() ; ++iter_addMap) {

            add_coords->push_back(osg::Vec3(iter_addMap->x(), iter_addMap->y(), iter_addMap->z()));
        }

        // saveRectNode 始终只挂一个结点
        if(saveRectNode && saveRectNode->getNumChildren() != 0)
        {
            saveRectNode->removeChild(0,saveRectNode->getNumChildren());
        }

        add_geom->setVertexArray(add_coords.get());

        add_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(add_coords->size())));
        addPointsGeode->addDrawable(add_geom.get());

        // 要补的矩形 边角
        saveRectNode->addChild(addPointsGeode);

        // 已取得多边形的最大，最小X，Y。清除保存顶点的数组
        saveVertex.clear();

        otherPointsGeode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> other_geom = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> other_coords = new osg::Vec3Array();
        otherPointsGeode->setName("OtherPoints");

        QMap<int, osg::Vec3>::iterator iter_groundMap ;

        for (iter_groundMap = groundNode_map.begin();  iter_groundMap != groundNode_map.end() ; ++iter_groundMap) {

            other_coords->push_back(osg::Vec3(iter_groundMap->x(), iter_groundMap->y(), iter_groundMap->z()));
        }

        // 始终让 otherNode 只有一个孩子被渲染出来
        if(otherNode && otherNode->getNumChildren() != 0)
        {
            otherNode->removeChild(0,otherNode->getNumChildren());
        }

        other_geom->setVertexArray(other_coords.get());

        other_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(other_coords->size())));
        otherPointsGeode->addDrawable(other_geom.get());

        otherNode->addChild(otherPointsGeode);

        if(groundNode)
        {
            groundNode->setNodeMask(0);
        }
        // 全部点云结点
        if(pointCloudNode)
        {
            pointCloudNode->setNodeMask(0);
        }
        // 非地面结点
        if(buildingNode)
        {
            buildingNode->setNodeMask(0);
        }
        // 保存清除点云的结点
        if(tempNode)
        {
            tempNode->setNodeMask(0);
        }
        // 保存余下点云的结点
        otherNode->setNodeMask(1);
        saveRectNode->setNodeMask(1);

        clean();

        std::cout<<"------------------------------clear done------------------------"<<std::endl;

    }
}

// 取消画线
void PolygonClearPointsEvents::clean()
{
    if (tempIrrelevantGeode != nullptr) {
        tempIrrelevantGeode->removeDrawables(0, tempIrrelevantGeode->getNumDrawables());
        tempIrrelevantGeode = nullptr;
    }
    // tempNode 挂的临时的线 （move的线）
    tempNode->removeChildren(0, tempNode->getNumChildren());

    // add by li
    tempNode->setNodeMask(1);

    // clearLineNode 挂的是画的 点和线
    clearLineNode->removeChild(0,clearLineNode->getNumChildren());

    // 队列： 保存用户选择的点
    selectedPoints.clear();

    // 保存顶点的数组
    saveVertex.clear();

    // 重置 tempPoint
    osg::Vec3 vec;

    tempPoint = vec;
}

// 完成画线
void PolygonClearPointsEvents::drawDone()
{
    // 按 END 自动连起点！

    auto sPoint = tempPoint;
    auto ePoint = firstVertex;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(sPoint.x(), sPoint.y(), sPoint.z()));
    vertices->push_back(osg::Vec3(ePoint.x(), ePoint.y(), ePoint.z()));

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    // 蓝色
    colors->push_back(osg::Vec4(0.6, 0.3, 0.9, 1.0));

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->setColorArray(colors.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName("Line");
    geode->addDrawable(geom);

    clearLineNode->addChild(geode);

    if (tempIrrelevantGeode != nullptr) {
        tempIrrelevantGeode->removeDrawables(0, tempIrrelevantGeode->getNumDrawables());
        tempIrrelevantGeode = nullptr;
    }
    tempNode->removeChildren(0, tempNode->getNumChildren());

//    virtualPlaneNode->removeChild(0,virtualPlaneNode->getNumChildren());

    selectedPoints.clear();

    osg::Vec3 vec;

    tempPoint = vec;
}

void PolygonClearPointsEvents::roolback()
{
    std::cout<<stack.size()<<std::endl;

    if(stack.size() == 0)
    {
        std::cout<<"------------------------stack.size is 0 -------------------"<<std::endl;

        QMessageBox::information(nullptr, "Tip", "The stack can't roolback!");

        return;
    }

    osg::ref_ptr<osg::Geometry> other_geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> other_coords = new osg::Vec3Array();
    otherPointsGeode->setName("OtherPoints");

    // 采用map的遍历方法
    auto it = stack.top().begin();

    for (;  it != stack.top().end() ; ) {

        other_coords->push_back(osg::Vec3(it->x(), it->y(), it->z()));

        ++it;
    }

    // 始终让 otherNode 只有一个孩子被渲染出来
    if( otherNode->getNumChildren() != 0)
    {
        otherNode->removeChild(0,otherNode->getNumChildren());
    }

    other_geom->setVertexArray(other_coords.get());

    other_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(other_coords->size())));

    otherPointsGeode->addDrawable(other_geom.get());

    otherNode->addChild(otherPointsGeode);

    stack.pop();
}

int PolygonClearPointsEvents::pnpoly(int nvert, std::vector<double >::iterator vertx, std::vector<double >::iterator verty, double testx, double testy)
{
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
             (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
            c = !c;
    }
    return c;
}


