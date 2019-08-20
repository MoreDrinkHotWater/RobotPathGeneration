//
// Created by zhihui on 8/6/19.
//

#include "ClearPointsEvents.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VertexVisitor.h"
#include "JudgeGroundPoint.h"
#include <cmath>

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

ClearPointsEvents::ClearPointsEvents(osg::Switch *rootNode, QObject *parent):
        QObject(parent),
        rootNode(rootNode),
        pointCloudNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointCloudNodeName))),
        tempNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, tempNodeName))),
        groundNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, groundNodeName))),
        buildingNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, buildingNodeName))),
        otherNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, otherNodeName))),
        virtualPlaneNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                      virtualPlaneNodeName))),
        tempIrrelevantGeode(nullptr),
        otherPointsGeode(nullptr),

        size_tempNode(0),
        size_groundNode(0),
        size_buildingNode(0),

        // judgeGroundPoint(new JudgeGroundPoint(iter,iterat,size_tempNode,size_groundNode,vtea)),
        x(0),
        y(0),
        vpW(10),
        vpH(10)
{

}

ClearPointsEvents::~ClearPointsEvents()
{
    clean();
}

bool ClearPointsEvents::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {


        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Q)
            {
                if(vpW < 50)
                {
                    vpW+=5,vpH+=5;
                }

            }
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_E)
            {
                if(vpW > 10)
                {
                    vpW-=5,vpH-=5;
                }
            }

            // ctrl + R 生成 map.png
            if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Control_L)
            {
                std::cout<<"user click KEY_Control_L"<<std::endl;

                buildingNode->setNodeMask(1);

                QMap<int, osg::Vec3> GroundNode_map;
                QMap<int, osg::Vec3> BulidingNode_map;

                std::vector<osg::Vec3>::iterator Iter_ground;
                std::vector<osg::Vec3>::iterator Iter_buliding;


                VertexVisitor Vtea_ground;
                VertexVisitor Vtea_buliding;

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

                std::cout<<"groundNode_map.size: "<<GroundNode_map.size()<<std::endl;

                for (int j = 0; j < size_buildingNode; ++j) {
                    osg::Vec3 temp_point = osg::Vec3(Iter_buliding->x(),Iter_buliding->y(),Iter_buliding->z());

                    BulidingNode_map.insert(j,temp_point);

                    ++Iter_buliding;
                }

                std::cout<<"BulidingNode_map.size: "<<BulidingNode_map.size()<<std::endl;

                QMap<int, osg::Vec3>::iterator Iter_groundMap = GroundNode_map.begin();
                QMap<int, osg::Vec3>::iterator Iter_bulidingMap = BulidingNode_map.begin();

//                for(int p = 0 ; p<10; p++)
//                {
//                    std::cout<<Iter_groundMap.value().x()<<"  "<<Iter_groundMap.value().y()<<"  "<<Iter_groundMap.value().z()<<std::endl;
//                    Iter_groundMap++;
//                }
//
//                std::cout<<"------------------break-----------------"<<std::endl;
//
//                for(int p = 0 ; p<10; p++)
//                {
//                    std::cout<<Iter_bulidingMap.value().x()<<"  "<<Iter_bulidingMap.value().y()<<"  "<<Iter_bulidingMap.value().z()<<std::endl;
//                    Iter_bulidingMap++;
//                }
//
//                Iter_groundMap = GroundNode_map.begin();
//                Iter_bulidingMap = BulidingNode_map.begin();

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
            return false;

        }

            // 鼠标移动
        case (osgGA::GUIEventAdapter::MOVE): {
            x = ea.getX();
            y = ea.getY();

            if (selectedPoints.empty()) {
                return false;
            }
        }

        case (osgGA::GUIEventAdapter::RELEASE): {
            if (x == ea.getX() && y == ea.getY()) {

                std::cout<<"the mouse: RELEASE"<<std::endl;

                pick(ea, view);
            }
            return true;
        }

        default:return false;
    }
}

void ClearPointsEvents::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {


    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON){

        std::cout<<"user click Right Mouse"<<std::endl;

        // 遍历 tempNode 结点 读取点数据

        // 定义两个 Map 数据结构
        QMap<int, osg::Vec3> tempNode_map;
        QMap<int, osg::Vec3> groundNode_map;

        VertexVisitor vtea_temp;
        VertexVisitor vtea_ground;

        if(tempNode->getNumChildren() == 1)
        {
            tempNode->setNodeMask(1);
        }

        tempNode->accept(vtea_temp);

        std::cout<<"otherNode->getNumChildren(): "<<otherNode->getNumChildren()<<std::endl;

        if(otherNode->getNumChildren() == 0)
        {
            groundNode->accept(vtea_ground);
        }
        else
        {
            otherNode->accept(vtea_ground);
        }

        size_tempNode = vtea_temp.extracted_verts->size();
        size_groundNode = vtea_ground.extracted_verts->size();

        iter_temp = vtea_temp.extracted_verts->begin();
        iter_ground = vtea_ground.extracted_verts->begin();

        // std::ofstream fout("/home/zhihui/Desktop/vertex.pcd");
        // std::ofstream fout_ground("/home/zhihui/Desktop/other_vertex.pcd");

        for (int i = 0; i < size_tempNode; ++i) {
            osg::Vec3 temp_point = osg::Vec3(iter_temp->x(),iter_temp->y(),iter_temp->z());

            tempNode_map.insert(i,temp_point);

            ++iter_temp;
        }

        std::cout<<"tempNode_map.size(): "<<tempNode_map.size()<<std::endl;

        for (int j = 0; j < size_groundNode; ++j) {
            osg::Vec3 temp_point = osg::Vec3(iter_ground->x(),iter_ground->y(),iter_ground->z());

            groundNode_map.insert(j,temp_point);

            ++iter_ground;
        }

        std::cout<<"groundNode_map.size(): "<<groundNode_map.size()<<std::endl;

        otherPointsGeode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> other_geom = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> other_coords = new osg::Vec3Array();
        otherPointsGeode->setName("OtherPoints");

        // 采用数组的遍历方法
//        for(int i = 0; i<size_tempNode; i++)
//        {
//            iter_ground = vtea_ground.extracted_verts->begin();
//
//            for(int j = 0; j<size_groundNode; j++)
//            {
//                //  精度小于 1.0e-4 说明相同  iter_temp: tempNode  iter_ground: groundNode
//                if(fabs(iter_temp->x()-iter_ground->x()) > 1.0e-3 && fabs(iter_temp->y()-iter_ground->y()) > 1.0e-4 && fabs(iter_temp->z()-iter_ground->z()) > 1.0e-4)
//                {
//                    other_coords->push_back(osg::Vec3(iter_ground->x(), iter_ground->y(), iter_ground->z()));
//                }
//                iter_ground++;
//            }
//
//            iter_temp++;
//        }

        // 采用map的遍历方法
        QMap<int, osg::Vec3>::iterator iter_tempMap = tempNode_map.begin();
        QMap<int, osg::Vec3>::iterator iter_groundMap;

//        for (int k = 0; k < size_tempNode; ++k) {
//
//            fout<<iter_tempMap.value().x()<<"  "<<iter_tempMap.value().y()<<"  "<<iter_tempMap.value().z()<<std::endl;
//
//            ++iter_tempMap;
//        }

        for ( ; iter_tempMap != tempNode_map.end() ; ) {

            iter_groundMap = groundNode_map.begin();

            for ( ; iter_groundMap != groundNode_map.end() ; ) {

                // 如果相等就移除
                if (iter_groundMap == groundNode_map.end()) {
                    std::cout << "-----------------------iter_groundMap: end-----------------------" << std::endl;
                }

                if(iter_tempMap.value().x() == iter_groundMap.value().x() &&
                   iter_tempMap.value().y() == iter_groundMap.value().y() &&
                   iter_tempMap.value().z() == iter_groundMap.value().z())
                {
                    int key = iter_groundMap.key();

                    // 从 Map 中删除这条记录
                    groundNode_map.remove(key);

                    break;
                }
                // 不然就指针后移
                ++iter_groundMap;

            }

            ++iter_tempMap;
        }

        for (iter_groundMap = groundNode_map.begin();  iter_groundMap != groundNode_map.end() ; ++iter_groundMap) {

            other_coords->push_back(osg::Vec3(iter_groundMap->x(), iter_groundMap->y(), iter_groundMap->z()));
        }

        std::cout<<"other_coords size: "<<other_coords->size()<<std::endl;

        // 始终让 otherNode 只有一个孩子被渲染出来
        if(otherNode->getNumChildren() != 0)
        {
            otherNode->removeChild(0,otherNode->getNumChildren());
        }

        other_geom->setVertexArray(other_coords.get());

        other_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(other_coords->size())));
        otherPointsGeode->addDrawable(other_geom.get());

        otherNode->addChild(otherPointsGeode);

//        // 移除tempNode 后 余下的结点
//        iter_groundMap = groundNode_map.begin();
//
//        for ( ;iter_groundMap != groundNode_map.end();) {
//
//            // fout_ground<<iter_groundMap.value().x()<<"  "<<iter_groundMap.value().y()<<"  "<<iter_groundMap.value().z()<<std::endl;
//
//            ++iter_groundMap;
//        }
//
//        // 打印 tempNode 的数据
//        iter_tempMap = tempNode_map.begin();
//
//        int size = 0;
//
//        for (;iter_tempMap != tempNode_map.end();)
//        {
//            // std::cout<<iter_tempMap.value().x()<<"  "<<iter_tempMap.value().y()<<"  "<<iter_tempMap.value().z()<<std::endl;
//
//            ++iter_tempMap;
//
//            ++size;
//        }
//
//        std::cout<<"size: "<<size<<std::endl;

        // 地面结点
        groundNode->setNodeMask(0);
        // 全部点云结点
        pointCloudNode->setNodeMask(0);
        // 保存清除点云的结点
        tempNode->setNodeMask(0);
        // 非地面结点
        buildingNode->setNodeMask(0);
        // 保存余下点云的结点
        otherNode->setNodeMask(1);

    }

    if (tempNode->getNumChildren() == 1) {

        // 防止多个刷子 清除前面一个刷子
        clean();
        if(!points.empty())
        {
            points.clear();
        }
    }


    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

        std::cout<<"user click the left mouse!"<<std::endl;

        // 取点方式和上面一致
        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);
        bool hasAlready = false;
        osg::Vec3d localPoint;
        if (picker->containsIntersections()) {

            auto intersections = picker->getIntersections();
            for (const auto &intersection : intersections) {
                auto childNode = intersection.nodePath.back();

                if (otherNode->getNumChildren() == 0 && childNode->getName() == "GroundPoints") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }
                    // 继续创建下一个虚拟平面
                    hasAlready = false;
                    break;
                }

                if(otherNode->getNumChildren() != 0 && childNode->getName() == "OtherPOints")
                {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }
                    // 继续创建下一个虚拟平面
                    hasAlready = false;
                    break;
                }
            }
        }

        if (!hasAlready) {
            // 生成虚拟平面用于选点
            std::cout<<"hasAlready: "<<hasAlready<<std::endl;
            // vpW = 10, vpH = 10;
            osg::ref_ptr<osgUtil::PolytopeIntersector> vpPicker =
                    new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                                     x - vpW,
                                                     y - vpH,
                                                     x + vpW,
                                                     y + vpH);
            osgUtil::IntersectionVisitor vpIv(vpPicker);
            view->getCamera()->accept(vpIv);

            osg::ref_ptr<osg::Switch> geom = new osg::Switch;
            // 平面四边形顶点
            osg::ref_ptr<osg::Vec3Array> quVertices = new osg::Vec3Array;
            osg::ref_ptr<osg::Geode> quGeode = new osg::Geode;
            osg::ref_ptr<osg::Geometry> quGeom = new osg::Geometry;
            osg::ref_ptr<osg::Vec4Array> quColors = new osg::Vec4Array;

            if (vpPicker->containsIntersections()) {
                double approximateZ;
                auto intersections = vpPicker->getIntersections();
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (otherNode->getNumChildren() == 0 && childNode->getName() == "GroundPoints") {
                        points.push_back(intersection.localIntersectionPoint);
                    }

                    if(otherNode->getNumChildren() != 0 && childNode->getName() == "OtherPoints")
                    {
                        points.push_back(intersection.localIntersectionPoint);
                    }
                }
                // 一个点不好画平面
                if (points.size() < 2) {
                    return;
                }
                std::cout << "points'size: " << points.size() << std::endl;
                approximateZ = calculateApproximateZ(points);
                std::cout << "approximateZ: " << approximateZ << std::endl;
                // 原始点数据的坐标
//                    for (const auto point : points) {
//                        std::cout << "x: " << point.x() << ", y: " << point.y() << ", z: " << point.z()
//                                  << std::endl;
//                    }


                // 叶子结点存储虚拟平面的点数据
                tempIrrelevantGeode = new osg::Geode;
                osg::ref_ptr<osg::Geometry> test_geom = new osg::Geometry;

                osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
                for (const auto point : points) {
                    coords->push_back(osg::Vec3(point.x(), point.y(), point.z()));
                }
                test_geom->setVertexArray(coords.get());

                test_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(coords->size())));
                tempIrrelevantGeode->addDrawable(test_geom.get());

                tempNode->addChild(tempIrrelevantGeode);

                std::cout<<"tempNode->getNumChildren(): "<<tempNode->getNumChildren()<<std::endl;


                // 原始点X-Y坐标值
                std::vector<std::pair<double, double>> vertices;
                for (const auto &point : points) {
                    vertices.emplace_back(std::make_pair(point.x(), point.y()));
                }
//                    for (const std::pair<double, double> pair : vertices) {
//                        std::cout << "(" << pair.first << ", " << pair.second << ")" << std::endl;
//                    }

                double minX, maxX;
                double minY, maxY;
                minX = maxX = vertices[0].first;
                minY = maxY = vertices[0].second;

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

                std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY
                          << std::endl;

                quVertices->push_back(osg::Vec3(minX, minY, approximateZ));
                quVertices->push_back(osg::Vec3(minX, maxY, approximateZ));
                quVertices->push_back(osg::Vec3(maxX, maxY, approximateZ));
                quVertices->push_back(osg::Vec3(maxX, minY, approximateZ));

                // red:: 1.0f, 0.0f, 0.0f, 1.0f  透明色： 1.0f, 1.0f, 1.0f, 0.1f
                quColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
                quColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
                quColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
                quColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));

                quGeom->setVertexArray(quVertices.get());
                quGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
                // quGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, 4));

                quGeom->setColorArray(quColors.get());
                // BIND_PER_VERTEX : 颜色的绑定方式为单个顶点  BIND_OVERALL : 颜色的绑定方式为所有顶点
                quGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

                quGeode->addDrawable(quGeom);

                // 设置透明度
                osg::ref_ptr<osg::StateSet> stateset = quGeode->getOrCreateStateSet();
                // stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
                stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
                // stateset->setMode(GL_DEPTH, osg::StateAttribute::OFF);
                stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                osg::BlendColor *bc = new osg::BlendColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
                osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
                blendFunc->setSource(osg::BlendFunc::CONSTANT_ALPHA);
                blendFunc->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
                stateset->setAttributeAndModes(bc, osg::StateAttribute::ON);
                stateset->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);
                bc->setConstantColor(osg::Vec4(1, 1, 1, 0.2));//第四个参数可用调节透明度

                geom->addChild(quGeode);
                virtualPlaneNode->addChild(geom);

                // 当前鼠标所选的点
                double w = 1.5, h = 1.5;
                osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                        osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
                osgUtil::IntersectionVisitor iv(picker);
                view->getCamera()->accept(iv);
                if (picker->containsIntersections()) {
                    auto intersections = picker->getIntersections();
                    std::cout << "intersections: " << intersections.size() << std::endl;
                    for (const auto &intersection: intersections) {
                        auto childNode = intersection.nodePath.back();
                        localPoint = intersection.localIntersectionPoint;
                        break;
                    }
                    localPoint.z() = approximateZ;
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }


                } else {
                    virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
                    return;
                }
            } else {
                return;
            }
        }
    }
}

double ClearPointsEvents::calculateApproximateZ(const std::vector<osg::Vec3d> &points) const {
    double minZ, maxZ;
    double delta;
    int maxN = 0;
    double res = 0;
    // 个数-Z坐标
    std::vector<std::pair<int, std::vector<double>>> numZ(10);

    minZ = maxZ = points[0].z();
    for (const auto &point : points) {
        if (point.z() > maxZ) {
            maxZ = point.z();
        }
        if (point.z() < minZ) {
            minZ = point.z();
        }
    }
    delta = (maxZ - minZ) / 10;

    for (const auto &point : points) {
        int i = static_cast<int>((point.z() - minZ) / delta);
        if (i < 0) {
            i = 0;
        }
        if (i >= 10) {
            i = 9;
        }
        numZ[i].first++;
        numZ[i].second.push_back(point.z());
    }

    for (const auto &num : numZ) {
        if (num.first > maxN) {
            maxN = num.first;
            double sum = 0;
            for (const auto &n : num.second) {
                sum += n;
            }
            res = sum / num.first;
        }
    }

    return res;
}


void ClearPointsEvents::clean()
{
    if (tempIrrelevantGeode != nullptr) {
        tempIrrelevantGeode->removeDrawables(0, tempIrrelevantGeode->getNumDrawables());
        tempIrrelevantGeode = nullptr;
    }

    points.clear();

    virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());

    if(tempNode->getNumChildren() != 0)
    {
        tempNode->removeChildren(0, tempNode->getNumChildren());
    }

    // tempNode->removeChildren(0, tempNode->getNumChildren());

    selectedPoints.clear();

    if( tempIrrelevantGeode == nullptr)
    {
        std::cout<<"(clean)tempIrrelevantGeode is null!"<<std::endl;

        std::cout << "------------Debug-----------------" << std::endl;

    }
}