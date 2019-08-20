//
// Created by zhihui on 4/22/19.
//

#include <iostream>
#include <iomanip>

#include <osg/Geode>
#include <osg/Switch>
#include <osgViewer/View>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgText/Text>

#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VectorMapSingleton.h"
#include "TrafficLightsEditor.h"
#include "Color.h"

TrafficLightsEditor::TrafficLightsEditor(osg::Switch *rootNode)
        : rootNode(rootNode),
          tempNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, tempNodeName))),
          tempLineGeode(nullptr),
          selectedPoints(),
          trafficLightsItemNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                             trafficLightsItemNodeName))),
          curPointIndex(0),
          x(0),
          y(0) {}

TrafficLightsEditor::~TrafficLightsEditor() {
    cleanup();
}

bool TrafficLightsEditor::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case (osgGA::GUIEventAdapter::KEYDOWN): {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape) {
                rollback();
                return true;
            }
            return false;
        }

        case (osgGA::GUIEventAdapter::MOVE): {
            x = ea.getX();
            y = ea.getY();

            if (selectedPoints.empty()) {
                return false;
            }

            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);

            if (picker->containsIntersections()) {
                osg::Vec3d curPoint;

                bool isIntersect = false;
                auto intersections = picker->getIntersections();
                for (const auto &intersection: intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        curPoint = intersection.localIntersectionPoint;
                        isIntersect = true;
                        break;
                    }
                    if (childNode->getName() == "Sphere") {
                        childNode->getUserValue("pos", curPoint);
                        isIntersect = true;
                        break;
                    }
                }

                if (!isIntersect) {
                    return false;
                }

                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(std::get<1>(selectedPoints.back()));
                vertices->push_back(curPoint);

                if (tempLineGeode == nullptr) {
                    tempLineGeode = new osg::Geode;
                    tempLineGeode->setName("tempLineGeode");
                    tempNode->addChild(tempLineGeode);
                } else {
                    tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
                }

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setName("tempLineGeom");
                tempLineGeode->addDrawable(geom);

                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertices->size()));
                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->setVertexArray(vertices.get());
            }

            return false;
        }

        case (osgGA::GUIEventAdapter::RELEASE): {
            if (x == ea.getX() && y == ea.getY()) {
                pick(ea, view);
            }
            return true;
        }

        default:return false;
    }
}

void TrafficLightsEditor::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    if (selectedPoints.empty()) {
        updateIndex();
        cleanup();
    }

    osg::ref_ptr<osg::Geode> childNode;

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        if (picker->containsIntersections()) {
            osg::Vec3d localPoint;
            int localPointIndex = 0;

            auto intersections = picker->getIntersections();
            for (const auto &intersection : intersections) {
                auto childNode = intersection.nodePath.back();
                if (childNode->getName() == "CloudPoints") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }
                    localPointIndex = curPointIndex++;

                    osg::ref_ptr<osg::Geode> nodeGeode = new osg::Geode;
                    nodeGeode->setName("Sphere");
                    nodeGeode->setUserValue("pos", localPoint);
                    nodeGeode->setUserValue("ID", localPointIndex);

                    osg::ref_ptr<osg::ShapeDrawable> nodeSphere = new osg::ShapeDrawable(
                            new osg::Sphere(localPoint, 0.1f));
                    nodeSphere->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
                    nodeGeode->addDrawable(nodeSphere.get());
                    tempNode->addChild(nodeGeode.get());

                    break;
                }
                if (childNode->getName() == "Sphere") {
                    childNode->getUserValue("pos", localPoint);
                    childNode->getUserValue("ID", localPointIndex);
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }
                    // 方便回退
                    osg::ref_ptr<osg::Geode> nodeGeode = new osg::Geode;
                    nodeGeode->setName("Sphere");
                    nodeGeode->setUserValue("pos", localPoint);
                    nodeGeode->setUserValue("ID", localPointIndex);
                    osg::ref_ptr<osg::ShapeDrawable> nodeSphere = new osg::ShapeDrawable(
                            new osg::Sphere(localPoint, 0.1f));
                    nodeSphere->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
                    nodeGeode->addDrawable(nodeSphere.get());
                    tempNode->addChild(nodeGeode.get());
                    break;
                }
            }
            // 所选点可能在临时的线上，localPoint是默认的0值
            if (localPointIndex == 0) {
                return;
            }
            std::cout << "------------Debug-----------------" << std::endl;
            std::cout << "localPointIndex: " << localPointIndex << " --- localPoint: " << localPoint << std::endl;
            selectedPoints.emplace_back(std::make_pair(localPointIndex, localPoint));
            if (selectedPoints.size() >= 2) {
                osg::Vec3d penultimatePoint = std::get<1>(*(selectedPoints.end() - 2));
                osg::Vec3d lastPoint = std::get<1>(selectedPoints.back());

                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(penultimatePoint);
                vertices->push_back(lastPoint);

                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setName("solidLineGeom");
                geom->setVertexArray(vertices.get());
                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->setName("solidLineGeode");
                geode->addDrawable(geom);

                tempNode->addChild(geode);
            }
        }
    }

    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && selectedPoints.size() == 2) {

        std::vector<mdc::Point> points;

        int curTrafficLightsIndex = VectorMapSingleton::getInstance()->getMaxTrafficLightsIndex() + 1;

        for (int i = 0; i < 2; ++i) {
            size_t index = selectedPoints[i].first;
            osg::Vec3d pos = selectedPoints[i].second;
            mdc::Point
                    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(selectedPoints[i].first));
            point.pID = index;
            point.bx = pos.y();
            point.ly = pos.x();
            point.h = pos.z();
            points.push_back(point);
        }

        mdc::TrafficLights trafficLights;
        trafficLights.tlID = curTrafficLightsIndex;
        trafficLights.lightLocationPoint = points[0].pID;
        osg::Vec2 pointXAngle = osg::Vec2((points[1].ly - points[0].ly), (points[1].bx - points[0].bx));
        trafficLights.rotation = calculatePointXAngle(pointXAngle);

        int trafficLightsIndex = trafficLightsItemNode->getNumChildren();
        std::string type("TrafficLights");

        osg::ref_ptr<osg::Switch> trafficLightsNode = new osg::Switch;
        trafficLightsNode->setName(trafficLightsItemName + std::to_string(trafficLightsIndex++));
        trafficLightsNode->setUserValue("itemType", type);
        trafficLightsItemNode->addChild(trafficLightsNode);

        osg::ref_ptr<osg::Geode> nodeGeode = new osg::Geode;
        nodeGeode->setName("TrafficLights_" + std::to_string(curTrafficLightsIndex));
        nodeGeode->setUserValue("pos", selectedPoints[0].second);
        nodeGeode->setUserValue("ID", curTrafficLightsIndex);

        osg::ref_ptr<osg::ShapeDrawable> nodeSphere = new osg::ShapeDrawable(
                new osg::Sphere(selectedPoints[0].second, 0.2f));
        nodeSphere->setColor(trafficLightsColor);
        nodeGeode->addDrawable(nodeSphere.get());
        trafficLightsNode->addChild(nodeGeode.get());

        // 显示交通信号灯ID
        osg::ref_ptr<osg::Switch>
                trafficLightsTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
                rootNode, trafficLightsTextNodeName));

        std::string name = "TTrafficLights_" + std::to_string(trafficLights.tlID);

        osg::ref_ptr<osg::Geode> trafficLightsTextGeode = new osg::Geode;
        trafficLightsTextGeode->setName(name);

        osg::ref_ptr<osgText::Text> text = new osgText::Text;
        text->setCharacterSize(0.5);
        text->setAxisAlignment(osgText::TextBase::XY_PLANE);
        text->setPosition(selectedPoints[0].second);
        text->setText(std::to_string(trafficLights.tlID));
        text->setColor(trafficLightsColor);

        trafficLightsTextGeode->addDrawable(text);

        trafficLightsTextNode->addChild(trafficLightsTextGeode);

        VectorMapSingleton::getInstance()->update(trafficLights);
        VectorMapSingleton::getInstance()->update(points[0]);

        cleanup();
    }
}

void TrafficLightsEditor::updateIndex() {
    curPointIndex = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;
}

void TrafficLightsEditor::rollback() {
    if (!selectedPoints.empty()) {
        selectedPoints.pop_back();
    }
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
    }
    // 删除已画的点和线
    tempNode->removeChildren(tempNode->getNumChildren() - 2, 2);
}

void TrafficLightsEditor::cleanup() {
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
        tempLineGeode = nullptr;
    }
    tempNode->removeChildren(0, tempNode->getNumChildren());

    selectedPoints.clear();
}

double TrafficLightsEditor::calculatePointXAngle(const osg::Vec2 &p1, const osg::Vec2 &p2) const {
    osg::Vec2 np1 = p1, np2 = p2;
    np1.normalize();
    np2.normalize();

    double temp = (np1 * np2);
    // 1和-1时为0和180°
    if (fabs(fabs(temp) - 1) < 1E-7) {
        return temp > 0 ? 0 : osg::PI;
    }
    return acos(temp);
}