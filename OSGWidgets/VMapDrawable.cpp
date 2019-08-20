//
// Created by zhihui on 4/4/19.
//

#include <osg/Node>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ValueObject>
#include <osgText/Text>
#include <osg/Geometry>
#include <osg/ShapeDrawable>

#include <string>

#include "VMapDrawable.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "../Common/VectorMapSingleton.h"

VMapDrawable::VMapDrawable(osg::Switch *rootNode) : rootNode(rootNode) {}

template<class T>
void VMapDrawable::drawVectorNode(const std::vector<mdc::Line> &lines, const T &object) {
    if (lines.empty()) {
        return;
    }
    osg::ref_ptr<osg::Switch> vectorItemNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                            vectorItemNodeName));
    std::vector<mdc::Point> points;
    for (const mdc::Line &line : lines) {
        mdc::Point sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(line.sPID));
        points.push_back(sPoint);
    }
    mdc::Point ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(lines.back().ePID));
    points.push_back(ePoint);

    int vectorIndex = vectorItemNode->getNumChildren();
    int startLineID = lines[0].lID;
    int endLineID = lines.back().lID;
    std::string type = "Uncertain";

    osg::ref_ptr<osg::Switch> vectorNode = new osg::Switch;
    vectorNode->setName(vectorItemName + std::to_string(vectorIndex++));
    vectorNode->setUserValue("startID", startLineID);
    vectorNode->setUserValue("endID", endLineID);
    vectorNode->setUserValue("itemType", type);

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    for (const auto &point : points) {
        vertices->push_back(osg::Vec3d(point.ly, point.bx, point.h));
    }
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
    colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->setColorArray(colors.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName("Line");
    geode->addDrawable(geom);

    vectorNode->addChild(geode);

    for (const auto &point : points) {
        int localPointIndex = point.pID;
        osg::Vec3d localPoint(point.ly, point.bx, point.h);

        osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
        pointGeode->setName("Sphere");
        pointGeode->setUserValue("ID", localPointIndex);
        pointGeode->setUserValue("pos", localPoint);

        osg::ref_ptr<osg::ShapeDrawable> pointSphere = new osg::ShapeDrawable(new osg::Sphere(localPoint, 0.15f));
        pointGeode->addDrawable(pointSphere);
        vectorNode->addChild(pointGeode);
    }

    osg::ref_ptr<osg::Switch> pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                           pointTextNodeName));
    for (const auto &point : points) {
        osg::Vec3d pos(point.ly, point.bx, point.h);
        std::string name = std::to_string(point.pID);
        osg::Vec4f color(0.0, 1.0, 0.0, 0.5);

        pointTextNode->addChild(drawTextGeode(pos, name, color).get());
    }

    osg::ref_ptr<osg::Switch> lineTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                          lineTextNodeName));
    for (const auto &line : lines) {
        auto sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(line.sPID));
        auto ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(line.ePID));

        osg::Vec3d pos1(sPoint.ly, sPoint.bx, sPoint.h);
        osg::Vec3d pos2(ePoint.ly, ePoint.bx, ePoint.h);
        osg::Vec3d pos = (pos1 + pos2) / 2;
        std::string name = std::to_string(line.lID);
        osg::Vec4 color(1.0, 0.0, 0.0, 0.5);

        lineTextNode->addChild(drawTextGeode(pos, name, color).get());
    }

    setNodeValue(object, vectorNode);
    vectorItemNode->addChild(vectorNode);
}

template<class T>
void VMapDrawable::setNodeValue(const T &object, osg::Node *node) {
    if (std::is_same<T, mdc::Line>::value) {
        setLineNodeValue(reinterpret_cast<const mdc::Line &>(object), node);
    } else {
        setNullNodeValue(node);
    }
}

osg::ref_ptr<osg::Geode> VMapDrawable::drawTextGeode(const osg::Vec3d &pos, const std::string &content,
                                                     const osg::Vec4f &color, float size) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName(content);

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setCharacterSize(size);
    text->setAxisAlignment(osgText::TextBase::XY_PLANE);
    text->setPosition(pos);
    text->setText(content);
    text->setColor(color);

    geode->addDrawable(text);
    return geode;
}

void VMapDrawable::setNullNodeValue(osg::Node *node) {
    std::string itemType = "Uncertain";
    node->setUserValue("itemType", itemType);
}

void VMapDrawable::setLineNodeValue(const mdc::Line &object, osg::Node *node) {
    std::string itemType = "Line";
    node->setUserValue("itemType", itemType);
}
