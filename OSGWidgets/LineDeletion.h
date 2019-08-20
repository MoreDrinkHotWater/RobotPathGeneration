//
// Created by zhihui on 4/9/19.
//

#ifndef HDMAPS_LINEDELETION_H
#define HDMAPS_LINEDELETION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

namespace osgFX {
    class Outline;
}

class LineDeletion : public osgGA::GUIEventHandler {
public:
    explicit LineDeletion(osg::Switch *rootNode);

    ~LineDeletion() override = default;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    // not necessary
    void cleanup();

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> vectorItemNode;
    osg::ref_ptr<osg::Geode> selectedLine;
    osg::ref_ptr<osgFX::Outline> outline;
    osg::ref_ptr<osg::Geode> childNode;
    float x, y;
};


#endif //HDMAPS_LINEDELETION_H
