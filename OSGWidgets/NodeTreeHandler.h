//
// Created by zhihui on 3/28/19.
//

#ifndef POINTCLOUDAPPLICATION_NODETREEHANDLER_H
#define POINTCLOUDAPPLICATION_NODETREEHANDLER_H

#include <osgGA/GUIEventHandler>

class NodeTreeHandler : public osgGA::GUIEventHandler {
public:
    explicit NodeTreeHandler(osg::Switch *node);

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

private:
    osg::ref_ptr<osg::Switch> rootNode;
};


#endif //POINTCLOUDAPPLICATION_NODETREEHANDLER_H
