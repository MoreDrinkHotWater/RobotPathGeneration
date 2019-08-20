//
// Created by zhihui on 8/13/19.
//

#ifndef DEMO_TWO_VERTEXVISITOR_H
#define DEMO_TWO_VERTEXVISITOR_H

#include <iostream>
#include <osg/NodeVisitor>
#include <osg/Geometry>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>


class VertexVisitor : public osg::NodeVisitor
{
public:
    osg::ref_ptr<osg::Vec3Array> extracted_verts;

    VertexVisitor():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        extracted_verts = new osg::Vec3Array();
    }

    void apply(osg::Geode &geode) {
        for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
        {
            auto *geom = dynamic_cast<osg::Geometry *>(geode.getDrawable(i));
            if(!geom)
            {
                std::cout<<"几何体错误"<<std::endl;
                continue;
            }

            auto *verts = dynamic_cast<osg::Vec3Array *>(geom->getVertexArray());
            if(!verts)
            {
                std::cout<<"顶点数组错误"<<std::endl;

                continue;
            }

            extracted_verts->insert(extracted_verts->end(), verts->begin(), verts->end());
        }


    }
};

#endif //DEMO_TWO_VERTEXVISITOR_H
