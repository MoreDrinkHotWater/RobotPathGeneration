//
// Created by zhihui on 8/13/19.
//
#include "JudgeGroundPoint.h"
#include "VertexVisitor.h"

JudgeGroundPoint::JudgeGroundPoint(std::vector<osg::Vec3>::iterator iter, std::vector<osg::Vec3>::iterator iterator, int size_tempNode, int size_groundNode, VertexVisitor vtea)
{
    std::cout<<"TestThread:run:"<<QThread::currentThreadId()<<std::endl;

    osg::ref_ptr<osg::Vec3Array> other_coords;

    for(int i = 0; i<size_groundNode; i++)
    {
        iter = vtea.extracted_verts->begin();

        for(int j = 0; j<size_tempNode; j++)
        {
            //  精度小于 1.0e-4 说明相同  iter: tempNode  iterator: groundNode
            if(fabs(iter->x()-iterator->x()) > 1.0e-3 && fabs(iter->y()-iterator->y()) > 1.0e-4 && fabs(iter->z()-iterator->z()) > 1.0e-4)
            {
                other_coords->push_back(osg::Vec3(iterator->x(), iterator->y(), iterator->z()));
            }
            iter++;
        }
        iterator++;
    }

    return_Vec3Array = other_coords;
}



