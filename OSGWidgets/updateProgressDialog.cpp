//
// Created by zhihui on 9/18/19.
//

#include <iostream>
#include "updateProgressDialog.h"
#include <QCoreApplication>
#include <QThread>


UpdateProgressDialog::UpdateProgressDialog(QObject *parent): QObject(parent){}

void UpdateProgressDialog::updateProgressDialogSlot(int steps, QProgressDialog *progressDialog)
{

    std::cout<<"UpdateProgressDialog thread: "<<QThread::currentThreadId()<<std::endl;

    steps++;

    if (steps == 100)
        steps = 0;
    progressDialog->setValue(steps);
    QCoreApplication::processEvents();//避免界面冻结
    if (progressDialog->wasCanceled())
        progressDialog->setHidden(true);//隐藏对话框

}