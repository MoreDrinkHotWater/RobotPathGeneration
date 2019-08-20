//
// Created by zhihui on 5/6/19.
//

#include <QProgressDialog>
#include <QDebug>

#include "ProgressBarWorker.h"

ProgressBarWorker::ProgressBarWorker(QObject *parent)
        : QObject(parent), progressDialog(new QProgressDialog("", QString(), 0, 100, nullptr)) {
    progressDialog->setFixedSize(150, 4);
    progressDialog->reset();
}

ProgressBarWorker::~ProgressBarWorker() {
    delete progressDialog;
}

void ProgressBarWorker::showProgressBar(const QString &progressDialogLabelText) {
    //progressDialog->setLabelText(progressDialogLabelText);
    progressDialog->setWindowTitle(progressDialogLabelText);
    progressDialog->setWindowModality(Qt::NonModal);
    progressDialog->setMinimumDuration(0);

    qDebug() << "showProgressBar...";
    progressDialog->exec();
}

void ProgressBarWorker::closeProgressBar() {
    progressDialog->cancel();
    qDebug() << "closeProgressBar...";
}