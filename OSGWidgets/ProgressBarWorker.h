//
// Created by zhihui on 5/6/19.
//

#ifndef HDMAPS_OSGWIDGETS_PROGRESSBARWORKER_H_
#define HDMAPS_OSGWIDGETS_PROGRESSBARWORKER_H_

#include <QObject>

class QProgressDialog;

class ProgressBarWorker : public QObject {
 Q_OBJECT

 public:
  explicit ProgressBarWorker(QObject *parent = nullptr);
  ~ProgressBarWorker() override;

 public Q_SLOTS:
  void showProgressBar(const QString &progressDialogLabelText);
  void closeProgressBar();

 private:
  QProgressDialog *progressDialog;
};

#endif //HDMAPS_OSGWIDGETS_PROGRESSBARWORKER_H_
