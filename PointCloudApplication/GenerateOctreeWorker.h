//
// Created by zhihui on 5/10/19.
//

#ifndef HDMAPS_POINTCLOUDAPPLICATION_GENERATEOCTREEWORKER_H_
#define HDMAPS_POINTCLOUDAPPLICATION_GENERATEOCTREEWORKER_H_

#include <QObject>

class GenerateOctreeWorker : public QObject {
 Q_OBJECT

 public:
  explicit GenerateOctreeWorker(QObject *parent = nullptr);
  ~GenerateOctreeWorker() override = default;

 public Q_SLOTS:
  void generateOctreeData(const QString &filePath, bool hasRGB);

 Q_SIGNALS:
  void doneGenerateOctreeDataSignal();
};

#endif //HDMAPS_POINTCLOUDAPPLICATION_GENERATEOCTREEWORKER_H_
