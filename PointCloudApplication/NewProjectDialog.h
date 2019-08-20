//
// Created by zhihui on 7/4/19.
//

#ifndef HDMAPSFORROBOT_POINTCLOUDAPPLICATION_NEWPROJECTDIALOG_H_
#define HDMAPSFORROBOT_POINTCLOUDAPPLICATION_NEWPROJECTDIALOG_H_

#include <QDialog>

class QPushButton;

class QLineEdit;

class NewProjectDialog : public QDialog {
 Q_OBJECT
 public:
  explicit NewProjectDialog(QWidget *parent = nullptr);
  ~NewProjectDialog() override = default;

  Q_DISABLE_COPY(NewProjectDialog);

 Q_SIGNALS:
  void newProjectInfoSignal(const QString &, const QString &, const QString &, const QString &, const QString &);

 public Q_SLOTS:

  void setAllDefaultDirectory(const QStringList &projectsNameList,
                              const QString &dataFilesDir,
                              const QString &jsonDataDir,
                              const QString &csvDataDir,
                              const QString &mapDataDir);
 private Q_SLOTS:

  void dataFilesDirectory();
  void jsonDataDirectory();
  void csvDataDirectory();
  void mapDataDirectory();

  void cancel();
  void confirm();

 private:
  bool isProjectNameValid();

 private:
  QPushButton *pbDataFiles;
  QPushButton *pbJSONData;
  QPushButton *pbCSVData;
  QPushButton *pbMapData;

  QPushButton *pbCancel;
  QPushButton *pbConfirm;

  QLineEdit *leProjectName;
  QLineEdit *leDataFiles;
  QLineEdit *leJSONData;
  QLineEdit *leCSVData;
  QLineEdit *leMapData;

 private:
  QStringList projectsNameList;
  QString projectName;
  QString dataFilesDir;
  QString jsonDataDir;
  QString csvDataDir;
  QString mapDataDir;
};

#endif //HDMAPSFORROBOT_POINTCLOUDAPPLICATION_NEWPROJECTDIALOG_H_
