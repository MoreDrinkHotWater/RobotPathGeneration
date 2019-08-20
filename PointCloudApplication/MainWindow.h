//
// Created by zhihui on 3/27/19.
//

#ifndef POINTCLOUDAPPLICATION_MAINWINDOW_H
#define POINTCLOUDAPPLICATION_MAINWINDOW_H

#include <QMainWindow>
#include <QFileInfo>
#include <QThread>

class OSGWidget;

class QDockWidget;

class QTreeWidget;

class QAction;

class QProgressDialog;

class GenerateOctreeWorker;

class MainWindow : public QMainWindow {
 Q_OBJECT
 public:
  explicit MainWindow(QWidget *parent = nullptr);

  ~MainWindow() override;

 private:

  void initActions();

  void initMenu();

  void initToolBar();

  void initStatusBar();

  void createDockWidget();

  QString calculateMD5(const QString &filePath) const;

  bool matchMD5(const QStringList &list, const QString &filePath) const;

 Q_SIGNALS:

  void test();

  void startGenerateOctreeDataSignal(const QString &filePath, bool hasRGB);

  void newProjectDefaultDirectorySignal(const QStringList &, const QString &, const QString &, const QString &, const QString &);

 public Q_SLOTS:

  void doneGernerateOctreeData();

  void newProjectDirectoryInfo(const QString &projectName,
                               const QString &dataFilesDir,
                               const QString &jsonDataDir,
                               const QString &csvDataDir,
                               const QString &mapDataDir);

 protected:
  void closeEvent(QCloseEvent *e) override;

 private Q_SLOTS:

  void newProject();

  void openProject();

  void openFileInfoSlot(QString &fileExtension, bool hasIntensity, bool hasRGB);

  void saveFile();

  void saveAsFile();

  void closeCurrentFile();

  // 暂时没有用到，保存的时候会写json文件
  void saveJsonFile();

  void quitApp();

  void drawLine(bool isActive);

  void modifyLine(bool isActive);

  void deleteLine(bool isActive);

  void drawTraceLine(bool isActive);

  void modifyTraceLine(bool isActive);

  void deleteTraceLine(bool isActive);

  void drawPavement(bool isActive);

  void deletePavement(bool isActive);

  void drawDrivingArrow(bool isActive);

  void modifyDrivingArrow(bool isActive);

  void deleteDrivingArrow(bool isActive);

  void drawTrafficLights(bool isActive);

  void deleteTrafficLights(bool isActive);

  void drawRoadLines(bool isActive);

  void modifyRoadLines(bool isActive);

  void deleteRoadLines(bool isActive);

  void measurePoints(bool isActive);

  void colorByZ(bool isActive);

  void colorByIntensity(bool isActive);

  void colorByTexture(bool isActive);

  void cutpoint(bool isActive);

  void help();

  void about();

 private:
  bool writeProjectXML(const QString &filePath);

 private:
  OSGWidget *osgWidget;

  QDockWidget *dockWidget;
  QTreeWidget *treeWidget;

  QMenuBar *pMenuBar;
  QToolBar *pToolBar;
  QStatusBar *pStatusBar;

  QAction *clickedEditorAction;
  QAction *clickedColorAction;

  QAction *newProjectAction, *openProjectAction, *saveFileAction, *saveAsFileAction, *closeCurrentFileAction,
          *quitAppActionAction;
  QAction *drawLineAction, *modifyLineAction, *deleteLineAction;
  QAction *drawTraceLineAction, *modifyTraceLineAction, *deleteTraceLineAction;
  QAction *drawPavementAction, *deletePavementAction;
  QAction *drawDrivingArrowAction, *modifyDrivingArrowAction, *deleteDrivingArrowAction;
  QAction *drawTrafficLightsAction, *deleteTrafficLightsAction;
  QAction *drawRoadLinesAction, *modifyRoadLinesAction, *deleteRoadLinesAction;
  QAction *measurePointsAction;
  QAction *colorByZAction, *colorByIntensityAction, *colorByTextureAction;
  QAction *helpAction, *aboutAction;
  // 剪切多余的点数据
  QAction *cutPointAction;

  QFileInfo openFileInfo;

  QString originalPCDFileName;

  // 当前工程的文件路径设置
  QString projectName;
  QString dataFilesDir;
  QString jsonDataDir;
  QString csvDataDir;
  QString mapDataDir;

  QString fileExtension;
  bool hasIntensity;
  bool hasRGB;

  bool hasBeenModified;

  GenerateOctreeWorker *generateOctreeWorker;
  QProgressDialog *progressDialog;

  QThread generateOctreeThread;

};

#endif //POINTCLOUDAPPLICATION_MAINWINDOW_H
