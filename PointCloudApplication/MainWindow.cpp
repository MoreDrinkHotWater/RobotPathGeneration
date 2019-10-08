//
// Created by zhihui on 3/27/19.
//

#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QMessageBox>
#include <QIcon>
#include <QApplication>
#include <QTreeWidget>
#include <QDockWidget>
#include <QFileDialog>
#include <QFile>
#include <QAction>
#include <QDir>
#include <QCloseEvent>
#include <QProgressDialog>
#include <QSettings>
#include <QCryptographicHash>
#include <QDomDocument>
#include <QDebug>

#include <QListWidget>
#include <QDockWidget>

#include "MainWindow.h"
#include "../OSGWidgets/OSGWidget.h"
#include "OpenFileDialog.h"
#include "NewProjectDialog.h"
#include "GenerateOctreeWorker.h"
#include "Toolbar.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          osgWidget(new OSGWidget(this)),
                                          dockWidget(new QDockWidget(this)),
                                          treeWidget(new QTreeWidget(this)),
                                          pMenuBar(menuBar()),
                                          pToolBar(addToolBar("Tools")),
                                          pStatusBar(statusBar()),
                                          clickedEditorAction(nullptr),
                                          clickedColorAction(nullptr),
                                          newProjectAction(new QAction(this)),
                                          openProjectAction(new QAction(this)),
                                          saveFileAction(new QAction(this)),
                                          saveAsFileAction(new QAction(this)),
                                          closeCurrentFileAction(new QAction(this)),
                                          quitAppActionAction(new QAction(this)),
                                          drawLineAction(new QAction(this)),
                                          modifyLineAction(new QAction(this)),
                                          deleteLineAction(new QAction(this)),
                                          drawTraceLineAction(new QAction(this)),
                                          modifyTraceLineAction(new QAction(this)),
                                          deleteTraceLineAction(new QAction(this)),
                                          drawPavementAction(new QAction(this)),
                                          deletePavementAction(new QAction(this)),
                                          drawDrivingArrowAction(new QAction(this)),
                                          modifyDrivingArrowAction(new QAction(this)),
                                          deleteDrivingArrowAction(new QAction(this)),
                                          drawTrafficLightsAction(new QAction(this)),
                                          deleteTrafficLightsAction(new QAction(this)),
                                          drawRoadLinesAction(new QAction(this)),
                                          modifyRoadLinesAction(new QAction(this)),
                                          deleteRoadLinesAction(new QAction(this)),
                                          measurePointsAction(new QAction(this)),
                                          colorByZAction(new QAction(this)),
                                          colorByIntensityAction(new QAction(this)),
                                          colorByTextureAction(new QAction(this)),
                                          helpAction(new QAction(this)),
                                          aboutAction(new QAction(this)),
                                          cutPointAction(new QAction(this)),
                                          polygonCutPointAction(new QAction(this)),
                                          openFileInfo(),
                                          originalPCDFileName(QString()),
                                          projectName(QString()),
                                          dataFilesDir(QString()),
                                          jsonDataDir(QString()),
                                          csvDataDir(QString()),
                                          mapDataDir(QString()),
                                          fileExtension(),
                                          hasIntensity(false),
                                          hasRGB(false),
                                          hasBeenModified(true),
                                          generateOctreeWorker(new GenerateOctreeWorker()),
                                          progressDialog(new QProgressDialog("Reading files...",
                                                                             QString(),
                                                                             0,
                                                                             0,
                                                                             this)),
                                          showConsoleAction(new QAction(this)),
                                          dock(),
                                          consoleList(),
                                          viewMenu(){
    setWindowTitle("HDMapsForRobot");

    progressDialog->reset();
    connect(this,
            &MainWindow::startGenerateOctreeDataSignal,
            generateOctreeWorker,
            &GenerateOctreeWorker::generateOctreeData);
    connect(generateOctreeWorker,
            &GenerateOctreeWorker::doneGenerateOctreeDataSignal,
            this,
            &MainWindow::doneGernerateOctreeData);
    generateOctreeWorker->moveToThread(&generateOctreeThread);
    // connect(&generateOctreeThread, &QThread::finished, generateOctreeWorker, &QObject::deleteLater);

    initActions();
    initMenu();
    initToolBar();
    initStatusBar();

    // 确保配置文件存在
    QFileInfo fileInfo(QDir::currentPath() + "/.project.ini");
    if (!fileInfo.exists()) {
        QFile file(fileInfo.absoluteFilePath());
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(nullptr, "Project Configure", "No .ini File!", QMessageBox::Yes);
        }
        file.close();
    }

    showconsole();

    setCentralWidget(osgWidget);
    osgWidget->init();
}

MainWindow::~MainWindow() {
    generateOctreeThread.quit();
    generateOctreeThread.wait();
    delete generateOctreeWorker;
}

void MainWindow::initActions() {

    newProjectAction->setIcon(QIcon::fromTheme("document-new", QIcon(":/images/file_new.png")));
    newProjectAction->setText("&New");
    newProjectAction->setShortcut(QKeySequence::New);
    newProjectAction->setStatusTip("New Project");
    connect(newProjectAction, &QAction::triggered, this, &MainWindow::newProject);

    openProjectAction->setIcon(QIcon::fromTheme("document-open", QIcon(":/images/file_open.png")));
    openProjectAction->setText("&Open...");
    openProjectAction->setShortcut(QKeySequence::Open);
    openProjectAction->setStatusTip("Open Project");
    connect(openProjectAction, &QAction::triggered, this, &MainWindow::openProject);

    saveFileAction->setIcon(QIcon::fromTheme("document-save", QIcon(":/images/file_save.png")));
    saveFileAction->setText("&Save");
    saveFileAction->setShortcut(QKeySequence::Save);
    saveFileAction->setStatusTip("Save File");
    connect(saveFileAction, &QAction::triggered, this, &MainWindow::saveFile);

    saveAsFileAction->setIcon(QIcon::fromTheme("document-save-as", QIcon(":/images/file_save_as.png")));
    saveAsFileAction->setText("&SaveAS");
    saveAsFileAction->setShortcut(QKeySequence::SaveAs);
    saveAsFileAction->setStatusTip("SaveAS File");
    connect(saveAsFileAction, &QAction::triggered, this, &MainWindow::saveAsFile);

    closeCurrentFileAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/file_close.png")));
    closeCurrentFileAction->setText("&CloseCurrentFile");
    closeCurrentFileAction->setShortcut(QKeySequence::Close);
    closeCurrentFileAction->setStatusTip("Close Current File");
    connect(closeCurrentFileAction, &QAction::triggered, this, &MainWindow::closeCurrentFile);

    quitAppActionAction->setIcon(QIcon::fromTheme("application-exit", QIcon(":/images/appExit.png")));
    quitAppActionAction->setText("&Quit");
    quitAppActionAction->setShortcut(QKeySequence::Quit);
    quitAppActionAction->setStatusTip("Quit");
    connect(quitAppActionAction, &QAction::triggered, this, &MainWindow::quitApp, Qt::QueuedConnection);

    drawLineAction->setIcon(QIcon(":/images/line.png"));
    drawLineAction->setText("Draw Line");
    drawLineAction->setStatusTip("Draw line");
    drawLineAction->setCheckable(true);
    connect(drawLineAction, &QAction::triggered, this, &MainWindow::drawLine);

    modifyLineAction->setIcon(QIcon(":/images/modify.jpeg"));
    modifyLineAction->setText("Modify line");
    modifyLineAction->setStatusTip("Modify line");
    modifyLineAction->setCheckable(true);
    connect(modifyLineAction, &QAction::triggered, this, &MainWindow::modifyLine);

    deleteLineAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/delete.png")));
    deleteLineAction->setText("Delete line");
    deleteLineAction->setStatusTip("Delete line");
    deleteLineAction->setCheckable(true);
    connect(deleteLineAction, &QAction::triggered, this, &MainWindow::deleteLine);

    drawTraceLineAction->setIcon(QIcon(":/images/traceLine.png"));
    drawTraceLineAction->setText("Draw Trace line");
    drawTraceLineAction->setStatusTip("Draw Trace line");
    drawTraceLineAction->setCheckable(true);
    connect(drawTraceLineAction, &QAction::triggered, this, &MainWindow::drawTraceLine);

    modifyTraceLineAction->setIcon(QIcon(":/images/modifyTraceLine.jpeg"));
    modifyTraceLineAction->setText("Modify Trace line");
    modifyTraceLineAction->setStatusTip("Modify Trace line");
    modifyTraceLineAction->setCheckable(true);
    connect(modifyTraceLineAction, &QAction::triggered, this, &MainWindow::modifyTraceLine);

    deleteTraceLineAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/deleteTraceLine.png")));
    deleteTraceLineAction->setText("Delete Trace line");
    deleteTraceLineAction->setStatusTip("Delete Trace line");
    deleteTraceLineAction->setCheckable(true);
    connect(deleteTraceLineAction, &QAction::triggered, this, &MainWindow::deleteTraceLine);

    drawPavementAction->setIcon(QIcon(":/images/pavement.png"));
    drawPavementAction->setText("Draw pavement");
    drawPavementAction->setStatusTip("Draw pavement");
    drawPavementAction->setCheckable(true);
    connect(drawPavementAction, &QAction::triggered, this, &MainWindow::drawPavement);

    deletePavementAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/deletePavement.png")));
    deletePavementAction->setText("Delete pavement");
    deletePavementAction->setStatusTip("Delete pavement");
    deletePavementAction->setCheckable(true);
    connect(deletePavementAction, &QAction::triggered, this, &MainWindow::deletePavement);

    drawDrivingArrowAction->setIcon(QIcon(":/images/drivingArrow.jpeg"));
    drawDrivingArrowAction->setText("Draw drivingArrow");
    drawDrivingArrowAction->setStatusTip("Draw drivingArrow");
    drawDrivingArrowAction->setCheckable(true);
    connect(drawDrivingArrowAction, &QAction::triggered, this, &MainWindow::drawDrivingArrow);

    modifyDrivingArrowAction->setIcon(QIcon(":/images/modifyDrivingArrow.jpeg"));
    modifyDrivingArrowAction->setText("Modify drivingArrow");
    modifyDrivingArrowAction->setStatusTip("Modify drivingArrow");
    modifyDrivingArrowAction->setCheckable(true);
    connect(modifyDrivingArrowAction, &QAction::triggered, this, &MainWindow::modifyDrivingArrow);

    deleteDrivingArrowAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/deleteDrivingArrow.png")));
    deleteDrivingArrowAction->setText("Delete drivingArrow");
    deleteDrivingArrowAction->setStatusTip("Delete drivingArrow");
    deleteDrivingArrowAction->setCheckable(true);
    connect(deleteDrivingArrowAction, &QAction::triggered, this, &MainWindow::deleteDrivingArrow);

    drawTrafficLightsAction->setIcon(QIcon(":/images/drawTrafficLights.jpeg"));
    drawTrafficLightsAction->setText("Draw TrafficLights");
    drawTrafficLightsAction->setStatusTip("Draw TrafficLights");
    drawTrafficLightsAction->setCheckable(true);
    connect(drawTrafficLightsAction, &QAction::triggered, this, &MainWindow::drawTrafficLights);

    deleteTrafficLightsAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/deleteTrafficLights.png")));
    deleteTrafficLightsAction->setText("Delete TrafficLights");
    deleteTrafficLightsAction->setStatusTip("Delete TrafficLights");
    deleteTrafficLightsAction->setCheckable(true);
    connect(deleteTrafficLightsAction, &QAction::triggered, this, &MainWindow::deleteTrafficLights);

    drawRoadLinesAction->setIcon(QIcon(":/images/drawRoadLines.jpeg"));
    drawRoadLinesAction->setText("Draw RoadLines");
    drawRoadLinesAction->setStatusTip("Draw RoadLines");
    drawRoadLinesAction->setCheckable(true);
    connect(drawRoadLinesAction, &QAction::triggered, this, &MainWindow::drawRoadLines);

    modifyRoadLinesAction->setIcon(QIcon(":/images/modifyRoadLines.png"));
    modifyRoadLinesAction->setText("Modify RoadLines");
    modifyRoadLinesAction->setStatusTip("Modify RoadLines");
    modifyRoadLinesAction->setCheckable(true);
    connect(modifyRoadLinesAction, &QAction::triggered, this, &MainWindow::modifyRoadLines);

    deleteRoadLinesAction->setIcon(QIcon::fromTheme("edit-delete", QIcon(":/images/deleteRoadLines.png")));
    deleteRoadLinesAction->setText("Delete RoadLines");
    deleteRoadLinesAction->setStatusTip("Delete RoadLines");
    deleteRoadLinesAction->setCheckable(true);
    connect(deleteRoadLinesAction, &QAction::triggered, this, &MainWindow::deleteRoadLines);

    measurePointsAction->setIcon(QIcon(":/images/measurePoints.png"));
    measurePointsAction->setText("Measure points");
    measurePointsAction->setStatusTip("Measure points");
    measurePointsAction->setCheckable(true);
    connect(measurePointsAction, &QAction::triggered, this, &MainWindow::measurePoints);

    colorByZAction->setIcon(QIcon(":/images/colorByZ.jpeg"));
    colorByZAction->setText("Color By Z");
    colorByZAction->setStatusTip("Color By Z");
    colorByZAction->setCheckable(true);
    connect(colorByZAction, &QAction::triggered, this, &MainWindow::colorByZ);

    colorByIntensityAction->setIcon(QIcon(":/images/colorByIntensity.jpeg"));
    colorByIntensityAction->setText("Color By Intensity");
    colorByIntensityAction->setStatusTip("Color By Intensity");
    colorByIntensityAction->setCheckable(true);
    connect(colorByIntensityAction, &QAction::triggered, this, &MainWindow::colorByIntensity);

    colorByTextureAction->setIcon(QIcon(":/images/colorByTexture.jpeg"));
    colorByTextureAction->setText("Color By Texture");
    colorByTextureAction->setStatusTip("Color By Texture");
    colorByTextureAction->setCheckable(true);
    connect(colorByTextureAction, &QAction::triggered, this, &MainWindow::colorByTexture);

    helpAction->setIcon(QIcon::fromTheme("help-contents", QIcon(":/images/help.png")));
    helpAction->setText("&Help");
    helpAction->setShortcut(QKeySequence::HelpContents);
    helpAction->setStatusTip("Help");
    connect(helpAction, &QAction::triggered, this, &MainWindow::help);

    aboutAction->setIcon(QIcon::fromTheme("help-about", QIcon(":/images/about.png")));
    aboutAction->setText("&About");
    aboutAction->setShortcut(QKeySequence::WhatsThis);
    aboutAction->setStatusTip("About");
    connect(aboutAction, &QAction::triggered, this, &MainWindow::about);

    cutPointAction->setIcon(QIcon(":/images/Cut.png"));
    cutPointAction->setText("cut point");
    cutPointAction->setStatusTip("cut point");
    cutPointAction->setCheckable(true);
    connect(cutPointAction, &QAction::triggered, this, &MainWindow::cutpoint);

    polygonCutPointAction->setIcon(QIcon(":/images/modifyRoadLines.png"));
    polygonCutPointAction->setText("polygon cut point");
    polygonCutPointAction->setStatusTip("polygon cut point");
    polygonCutPointAction->setCheckable(true);
    connect(polygonCutPointAction, &QAction::triggered, this, &MainWindow::polygonCutPoint);
}

void MainWindow::initMenu() {

    QMenu *menu;

    menu = new QMenu("&File", this);
    menu->addAction(newProjectAction);
    menu->addAction(openProjectAction);
    menu->addSeparator();

    // new 一个 类ToolBar的对象指针
    ToolBar *tb = new ToolBar("Open Rencent File", this);

    // 把对象指针 加入到 QList<ToolBar*> 中
    toolBars.append(tb);

    connect(osgWidget, &OSGWidget::doneOpenRecentSignal, tb, &ToolBar::endprogress);

    connect(tb, &ToolBar::SendSignal, this, &MainWindow::UpdateConsole);

    connect(tb, &ToolBar::SendSignal, osgWidget, &OSGWidget::openRecentSlot);

    // 将工具栏添加到主窗口
    addToolBar(tb);

    // toolbarMenu方法会返回一个 menu[Test, Manage Project...] 这里的 toolBars.at(0) 即 tb
    menu->addMenu(toolBars.at(0)->toolbarMenu());

    menu->addAction(closeCurrentFileAction);
    menu->addSeparator();

    menu->addAction(saveFileAction);
    menu->addAction(saveAsFileAction);
    menu->addSeparator();

    menu->addAction(quitAppActionAction);

    pMenuBar->addMenu(menu);

    menu = new QMenu("&Draw", this);
//    menu->addActions({drawLineAction, modifyLineAction, deleteLineAction});
//    menu->addSeparator();

    menu->addActions({drawTraceLineAction, modifyTraceLineAction, deleteTraceLineAction});
    menu->addSeparator();

//    menu->addActions({drawPavementAction, deletePavementAction});
//    menu->addSeparator();
//
//    menu->addActions({drawDrivingArrowAction, modifyDrivingArrowAction, deleteDrivingArrowAction});
//    menu->addSeparator();
//
//    menu->addActions({drawTrafficLightsAction, deleteTrafficLightsAction});
//    menu->addSeparator();
//
//    menu->addActions({drawRoadLinesAction, modifyRoadLinesAction, deleteRoadLinesAction});
//    menu->addSeparator();
//
    pMenuBar->addMenu(menu);

    menu = new QMenu("&Measurement", this);
    menu->addActions({measurePointsAction});
    menu->addSeparator();

    pMenuBar->addMenu(menu);

    menu = new QMenu("&Render", this);
    menu->addActions({colorByZAction, colorByIntensityAction, colorByTextureAction});

    pMenuBar->addMenu(menu);

    menu = new QMenu("&Help", this);
    menu->addActions({helpAction, aboutAction});

    pMenuBar->addMenu(menu);

    viewMenu = new QMenu("&View", this);

    pMenuBar->addMenu(viewMenu);

}

void MainWindow::initToolBar() {
    pToolBar->addActions({newProjectAction, openProjectAction, saveFileAction, saveAsFileAction,
                          closeCurrentFileAction});
    pToolBar->addSeparator();
    pToolBar->addActions({drawTraceLineAction, modifyTraceLineAction, deleteTraceLineAction});
    pToolBar->addSeparator();
    pToolBar->addActions({measurePointsAction});
    pToolBar->addSeparator();
//    pToolBar->addActions({drawLineAction, modifyLineAction, deleteLineAction});
//    pToolBar->addSeparator();
//    pToolBar->addActions({drawPavementAction, deletePavementAction});
//    pToolBar->addSeparator();
//    pToolBar->addActions({drawDrivingArrowAction, modifyDrivingArrowAction, deleteDrivingArrowAction});
//    pToolBar->addSeparator();
//    pToolBar->addActions({drawTrafficLightsAction, deleteTrafficLightsAction});
//    pToolBar->addSeparator();
//    pToolBar->addActions({drawRoadLinesAction, modifyRoadLinesAction, deleteRoadLinesAction});
//    pToolBar->addSeparator();
    pToolBar->addActions({colorByZAction, colorByIntensityAction, colorByTextureAction});
    pToolBar->addSeparator();

    pToolBar->addActions({cutPointAction, polygonCutPointAction});

}

void MainWindow::initStatusBar() {
    pStatusBar->showMessage("Ready");
}

void MainWindow::createDockWidget() {

    treeWidget->setColumnCount(2);
    treeWidget->setHeaderHidden(true);
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidget, QStringList({"FirstColumn", "SecondColumn"}));
    item->setExpanded(true);

    dockWidget->setWindowTitle(QStringLiteral("场景数据"));
    dockWidget->setFixedWidth(210);
    dockWidget->setFeatures(QDockWidget::AllDockWidgetFeatures);
    dockWidget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dockWidget->setWidget(treeWidget);
    this->addDockWidget(Qt::LeftDockWidgetArea, dockWidget);
}

QString MainWindow::calculateMD5(const QString &filePath) const {
    QFile file(filePath);
    qint64 fileSize = file.size();
    const qint64 bufferSize = 10240;

    if (file.open(QIODevice::ReadOnly)) {
        char buffer[bufferSize];
        int bytesRead;
        int readSize = qMin(fileSize, bufferSize);

        QCryptographicHash hash(QCryptographicHash::Md5);

        while (readSize > 0 && (bytesRead = file.read(buffer, readSize)) > 0) {
            fileSize -= bytesRead;
            hash.addData(buffer, bytesRead);
            readSize = qMin(fileSize, bufferSize);
        }

        file.close();
        return QString(hash.result().toHex());
    }

    return QString();
}

bool MainWindow::matchMD5(const QStringList &list, const QString &filePath) const {
    QFile file(filePath);
    QStringList list2;
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.isEmpty()) {
            continue;
        }
        list2.append(line);
    }

    file.close();

    if (list.size() != list2.size()) {
        return false;
    }

    for (const auto &str : list2) {
        if (!list.contains(str)) {
            return false;
        }
    }

    for (const auto &str : list) {
        if (!list2.contains(str)) {
            return false;
        }
    }

    return true;
}

void MainWindow::doneGernerateOctreeData() {
    progressDialog->cancel();
}

void MainWindow::newProjectDirectoryInfo(const QString &projectName,
                                         const QString &dataFilesDir,
                                         const QString &jsonDataDir,
                                         const QString &csvDataDir,
                                         const QString &mapDataDir) {
    this->projectName = projectName;
    this->dataFilesDir = dataFilesDir;
    this->jsonDataDir = jsonDataDir;
    this->csvDataDir = csvDataDir;
    this->mapDataDir = mapDataDir;


    std::cout<<"------------------dataFilesDir---------------: "<<dataFilesDir.toStdString()<<std::endl;
    std::cout<<"------------------this->dataFilesDir---------------: "<<this->dataFilesDir.toStdString()<<std::endl;
}

void MainWindow::closeEvent(QCloseEvent *e) {
    // 文件未打开
    if (!openFileInfo.fileName().isEmpty()) {
        if (QMessageBox::warning(nullptr,
                                 "Close Current File",
                                 "Save file or not?",
                                 QMessageBox::Yes | QMessageBox::No,
                                 QMessageBox::Yes) == QMessageBox::Yes) {
            saveFile();
        }
    }
    e->accept();
}

void MainWindow::newProject() {

    // 默认路径
    // QString dataFilesDir = QDir::currentPath() + "/pointCloudData/";
    QString jsonDataDir = QDir::currentPath() + "/data/JSONData/";
    QString csvDataDir = QDir::currentPath() + "/data/CSVData/";
    QString mapDataDir = QDir::currentPath() + "/data/MapData/";

//    QString fileName =
//            QFileDialog::getExistingDirectory(this, "Open PointCloud", "../../PointCloudData");
//
//    dataFilesDir = fileName;

    // 读取软件配置INI文件获取已有项目名
    QSettings projectINIFile(QDir::currentPath() + "/.project.ini", QSettings::IniFormat);
    projectINIFile.beginGroup("Projects");
    QStringList projectsNameList = projectINIFile.allKeys();
    projectINIFile.endGroup();

    auto *newProjectDialog = new NewProjectDialog();
    connect(this,
            &MainWindow::newProjectDefaultDirectorySignal,
            newProjectDialog,
            &NewProjectDialog::setAllDefaultDirectory);
    connect(newProjectDialog, &NewProjectDialog::newProjectInfoSignal, this, &MainWindow::newProjectDirectoryInfo);

    emit newProjectDefaultDirectorySignal(projectsNameList, dataFilesDir, jsonDataDir, csvDataDir, mapDataDir);
    newProjectDialog->exec();



    if (projectName.isEmpty()) {
        return;
    }

    // write XML
    QString XMLFilePath = QDir::currentPath() + "/pointCloudData/.projects/" + projectName + ".xml";

    std::cout<<"--------------------XMLFilePath-----------------: "<<XMLFilePath.toStdString()<<std::endl;

    if (!writeProjectXML(XMLFilePath)) {
        QMessageBox::warning(nullptr, "New Project", "New Project Error!", QMessageBox::Yes);
        return;
    }

    // wirte INIFile
//    projectINIFile.beginGroup("Projects");
//    projectINIFile.setValue(projectName, XMLFilePath);
//    projectINIFile.endGroup();


    /*
     * 取组 和 keys
     */
    QStringList all = projectINIFile.childGroups();

    projectINIFile.beginGroup(all[1]);

    // 取分组里的 数据组数
    QStringList keys = projectINIFile.childKeys();

    QStringList NameValues;

    for(auto &key:keys)
    {
        NameValues.append("Project/"+key);
    }

    projectINIFile.endGroup();

    projectINIFile.beginGroup(all[0]);
    // 取分组里的 数据组数
    QStringList Url_keys = projectINIFile.childKeys();

    QStringList UrlValues;

    for(auto &key:Url_keys)
    {
        UrlValues.append("DataFilesPath/"+key);
    }

    projectINIFile.endGroup();

    int size = NameValues.size();

    QString NameStr,DataFilesPathStr,flag;

    // 遍历ini文件 看是否冲突
    for (int i = 0; i < size; ++i) {
        NameStr = projectINIFile.value(NameValues[i]).toString();

        // 写文件存储路径
        DataFilesPathStr = projectINIFile.value(UrlValues[i]).toString();

        if(NameStr == projectName && DataFilesPathStr == dataFilesDir)
        {
            flag = "The file has exists";
        }
    }
    if (flag == "The file has exists") {
        QMessageBox::information(nullptr, "Warring", "The file has exists");
    }

    if (flag != "The file has exists")
    {
        std::cout<<"dataFilesDir: "<<dataFilesDir.toStdString()<<std::endl;

        // 创建.ini 文件
        all = projectINIFile.childGroups();

        //此处只拿用一组举例，有多个可以自行加设循环
        projectINIFile.beginGroup(all[0]);
        //存数据前要保存成QString形式的

        // 取分组里的 数据组数
        keys = projectINIFile.childKeys();

        projectINIFile.setValue(QString::fromStdString("Url_"+std::to_string(keys.length())),dataFilesDir);

        projectINIFile.endGroup();

        projectINIFile.beginGroup(all[1]);
        //存数据前要保存成QString形式的

        projectINIFile.setValue(QString::fromStdString("Name_"+std::to_string(keys.length())),projectName);

        projectINIFile.endGroup();
    }

    osgWidget->readPCDataFromFiles(this->dataFilesDir, hasBeenModified);
    osgWidget->initTerrainManipulator();
}

void MainWindow::openProject() {

    QString filesDir = QFileDialog::getExistingDirectory(this,
                                                         "Select PointCloud Directory",
                                                         "./pointCloudData",
                                                         QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    QDir dir(filesDir);
    QString filesDirName = dir.dirName();
    qDebug() << "filesDirName: " << filesDirName;
    dir.setNameFilters({"*.pcd", "*.txt", "*.las"});
    QStringList filesMD5;

    QFileInfoList list = dir.entryInfoList();
    for (const QFileInfo &fileInfo : list) {
        QString filePath = fileInfo.absoluteFilePath();
        filesMD5.append(calculateMD5(filePath));
    }

    bool hasSaveDirectory = false;
    QString saveFilesDir = QCoreApplication::applicationDirPath() + "/data/";
    qDebug() << "saveFilesDir: " << saveFilesDir;
    dir.setPath(saveFilesDir);
    dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);
    list = dir.entryInfoList();
    for (const QFileInfo &fileInfo : list) {
        if (fileInfo.isDir()) {
            if (filesDirName == fileInfo.fileName()) {
                qDebug() << "filesDirName: " << filesDirName << " fileInfo.fileName: " << fileInfo.fileName();
                hasSaveDirectory = true;
                QString MD5File = fileInfo.absoluteFilePath() + "/.MD5SUM";
                qDebug() << "MD5File: " << MD5File;
                if (QFileInfo(MD5File).exists()) {
                    if (matchMD5(filesMD5, MD5File)) {
                        hasBeenModified = false;
                    }
                }
                break;
            }
        }
    }

    // 第一次读取时没有
    if (!hasSaveDirectory) {
        QString MD5FileDir = saveFilesDir + filesDirName;
        QDir().mkpath(MD5FileDir);
        QFile file(MD5FileDir + "/.MD5SUM");
        qDebug() << "file: " << MD5FileDir + "/.MD5SUM";
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            return;
        }
        QTextStream out(&file);
        for (const auto &MD5Str : filesMD5) {
            out << MD5Str << "\n";
        }
        file.close();
    }

    // 文件被修改MD5值不一致
    if (hasBeenModified) {
        QString MD5FileDir = saveFilesDir + filesDirName;
        QDir dir(MD5FileDir);
        dir.removeRecursively();
        QDir().mkpath(MD5FileDir);
        QFile file(MD5FileDir + "/.MD5SUM");
        qDebug() << "file: " << MD5FileDir + "/.MD5SUM";
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            return;
        }
        QTextStream out(&file);
        for (const auto &MD5Str : filesMD5) {
            out << MD5Str << "\n";
        }
        file.close();
    }

    osgWidget->readPCDataFromFiles(filesDir, hasBeenModified);
    // osgWidget->loadVectorMap(saveFilesDir);
    osgWidget->loadVectorMap();
    osgWidget->initTerrainManipulator();
}

void MainWindow::openFileInfoSlot(QString &fileExtension, bool hasIntensity, bool hasRGB) {
    this->fileExtension = fileExtension;
    this->hasIntensity = hasIntensity;
    this->hasRGB = hasRGB;
}

void MainWindow::saveFile() {
    QDir dir;
    // 调试用上面一个，因为是外围build编译，部署发布用下面，数据和可执行文件在同一层
    // dir.cd("../../");

    // 文件未打开
    if (openFileInfo.fileName().isEmpty()) {
        return;
    }

    QString fileName;

    if (openFileInfo.suffix() == "pcd") {
        fileName = originalPCDFileName;
    } else {
        fileName = openFileInfo.fileName();
    }

    QString dirName = "data/" + fileName + "/";
    if (!dir.exists(dirName)) {
        std::cout << "data directory doesn't exists, try to create it " << std::endl;
        dir.mkpath(dirName);
    }

    dir.cd(dirName);
    std::cout << "save to: " << dir.absolutePath().toStdString() << std::endl;
    osgWidget->saveVectorMap(dir.absolutePath().toStdString());
    osgWidget->transVectorMapToJson(dir.absolutePath().toStdString());
    osgWidget->transVectorMapToCSV(dir.absolutePath().toStdString());
    //osgWidget->transAllPointsToJSON(dir.absolutePath().toStdString());
}

void MainWindow::saveAsFile() {
    QDir dir;
    // 调试用上面一个，因为是外围build编译，部署发布用下面，数据和可执行文件在同一层
    // dir.cd("../../");

    // 文件未打开
    if (openFileInfo.fileName().isEmpty()) {
        return;
    }

    QString fileName;

    if (openFileInfo.suffix() == "pcd") {
        fileName = originalPCDFileName;
    } else {
        fileName = openFileInfo.fileName();
    }

    QString dirName = "data/" + fileName + "/";
    if (!dir.exists(dirName)) {
        std::cout << "data directory doesn't exists, try to create it " << std::endl;
        dir.mkpath(dirName);
    }

    dir.cd(dirName);
    std::cout << "save MetaData to: " << dir.absolutePath().toStdString() << std::endl;
    osgWidget->saveVectorMap(dir.absolutePath().toStdString());

    QString saveDir =
            QFileDialog::getSaveFileName(this, "Save Json and CSV Data", "./data/dummy.json", "Data (*.json *.CSV)");
    if (saveDir.isEmpty()) {
        QMessageBox::warning(nullptr, "Save File", "Save cancel", QMessageBox::Yes);
        return;
    }
    dir.cd(QFileInfo(saveDir).absolutePath());
    std::cout << "save customData to: " << dir.absolutePath().toStdString() << std::endl;
    osgWidget->transVectorMapToJson(dir.absolutePath().toStdString());
    osgWidget->transVectorMapToCSV(dir.absolutePath().toStdString());
    //osgWidget->transAllPointsToJSON(dir.absolutePath().toStdString());
}

void MainWindow::closeCurrentFile() {
    if (QMessageBox::warning(nullptr,
                             "Close Current File",
                             "Save file or not?",
                             QMessageBox::Yes | QMessageBox::No,
                             QMessageBox::Yes) == QMessageBox::Yes) {
        saveFile();
    }
    // 文件关闭时关闭程序不会提醒保存
    openFileInfo.setFile(nullptr);
    osgWidget->reset();
}

void MainWindow::saveJsonFile() {
    QDir dir;
    dir.cd("../../");

    // 文件未打开
    if (openFileInfo.fileName().isEmpty()) {
        return;
    }

    QString dirName = "data/" + openFileInfo.fileName() + "/";
    if (!dir.exists(dirName)) {
        std::cout << "data directory doesn't exists, try to create it " << std::endl;
        dir.mkpath(dirName);
    }

    dir.cd(dirName);
    std::cout << "save to: " << dir.absolutePath().toStdString() << std::endl;
    osgWidget->transVectorMapToJson(dir.absolutePath().toStdString());
}

void MainWindow::quitApp() {
    QApplication::quit();
}

void MainWindow::drawLine(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != drawLineAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = drawLineAction;
    }

    osgWidget->activeLineEditor(isActive);
}

void MainWindow::modifyLine(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != modifyLineAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = modifyLineAction;
    }

    osgWidget->activeLineModification(isActive);
}

void MainWindow::deleteLine(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != deleteLineAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = deleteLineAction;
    }

    osgWidget->activeLineDeletion(isActive);
}

void MainWindow::drawTraceLine(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != drawTraceLineAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = drawTraceLineAction;
    }

    osgWidget->activeTraceLineEditor(isActive);
}

void MainWindow::modifyTraceLine(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != modifyTraceLineAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = modifyTraceLineAction;
    }

    osgWidget->activeTraceLineModification(isActive);
}

void MainWindow::deleteTraceLine(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != deleteTraceLineAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = deleteTraceLineAction;
    }

    osgWidget->activeTraceLineDeletion(isActive);
}

void MainWindow::drawPavement(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != drawPavementAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = drawPavementAction;
    }

    osgWidget->activePavementEditor(isActive);
}

void MainWindow::deletePavement(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != deletePavementAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = deletePavementAction;
    }

    osgWidget->activePavementDeletion(isActive);
}

void MainWindow::drawDrivingArrow(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != drawDrivingArrowAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = drawDrivingArrowAction;
    }

    osgWidget->activeDrivingArrowEditor(isActive);
}

void MainWindow::modifyDrivingArrow(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != modifyDrivingArrowAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = modifyDrivingArrowAction;
    }

    osgWidget->activeDrivingArrowModification(isActive);
}

void MainWindow::deleteDrivingArrow(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != deleteDrivingArrowAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = deleteDrivingArrowAction;
    }

    osgWidget->activeDrivingArrowDeletion(isActive);
}

void MainWindow::drawTrafficLights(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != drawTrafficLightsAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = drawTrafficLightsAction;
    }

    osgWidget->activeTrafficLightsEditor(isActive);
}

void MainWindow::deleteTrafficLights(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != deleteTrafficLightsAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = deleteTrafficLightsAction;
    }

    osgWidget->activeTrafficLightsDeletion(isActive);
}

void MainWindow::drawRoadLines(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != drawRoadLinesAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = drawRoadLinesAction;
    }

    osgWidget->activeRoadLinesEditor(isActive);
}

void MainWindow::modifyRoadLines(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != modifyRoadLinesAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesDeletion(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = modifyRoadLinesAction;
    }

    osgWidget->activeRoadLinesModification(isActive);
}

void MainWindow::deleteRoadLines(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != deleteRoadLinesAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);

            osgWidget->activeMeasurePoints(false);
        }
        clickedEditorAction = deleteRoadLinesAction;
    }

    osgWidget->activeRoadLinesDeletion(isActive);
}

void MainWindow::measurePoints(bool isActive) {
    if (isActive) {
        if (clickedEditorAction && clickedEditorAction != measurePointsAction) {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeLineModification(false);
            osgWidget->activeLineDeletion(false);

            osgWidget->activeTraceLineEditor(false);
            osgWidget->activeTraceLineModification(false);
            osgWidget->activeTraceLineDeletion(false);

            osgWidget->activePavementEditor(false);
            osgWidget->activePavementDeletion(false);

            osgWidget->activeDrivingArrowEditor(false);
            osgWidget->activeDrivingArrowModification(false);
            osgWidget->activeDrivingArrowDeletion(false);

            osgWidget->activeTrafficLightsEditor(false);
            osgWidget->activeTrafficLightsDeletion(false);

            osgWidget->activeRoadLinesEditor(false);
            osgWidget->activeRoadLinesModification(false);
            osgWidget->activeRoadLinesDeletion(false);
        }
        clickedEditorAction = measurePointsAction;
    }

    osgWidget->activeMeasurePoints(isActive);
}

void MainWindow::colorByZ(bool isActive) {
    if (isActive) {
        if (clickedColorAction && clickedColorAction != colorByZAction) {
            clickedColorAction->setChecked(false);
            osgWidget->activeColorByIntensity(false);
            osgWidget->activeColorByTexture(false);
        }
        clickedColorAction = colorByZAction;
    }

    osgWidget->activeColorByZ(isActive);
}

void MainWindow::colorByIntensity(bool isActive) {
    if (isActive) {
        if (clickedColorAction && clickedColorAction != colorByIntensityAction) {
            clickedColorAction->setChecked(false);
            osgWidget->activeColorByZ(false);
            osgWidget->activeColorByTexture(false);
        }
        clickedColorAction = colorByIntensityAction;
    }

    osgWidget->activeColorByIntensity(isActive);
}

void MainWindow::colorByTexture(bool isActive) {
    if (isActive) {
        if (clickedColorAction && clickedColorAction != colorByTextureAction) {
            clickedColorAction->setChecked(false);
            osgWidget->activeColorByZ(false);
            osgWidget->activeColorByIntensity(false);
        }
        clickedColorAction = colorByTextureAction;
    }

    osgWidget->activeColorByTexture(isActive);
}

void MainWindow::help() {

}

void MainWindow::about() {
    QMessageBox::about(this, "About HDMap", "Version: 1.0");
}

bool MainWindow::writeProjectXML(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }

    QDomDocument doc;
    QDomProcessingInstruction instruction;
    instruction = doc.createProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");
    doc.appendChild(instruction);
    doc.appendChild(doc.createComment("项目名后面的时间为软件自动添加（防止命名冲突），exists表示该项目是否被删除"));

    QDomElement root = doc.createElement("project");
    root.setAttribute("exists", "true");
    root.setAttribute("name", QFileInfo(filePath).baseName());
    root.appendChild(doc.createComment("所有地址都有默认值，在软件当前目录下"));
    doc.appendChild(root);

    QDomElement directory = doc.createElement("directory");
    directory.appendChild(doc.createComment("原数据文件夹地址及个数 删除时提示"));

    QDir dir(dataFilesDir);
    dir.setNameFilters({"*.pcd", "*.txt", "*.las"});
    QFileInfoList list = dir.entryInfoList();

    QDomElement dataFiles = doc.createElement("dataFiles");
    dataFiles.setAttribute("dir", dataFilesDir);
    dataFiles.setAttribute("num", list.size());
    for (const QFileInfo &fileInfo : list) {
        QString fileName = fileInfo.fileName();
        QString MD5str = calculateMD5(fileInfo.absoluteFilePath());
        QDomElement MD5 = doc.createElement("MD5");
        MD5.setAttribute("fileName", fileName);
        MD5.setAttribute("MD5", MD5str);
        dataFiles.appendChild(MD5);
    }
    directory.appendChild(dataFiles);

    directory.appendChild(doc.createComment("待加载数据地址（原则上不能自定义）"));
    QDomElement metaData = doc.createElement("metaData");
    metaData.setAttribute("dir", "N/A");
    directory.appendChild(metaData);

    directory.appendChild(doc.createComment("中间文件（分割地面时产生的CSV，默认不产生）"));
    QDomElement groundCSVData = doc.createElement("groundCSVData");
    groundCSVData.setAttribute("dir", "N/A");
    directory.appendChild(groundCSVData);

    directory.appendChild(doc.createComment("输出数据地址（JSON前端）"));
    QDomElement JSONData = doc.createElement("JSONData");
    JSONData.setAttribute("dir", jsonDataDir);
    directory.appendChild(JSONData);

    directory.appendChild(doc.createComment("CSV数据地址"));
    QDomElement CSVData = doc.createElement("CSVData");
    CSVData.setAttribute("dir", csvDataDir);
    directory.appendChild(CSVData);

    directory.appendChild(doc.createComment("map和yaml数据地址"));
    QDomElement mapData = doc.createElement("mapData");
    mapData.setAttribute("dir", mapDataDir);
    directory.appendChild(mapData);

    root.appendChild(directory);

    QTextStream out(&file);
    doc.save(out, 4);
    file.close();

    return true;
}

void MainWindow::cutpoint(bool isActive)
{
    if (isActive){
        if(clickedEditorAction && clickedEditorAction != cutPointAction)
        {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeColorByZ(false);
            osgWidget->activeColorByIntensity(false);
            osgWidget->activeColorByTexture(false);
        }
        clickedEditorAction = cutPointAction;
    } else
    {
        QMessageBox::StandardButtons Msg = QMessageBox::question(nullptr, "Tip", "Do you want to quit editing? If choose 'Yes' this edit will not be returnable",
                                                                 QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Yes);
        switch (Msg)
        {
            case QMessageBox::Cancel : clickedEditorAction->setChecked(true);
                return;
            default:
                break;
        }
    }
    osgWidget->activeClearIrrelevantPoints(isActive);
}

void MainWindow::polygonCutPoint(bool isActive)
{
    if (isActive){
        if(clickedEditorAction && clickedEditorAction != polygonCutPointAction)
        {
            clickedEditorAction->setChecked(false);

            osgWidget->activeLineEditor(false);
            osgWidget->activeColorByZ(false);
            osgWidget->activeColorByIntensity(false);
            osgWidget->activeColorByTexture(false);
            osgWidget->activeClearIrrelevantPoints(false);
        }
        clickedEditorAction = polygonCutPointAction;
    } else
    {
        QMessageBox::StandardButtons Msg = QMessageBox::question(nullptr, "Tip", "Do you want to quit editing? If choose 'Yes' this edit will not be returnable",
                                                                 QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Yes);
        switch (Msg)
        {
            case QMessageBox::Cancel : clickedEditorAction->setChecked(true);
                return;
            default:
                break;
        }
    }
    osgWidget->activePolygonClearIrrelevantPoints(isActive);
}

void MainWindow::showconsole() {
//    auto *addConsoleDialog = new AddConsoleDialog();
//    connect(this,&MainWindow::showConsoleSignal,addConsoleDialog,&AddConsoleDialog::showConsoleSlot);
//    addConsoleDialog->show();

    dock = new QDockWidget(tr("Console"), this);
    dock->setAllowedAreas(Qt::BottomDockWidgetArea);
    dock->setHidden(true);
    consoleList = new QListWidget(dock);
    consoleList->addItems(QStringList()
                                  << NamePath + DataFilesPath
    );
    dock->setWidget(consoleList);
    addDockWidget(Qt::BottomDockWidgetArea, dock);
    viewMenu->addActions({dock->toggleViewAction()});
}

void MainWindow::UpdateConsole(const QString &projectName, const QString &datafile, const QString &jsondata, const QString &csvdata, const QString &mapdata)
{
    std::cout << "The Project name is:" << projectName.toStdString() << std::endl;

    consoleList->clear();

    consoleList->addItems(QStringList() << "Project name: " + projectName);
    consoleList->addItems(QStringList() << "DataFile: " + datafile);
    consoleList->addItems(QStringList() << "JSONData: " + jsondata);
    consoleList->addItems(QStringList() << "CSVData: " + csvdata);
    consoleList->addItems(QStringList() << "mapData: " + mapdata);

    dock->setWidget(consoleList);
    addDockWidget(Qt::BottomDockWidgetArea, dock);
}