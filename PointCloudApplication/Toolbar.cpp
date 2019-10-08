//
// Created by zhihui on 7/23/19.
//
#include "Toolbar.h"

#include <QMainWindow>
#include <QMenu>
#include <QMessageBox>
#include <QCoreApplication>
#include <QSettings>
#include <QDomElement>
#include <QTreeWidget>
#include <QDir>
#include <QTimer>

#include <stdlib.h>
#include <iostream>

#include "MainWindow.h"
#include "NodeNames.h"
#include "NodeTreeSearch.h"
#include <QDockWidget>
#include <QListWidget>
#include <QMainWindow>
#include <QProgressDialog>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <osg/Geode>

//#include "updateProgressDialog.h"


ToolBar::ToolBar(const QString &title, QWidget *parent)

        : QToolBar(parent),
          steps(0),
//          updateProgress(new UpdateProgressDialog),
          progressDialog(new QProgressDialog("Reading files...", QString(), 0, 0, this))
{

    readIniFile();

    menu = new QMenu(title, this);

    for (int i = 0; i < ProjoctNameStr.size(); ++i) {

        TestActionGroup = new QActionGroup(this);

        auto *action = new QAction(this);

        TestActionGroup->addAction(action);

        action->setText(ProjoctNameStr.at(i));

        menu->addAction(action);

        //connect(TestActionGroup, &QAction::triggered, this, &ToolBar::Test);

        // 为了获得点击的项目名 这里我们采用这种的写法 传一个 action 过去
        connect(TestActionGroup, SIGNAL(triggered(QAction*)), this, SLOT(Test(QAction*)));

        connect(action, &QAction::triggered, this, &ToolBar::SendData);
    }

    ManageAction = new QAction(this);
    ManageAction->setText(tr("Manage Projects..."));
    connect(ManageAction, &QAction::triggered, this, &ToolBar::Manage);

    menu->addAction(ManageAction);

    progressDialog->reset();

//    updateProgress->moveToThread(&updateProgressThread);
//
//    connect(this, &ToolBar::tempSignal, updateProgress, &UpdateProgressDialog::updateProgressDialogSlot);
//
//    connect(&updateProgressThread, &QThread::finished, updateProgress, &QObject::deleteLater);

}

//ToolBar::~ToolBar()
//{
//    updateProgressThread.quit();
//    updateProgressThread.wait();
//
//    delete updateProgress;
//}

void ToolBar::readIniFile()
{
    QString filePath;
//    filePath += "/Application/test.ini";
    filePath = QDir::currentPath() + "/.project.ini";

    QSettings settings(filePath, QSettings::IniFormat);

    /*
    * 取组 和 keys
    */
    QStringList all = settings.childGroups();

    settings.beginGroup(all[1]);

    // 取分组里的 数据组数
    QStringList keys = settings.childKeys();

    QStringList NameValues;

    for(auto &key:keys)
    {
        NameValues.append("Project/"+key);
    }

    settings.endGroup();

    settings.beginGroup(all[0]);
    // 取分组里的 数据组数
    QStringList Url_keys = settings.childKeys();

    QStringList UrlValues;

    for(auto &key:Url_keys)
    {
        UrlValues.append("DataFilesPath/"+key);
    }

    settings.endGroup();

    int size = NameValues.size();

    QString NameStr,DataFilesPathStr,flag;

    for (int i = 0; i < size; ++i) {
        NameStr = settings.value(NameValues[i]).toString();
        DataFilesPathStr = settings.value(UrlValues[i]).toString();

        ProjoctNameStr.append(NameStr);

        map.insert(NameStr,DataFilesPathStr);
    }

}

void ToolBar::Test(QAction *action)
{


    QString Text = action->text();

    QString DataFilesPath;

    for (auto &temp:map) {
        QMap<QString,QString>::iterator it = map.find(Text);
        DataFilesPath = it.value();
        break;
    }

    std::cout<<"map.key: "<<Text.toStdString()<<std::endl;
    std::cout<<"map.value: "<<DataFilesPath.toStdString()<<std::endl;

    // 如果有项目被打开了， 则先关闭该项目， 再加载用户点击的项目，并且修改ini文件的次序。

    readXmlFile(DataFilesPath, Text);


    // 如果没有项目被打开， 则直接加载用户点击的项目，并且修改ini文件的次序。

}

void ToolBar::readXmlFile(QString &DataFilesPath, QString &NameStr)
{
    std::cout<<"-----------------------ToolBar::readXmlFile DataFilesPath: "<<DataFilesPath.toStdString()<<std::endl;

//    QString temp = QDir::currentPath() + "/pointCloudData/.projects/" + NameStr + ".xml";
//    std::cout<<"temp: "<<temp.toStdString()<<std::endl;

    QFile file(QDir::currentPath() + "/pointCloudData/.projects/" + NameStr + ".xml");

    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        QMessageBox::information(nullptr, "Title", "Cannot read filr %1",
                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        return;
    }

    QString errorStr;
    int errorLine,errorColumn;

    QDomDocument doc;
    if (!doc.setContent(&file, false, &errorStr, &errorLine,
                        &errorColumn)) {
        QMessageBox::critical(nullptr, tr("Error"),
                              tr("Parse error at line %1, column %2: %3")
                                      .arg(errorLine).arg(errorColumn).arg(errorStr));
        return;
    }

    QDomElement root = doc.documentElement();

    ProjectName =  root.attribute("name");

    std::cout<<"root.tagName: "<<root.tagName().toStdString()<<std::endl;
    std::cout<<"root.attribute(\"name\"): "<<root.attribute("name").toStdString()<<std::endl;
    std::cout<<"root.attribute(\"exists\"): "<<root.attribute("exists").toStdString()<<std::endl;

    if(root.tagName() != "project")
    {
        QMessageBox::critical(nullptr, tr("Error"),
                              tr("Not a project file"));
        return;
    }

    parseProjectElement(root);

    file.close();
}

// 普通函数
void ToolBar::parseProjectElement(const QDomElement &element)
{
    QDomElement directory = element.firstChildElement();
    if (!directory.isNull() && directory.toElement().tagName() == "directory") {
            parseDirectoryElement(directory);
    }
}

void ToolBar::parseDirectoryElement(const QDomElement &element)
{
    QDomElement dataFiles = element.firstChildElement();

    DataFile = dataFiles.attribute("dir");

    std::cout<<"dir: "<<dataFiles.attribute("dir").toStdString()<<std::endl;
    std::cout<<"num: "<<dataFiles.attribute("num").toStdString()<<std::endl;
    QDomElement MD5 = dataFiles.firstChildElement();
    while(!MD5.isNull())
    {
        if (MD5.toElement().tagName() == "MD5") {
            std::cout<<"MD5: "<<MD5.attribute("MD5").toStdString()<<std::endl;
            std::cout<<"fileName: "<<MD5.attribute("fileName").toStdString()<<std::endl;
        }
        MD5 = MD5.nextSiblingElement();
    }
    parseMetaDataElement(dataFiles);
}


void ToolBar::parseMetaDataElement(const QDomElement &element)
{
    QDomElement metaData = element.nextSiblingElement();
    std::cout<<"dir: "<<metaData.attribute("dir").toStdString()<<std::endl;
    parseGroundCSVDataElement(metaData);
}

void ToolBar::parseGroundCSVDataElement(const QDomElement &element)
{
    QDomElement groundCSVData = element.nextSiblingElement();
    std::cout<<"dir: "<<groundCSVData.attribute("dir").toStdString()<<std::endl;
    parseJSONDataElement(groundCSVData);
}

void ToolBar::parseJSONDataElement(const QDomElement &element)
{
    QDomElement JsonData = element.nextSiblingElement();

    JSONData = JsonData.attribute("dir");

    std::cout<<"dir: "<<JsonData.attribute("dir").toStdString()<<std::endl;
    parseCSVDataElement(JsonData);
}

void ToolBar::parseCSVDataElement(const QDomElement &element)
{
    QDomElement CsvData = element.nextSiblingElement();
    CSVData = CsvData.attribute("dir");
    std::cout<<"dir: "<<CsvData.attribute("dir").toStdString()<<std::endl;
    parseMapDataElement(CsvData);
}

void ToolBar::parseMapDataElement(const QDomElement &element)
{
    QDomElement MapData = element.nextSiblingElement();
    mapData = MapData.attribute("dir");
    std::cout<<"dir: "<<MapData.attribute("dir").toStdString()<<std::endl;
}

void ToolBar::Manage()
{
    QMessageBox::information(nullptr, "Title", "Manage",
                             QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
}

void ToolBar::SendData()
{
    std::cout << "start SendData!===========================" << std::endl;
    std::cout<<"ToolBar thread: "<<QThread::currentThreadId()<<std::endl;

    progressDialog->setWindowTitle("HDMaps");
    // 阻塞除当前窗体之外的所有的窗体
    progressDialog->setWindowModality(Qt::ApplicationModal);

    // 设置对话框出现需要等待的时间
    progressDialog->setMinimumDuration(0);

    //处理过程。。。
    t = new QTimer(this);

    connect(t, &QTimer::timeout, this, &ToolBar::tempSlot);

//    updateProgressThread.start();

    t->start(800);

    // 不会阻塞 但显示不出来进度条
    progressDialog->show();

    // 会发生阻塞, 可以显示进度条
//    progressDialog->exec();

    emit SendSignal(ProjectName, DataFile, JSONData, CSVData, mapData);



}

void ToolBar::endprogress()
{
    t->stop();//停止定时器
    if(steps != 100)
        steps = 100;
    progressDialog->setValue(steps);//进度达到最大值
    progressDialog->cancel();//关闭进度对话框
}

void ToolBar::tempSlot()
{

//    std::cout<<"-------------------------tempSlot------------------"<<std::endl;
//
//    steps += 5;
//
//    if(steps< 100)
//    {
//        emit tempSignal(steps, progressDialog);
//    }

    steps += 5;
    if (steps == 100)
        steps = 0;
    progressDialog->setValue(steps);
    QCoreApplication::processEvents();//避免界面冻结
    if (progressDialog->wasCanceled())
        progressDialog->setHidden(true);//隐藏对话框
}
