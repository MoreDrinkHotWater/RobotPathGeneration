//
// Created by zhihui on 7/4/19.
//

#include <iostream>
#include <string>

#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>
#include <QLineEdit>
#include <QPushButton>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QDate>
#include <QDebug>

#include "NewProjectDialog.h"

NewProjectDialog::NewProjectDialog(QWidget *parent) : QDialog(parent) {

    setWindowTitle("NewProjectDialog");
    setFixedSize(370, 380);
    setAttribute(Qt::WA_DeleteOnClose);

    auto *lProject = new QFrame(this);
    lProject->setGeometry(QRect(0, 20, 371, 16));
    lProject->setFrameShape(QFrame::HLine);
    lProject->setFrameShadow(QFrame::Sunken);

    auto *label_3 = new QLabel(this);
    label_3->setGeometry(QRect(10, 10, 41, 17));
    label_3->setText("Project");
    auto *layoutWidget2 = new QWidget(this);
    layoutWidget2->setGeometry(QRect(20, 40, 331, 27));
    auto horizontalLayout_6 = new QHBoxLayout(layoutWidget2);
    horizontalLayout_6->setSpacing(6);
    horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
    auto *lbProjectName = new QLabel(layoutWidget2);
    lbProjectName->setFixedSize(QSize(60, 25));
    lbProjectName->setText("Name:");

    horizontalLayout_6->addWidget(lbProjectName);

    auto *horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_6->addItem(horizontalSpacer_12);

    leProjectName = new QLineEdit(layoutWidget2);
    leProjectName->setFixedSize(QSize(210, 25));

    horizontalLayout_6->addWidget(leProjectName);

    auto *label_4 = new QLabel(layoutWidget2);
    label_4->setFixedSize(QSize(20, 25));

    horizontalLayout_6->addWidget(label_4);

    auto *horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_6->addItem(horizontalSpacer_13);

    auto *layoutWidget = new QWidget(this);
    layoutWidget->setGeometry(QRect(20, 110, 331, 27));
    auto *horizontalLayout = new QHBoxLayout(layoutWidget);
    horizontalLayout->setSpacing(6);
    horizontalLayout->setContentsMargins(0, 0, 0, 0);

    auto *lbDataFiles = new QLabel(layoutWidget);
    lbDataFiles->setFixedSize(QSize(60, 25));
    lbDataFiles->setText("DataFiles:");

    horizontalLayout->addWidget(lbDataFiles);

    auto *horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer_11);

    leDataFiles = new QLineEdit(layoutWidget);
    leDataFiles->setFixedSize(QSize(210, 25));

    horizontalLayout->addWidget(leDataFiles);

    pbDataFiles = new QPushButton(layoutWidget);
    pbDataFiles->setFixedSize(QSize(20, 25));
    pbDataFiles->setText("...");

    connect(pbDataFiles, &QPushButton::clicked, this, &NewProjectDialog::dataFilesDirectory);

    horizontalLayout->addWidget(pbDataFiles);

    auto *horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer_10);

    auto *lInput = new QFrame(this);
    lInput->setGeometry(QRect(0, 90, 371, 20));
    lInput->setFrameShape(QFrame::HLine);
    lInput->setFrameShadow(QFrame::Sunken);
    auto *label = new QLabel(this);
    label->setGeometry(QRect(10, 80, 84, 17));
    label->setText("Input");

    auto *lOutput = new QFrame(this);
    lOutput->setGeometry(QRect(0, 170, 371, 16));
    lOutput->setFrameShape(QFrame::HLine);
    lOutput->setFrameShadow(QFrame::Sunken);
    auto *label_2 = new QLabel(this);
    label_2->setGeometry(QRect(10, 160, 94, 17));
    label_2->setText("Output");

    auto *layoutWidget1 = new QWidget(this);
    layoutWidget1->setGeometry(QRect(20, 190, 331, 176));
    auto *verticalLayout_2 = new QVBoxLayout(layoutWidget1);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setContentsMargins(0, 0, 0, 0);
    auto *verticalLayout = new QVBoxLayout();
    verticalLayout->setSpacing(6);
    auto *horizontalLayout_2 = new QHBoxLayout();
    horizontalLayout_2->setSpacing(6);
    auto *lbJSONData = new QLabel(layoutWidget1);
    lbJSONData->setFixedSize(QSize(60, 25));
    lbJSONData->setText("JSONData:");

    horizontalLayout_2->addWidget(lbJSONData);

    auto *horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_2->addItem(horizontalSpacer_5);

    leJSONData = new QLineEdit(layoutWidget1);
    leJSONData->setFixedSize(QSize(210, 25));

    horizontalLayout_2->addWidget(leJSONData);

    pbJSONData = new QPushButton(layoutWidget1);
    pbJSONData->setMaximumSize(QSize(20, 25));
    pbJSONData->setText("...");

    connect(pbJSONData, &QPushButton::clicked, this, &NewProjectDialog::jsonDataDirectory);

    horizontalLayout_2->addWidget(pbJSONData);

    auto *horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_2->addItem(horizontalSpacer_2);

    verticalLayout->addLayout(horizontalLayout_2);

    auto *horizontalLayout_3 = new QHBoxLayout();
    horizontalLayout_3->setSpacing(6);
    auto *lbCSVData = new QLabel(layoutWidget1);
    lbCSVData->setFixedSize(QSize(60, 25));
    lbCSVData->setText("CSVData:");

    horizontalLayout_3->addWidget(lbCSVData);

    auto *horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_3->addItem(horizontalSpacer_6);

    leCSVData = new QLineEdit(layoutWidget1);
    leCSVData->setFixedSize(QSize(210, 25));

    horizontalLayout_3->addWidget(leCSVData);

    pbCSVData = new QPushButton(layoutWidget1);
    pbCSVData->setFixedSize(QSize(20, 25));
    pbCSVData->setText("...");

    connect(pbCSVData, &QPushButton::clicked, this, &NewProjectDialog::csvDataDirectory);

    horizontalLayout_3->addWidget(pbCSVData);

    auto *horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_3->addItem(horizontalSpacer_3);

    verticalLayout->addLayout(horizontalLayout_3);

    auto *horizontalLayout_4 = new QHBoxLayout();
    horizontalLayout_4->setSpacing(6);
    auto *lbMapData = new QLabel(layoutWidget1);
    lbMapData->setFixedSize(QSize(60, 25));
    lbMapData->setText("MapData:");

    horizontalLayout_4->addWidget(lbMapData);

    auto *horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_4->addItem(horizontalSpacer_7);

    leMapData = new QLineEdit(layoutWidget1);
    leMapData->setFixedSize(QSize(210, 25));

    horizontalLayout_4->addWidget(leMapData);

    pbMapData = new QPushButton(layoutWidget1);
    pbMapData->setMaximumSize(QSize(20, 25));
    pbMapData->setText("...");

    connect(pbMapData, &QPushButton::clicked, this, &NewProjectDialog::mapDataDirectory);

    horizontalLayout_4->addWidget(pbMapData);

    auto *horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_4->addItem(horizontalSpacer_4);

    verticalLayout->addLayout(horizontalLayout_4);

    auto *verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout->addItem(verticalSpacer_2);

    verticalLayout_2->addLayout(verticalLayout);

    auto *horizontalLayout_5 = new QHBoxLayout();
    horizontalLayout_5->setSpacing(6);
    auto *horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_5->addItem(horizontalSpacer);

    pbCancel = new QPushButton(layoutWidget1);
    pbCancel->setText("Cancel");

    connect(pbCancel, &QPushButton::clicked, this, &NewProjectDialog::cancel);

    horizontalLayout_5->addWidget(pbCancel);

    auto *horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_5->addItem(horizontalSpacer_9);

    pbConfirm = new QPushButton(layoutWidget1);
    pbConfirm->setText("Confirm");

    connect(pbConfirm, &QPushButton::clicked, this, &NewProjectDialog::confirm);

    horizontalLayout_5->addWidget(pbConfirm);

    auto *horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_5->addItem(horizontalSpacer_8);

    verticalLayout_2->addLayout(horizontalLayout_5);
}

void NewProjectDialog::setAllDefaultDirectory(const QStringList &projectsNameList,
                                              const QString &dataFilesDir,
                                              const QString &jsonDataDir,
                                              const QString &csvDataDir,
                                              const QString &mapDataDir) {
    this->projectsNameList = projectsNameList;
    this->dataFilesDir = dataFilesDir;
    this->jsonDataDir = jsonDataDir;
    this->csvDataDir = csvDataDir;
    this->mapDataDir = mapDataDir;

    leDataFiles->setText(dataFilesDir);
    leJSONData->setText(jsonDataDir);
    leCSVData->setText(csvDataDir);
    leMapData->setText(mapDataDir);
}

void NewProjectDialog::dataFilesDirectory() {
    QString dataFilesDir = QFileDialog::getExistingDirectory(this,
                                                             "Select PointCloud Directory",
                                                             "./pointCloudData",
                                                             QFileDialog::ShowDirsOnly
                                                                     | QFileDialog::DontResolveSymlinks);
    if (!dataFilesDir.isEmpty()) {
        leDataFiles->setText(dataFilesDir);
        this->dataFilesDir = dataFilesDir;
    }
}
void NewProjectDialog::jsonDataDirectory() {
    QString jsonDataDir = QFileDialog::getExistingDirectory(this,
                                                            "Select JSONData Directory",
                                                            "./data/JSONData",
                                                            QFileDialog::ShowDirsOnly
                                                                    | QFileDialog::DontResolveSymlinks);
    if (!jsonDataDir.isEmpty()) {
        leJSONData->setText(jsonDataDir);
        this->jsonDataDir = jsonDataDir;
    }
}
void NewProjectDialog::csvDataDirectory() {
    QString csvDataDir = QFileDialog::getExistingDirectory(this,
                                                           "Select CSVData Directory",
                                                           "./data/CSVData",
                                                           QFileDialog::ShowDirsOnly
                                                                   | QFileDialog::DontResolveSymlinks);
    if (!csvDataDir.isEmpty()) {
        leCSVData->setText(csvDataDir);
        this->csvDataDir = csvDataDir;
    }
}
void NewProjectDialog::mapDataDirectory() {
    QString mapDataDir = QFileDialog::getExistingDirectory(this,
                                                           "Select MapData Directory",
                                                           "./data/MapData",
                                                           QFileDialog::ShowDirsOnly
                                                                   | QFileDialog::DontResolveSymlinks);
    if (!mapDataDir.isEmpty()) {
        leMapData->setText(mapDataDir);
        this->mapDataDir = mapDataDir;
    }
}

void NewProjectDialog::cancel() {
    close();
}

void NewProjectDialog::confirm() {

    projectName = leProjectName->text();
    dataFilesDir = leDataFiles->text();
    jsonDataDir = leJSONData->text();
    csvDataDir = leCSVData->text();
    mapDataDir = leMapData->text();

    if (!isProjectNameValid()) {
        QMessageBox::warning(nullptr, "New Project", "Invalid Project Name!", QMessageBox::Yes);
        return;
    }

    if (!QDir(dataFilesDir).exists()) {
        QMessageBox::warning(nullptr, "New Project", "DataFilesDirectory does not exist!", QMessageBox::Yes);
        return;
    }

    if (!QDir(jsonDataDir).exists()) {
        QMessageBox::warning(nullptr, "New Project", "JSONDataDirectory does not exist!", QMessageBox::Yes);
        return;
    }

    if (!QDir(csvDataDir).exists()) {
        QMessageBox::warning(nullptr, "New Project", "CSVDataDirectory does not exist!", QMessageBox::Yes);
        return;
    }

    if (!QDir(mapDataDir).exists()) {
        QMessageBox::warning(nullptr, "New Project", "MapDataDirectory does not exist!", QMessageBox::Yes);
        return;
    }

    emit newProjectInfoSignal(projectName, dataFilesDir, jsonDataDir, csvDataDir, mapDataDir);

    auto s = dataFilesDir.toStdString();

    std::cout<<"newProjectInfoSignal dataFilesDir: "<<s<<std::endl;

    close();
}

bool NewProjectDialog::isProjectNameValid() {
    if (projectName.isEmpty()) {
        return false;
    }

    QString currentDate = QDate::currentDate().toString("yyyy-MM-dd");
    projectName += "_" + currentDate;

    for (auto it = projectsNameList.constBegin(); it != projectsNameList.constEnd(); ++it) {
        if ((*it).constData() == projectName) {
            return false;
        }
    }

    return true;
}