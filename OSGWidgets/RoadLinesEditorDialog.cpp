//
// Created by zhihui on 4/23/19.
//

#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <QDebug>

#include "RoadLinesEditorDialog.h"

RoadLinesEditorDialog::RoadLinesEditorDialog(std::string *type, QWidget *parent) : QDialog(parent), pType(type),
                                                                                   type() {
    setWindowTitle("RoadLinesEditor");
    setFixedSize(161, 101);
    setAttribute(Qt::WA_DeleteOnClose);

    pbSave = new QPushButton(this);
    pbSave->setText("Save");
    pbSave->setGeometry(QRect(100, 60, 41, 25));

    connect(pbSave, &QPushButton::clicked, this, &RoadLinesEditorDialog::save);

    auto *layoutWidget = new QWidget(this);
    layoutWidget->setGeometry(QRect(22, 20, 121, 27));


    auto *horizontalLayout = new QHBoxLayout(layoutWidget);
    horizontalLayout->setSpacing(6);
    horizontalLayout->setContentsMargins(11, 11, 11, 11);
    horizontalLayout->setContentsMargins(0, 0, 0, 0);

    auto *lType = new QLabel(layoutWidget);
    lType->setText("Type");

    horizontalLayout->addWidget(lType);

    auto *horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer);

    typeComboBox = new QComboBox(layoutWidget);
    typeComboBox->addItems({"SOLID_WHITE", "DOUBLE_YELLOW", "S_DASH_YELLOW"});
    typeComboBox->setCurrentIndex(-1);
    horizontalLayout->addWidget(typeComboBox);
}

void RoadLinesEditorDialog::save() {
    type = typeComboBox->currentText();
    if (typeComboBox->currentIndex() == -1) {
        return;
    }
    *pType = type.toStdString();
    close();
}