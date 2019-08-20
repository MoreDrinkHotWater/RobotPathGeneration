//
// Created by zhihui on 4/19/19.
//

#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <QDebug>

#include "DrivingArrowEditorDialog.h"

DrivingArrowEditorDialog::DrivingArrowEditorDialog(std::string *type, int *typeIndex, QWidget *parent) : QDialog(
        parent), pType(type), pTypeIndex(typeIndex), type(), typeIndex(-1) {
    setWindowTitle("DrivingArrowEditor");
    setFixedSize(210, 150);
    setAttribute(Qt::WA_DeleteOnClose);

    pbSave = new QPushButton(this);
    pbSave->setText("Save");
    pbSave->setGeometry(QRect(130, 110, 51, 25));

    connect(pbSave, &QPushButton::clicked, this, &DrivingArrowEditorDialog::save);

    auto *layoutWidget = new QWidget(this);
    layoutWidget->setGeometry(QRect(20, 10, 174, 91));

    auto *verticalLayout = new QVBoxLayout(layoutWidget);
    verticalLayout->setSpacing(6);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    verticalLayout->setContentsMargins(0, 0, 0, 0);

    auto *horizontalLayout = new QHBoxLayout();
    horizontalLayout->setSpacing(6);

    auto *lType = new QLabel(layoutWidget);
    lType->setText("Type");

    horizontalLayout->addWidget(lType);

    auto *horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer);

    typeComboBox = new QComboBox(layoutWidget);
    typeComboBox->setFixedSize(100, 20);
    typeComboBox->addItem("LEFT_ARROW");
    typeComboBox->addItem("RIGHT_ARROW");
    typeComboBox->setCurrentIndex(-1);
    horizontalLayout->addWidget(typeComboBox);

    verticalLayout->addLayout(horizontalLayout);

    auto *horizontalLayout_2 = new QHBoxLayout();
    horizontalLayout_2->setSpacing(6);

    auto *lTypeIndex = new QLabel(layoutWidget);
    lTypeIndex->setText("typeIndex");

    horizontalLayout_2->addWidget(lTypeIndex);

    auto *horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_2->addItem(horizontalSpacer_2);

    typeIndexComboBox = new QComboBox(layoutWidget);
    typeIndexComboBox->setFixedSize(100, 20);
    typeIndexComboBox->addItems({"0", "1", "2", "3"});
    typeIndexComboBox->setCurrentIndex(-1);
    horizontalLayout_2->addWidget(typeIndexComboBox);

    verticalLayout->addLayout(horizontalLayout_2);

}

void DrivingArrowEditorDialog::save() {

    type = typeComboBox->currentText();
    typeIndex = typeIndexComboBox->currentText().toInt();

    if (typeComboBox->currentIndex() == -1 || typeIndexComboBox->currentIndex() == -1) {
        return;
    }

    qDebug() << "type: " << type << "typeIndex: " << typeIndex;
    *pType = type.toStdString();
    *pTypeIndex = typeIndex;
    emit drivingArrowType(type, typeIndex);
    close();
}