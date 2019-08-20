//
// Created by zhihui on 4/23/19.
//

#ifndef HDMAPS_ROADLINESEDITORDIALOG_H
#define HDMAPS_ROADLINESEDITORDIALOG_H

#include <QDialog>
#include <QString>

class QPushButton;

class QComboBox;

class RoadLinesEditorDialog : public QDialog {
    Q_OBJECT
public:
    explicit RoadLinesEditorDialog(std::string *type, QWidget *parent = nullptr);

    ~RoadLinesEditorDialog() override = default;

    Q_DISABLE_COPY(RoadLinesEditorDialog);

private Q_SLOTS:
    void save();

private:
    QPushButton *pbSave;
    QComboBox *typeComboBox;

private:
    std::string *pType;

    QString type;
};


#endif //HDMAPS_ROADLINESEDITORDIALOG_H
