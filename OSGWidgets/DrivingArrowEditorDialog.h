//
// Created by zhihui on 4/19/19.
//

#ifndef HDMAPS_DRIVINGARROWEDITORDIALOG_H
#define HDMAPS_DRIVINGARROWEDITORDIALOG_H

#include <QDialog>
#include <QString>

class QPushButton;

class QComboBox;

class DrivingArrowEditorDialog : public QDialog {
    Q_OBJECT
public:
    explicit DrivingArrowEditorDialog(std::string *type, int *typeIndex, QWidget *parent = nullptr);

    ~DrivingArrowEditorDialog() override = default;

    Q_DISABLE_COPY(DrivingArrowEditorDialog);

Q_SIGNALS:

    // 暂时用不到
    void drivingArrowType(const QString &type, const int typeIndex);

private Q_SLOTS:

    void save();

private:
    QPushButton *pbSave;
    QComboBox *typeComboBox;
    QComboBox *typeIndexComboBox;

private:
    std::string *pType;
    int *pTypeIndex;

    QString type;
    int typeIndex;

};


#endif //HDMAPS_DRIVINGARROWEDITORDIALOG_H
