//
// Created by zhihui on 9/18/19.
//

#ifndef HDMAPSFORROBOT_UPDATEPROGRESSDIALOG_H
#define HDMAPSFORROBOT_UPDATEPROGRESSDIALOG_H

#include <QProgressDialog>
#include <QObject>

class UpdateProgressDialog : public QObject
{
    Q_OBJECT

public:

    explicit UpdateProgressDialog(QObject *parent= nullptr);

    ~UpdateProgressDialog() override = default;

public:
    void updateProgressDialogSlot(int steps, QProgressDialog *progressDialog);

};

#endif //HDMAPSFORROBOT_UPDATEPROGRESSDIALOG_H
