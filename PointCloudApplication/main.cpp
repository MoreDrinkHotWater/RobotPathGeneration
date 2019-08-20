//
// Created by zhihui on 3/27/19.
//

#include <QApplication>

#include "MainWindow.h"

int main(int argc, char *argv[]) {
    // suppress the qt style warning
    qputenv("QT_STYLE_OVERRIDE", "");
    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.setGeometry(100, 100, 1500, 800);
    mainWindow.show();

    return app.exec();
}