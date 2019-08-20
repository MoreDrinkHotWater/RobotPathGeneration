//
// Created by zhihui on 4/25/19.
//

#ifndef HDMAPS_POINTCLOUDAPPLICATION_OPENFILEDIALOG_H_
#define HDMAPS_POINTCLOUDAPPLICATION_OPENFILEDIALOG_H_

#include <QDialog>

class QPushButton;

class QComboBox;

class OpenFileDialog : public QDialog {
 Q_OBJECT
 public:
  explicit OpenFileDialog(QWidget *parent = nullptr);

  ~OpenFileDialog() override = default;

  Q_DISABLE_COPY(OpenFileDialog);

 protected:
  void closeEvent(QCloseEvent *e) override;

 Q_SIGNALS:
  void openFileInfoSignal(QString &, bool, bool);

 private Q_SLOTS:

  void confirm();

 private:
  void cleanup();

 private:

  QPushButton *pbConfirm;

  QComboBox *fileExtensionComboBox;
  QComboBox *intensityComboBox;
  QComboBox *RGBComboBox;

 private:

  QString fileExtension;
  bool hasIntensity;
  bool hasRGB;
};

#endif //HDMAPS_POINTCLOUDAPPLICATION_OPENFILEDIALOG_H_
