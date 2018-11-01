#ifndef DIALOGSETPOINTS_H
#define DIALOGSETPOINTS_H

#include <QDialog>

namespace Ui {
class Dialogsetpoints;
}

class Dialogsetpoints : public QDialog {
  Q_OBJECT

 public:
  explicit Dialogsetpoints(QWidget *parent = 0);
  ~Dialogsetpoints();

private slots:
    void on_setupsetpointI_clicked();

    void on_resetsetpointI_clicked();

private:
  Ui::Dialogsetpoints *ui;
  void initializesetpointDiag();
};

#endif  // DIALOGSETPOINTS_H
