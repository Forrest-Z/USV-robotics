#ifndef DIALOGSETPARAMETER_H
#define DIALOGSETPARAMETER_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"

namespace Ui {
class Dialogsetparameter;
}

class Dialogsetparameter : public QDialog {
  Q_OBJECT

 public:
  explicit Dialogsetparameter(QWidget *parent = 0);
  ~Dialogsetparameter();

private slots:
    void on_PIDsetupI_clicked();

    void on_GB_II_clicked();

private:
  Ui::Dialogsetparameter *ui;
  void initializePIDData();
};

#endif  // DIALOGSETPARAMETER_H
