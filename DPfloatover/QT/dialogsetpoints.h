#ifndef DIALOGSETPOINTS_H
#define DIALOGSETPOINTS_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"

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

  void on_RB_fixedpointI_clicked();

  void on_RB_straightlineI_clicked();

 private:
  Ui::Dialogsetpoints *ui;
  void initializesetpointDiag();
};

#endif  // DIALOGSETPOINTS_H
