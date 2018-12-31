#ifndef DIALOGSTRAIGHTLINE_H
#define DIALOGSTRAIGHTLINE_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"

namespace Ui {
class Dialogstraightline;
}

class Dialogstraightline : public QDialog {
  Q_OBJECT

 public:
  explicit Dialogstraightline(QWidget *parent = 0);
  ~Dialogstraightline();

 private slots:

  void on_RB_VI_clicked();

  void on_RB_VII_clicked();

  void on_RB_VIII_clicked();

  void on_setupsetpoint_clicked();

  void on_resetsetpoint_clicked();

  void on_getcurrentposition_clicked();

 private:
  Ui::Dialogstraightline *ui;
  void initializeStraightlineDiag();
  int index_vessel;
};

#endif  // DIALOGSTRAIGHTLINE_H
