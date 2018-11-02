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
  void on_setupsetpointII_clicked();

  void on_resetsetpointII_clicked();

  void on_setupsetpointI_clicked();

  void on_resetsetpointI_clicked();

 private:
  Ui::Dialogstraightline *ui;
  void initializeStraightlineDiag();
};

#endif  // DIALOGSTRAIGHTLINE_H
