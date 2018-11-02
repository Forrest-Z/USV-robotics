#ifndef DIALOGFIXEDSETPOINT_H
#define DIALOGFIXEDSETPOINT_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"

namespace Ui {
class Dialogfixedsetpoint;
}

class Dialogfixedsetpoint : public QDialog {
  Q_OBJECT

 public:
  explicit Dialogfixedsetpoint(QWidget *parent = 0);
  ~Dialogfixedsetpoint();

 private:
  Ui::Dialogfixedsetpoint *ui;
};

#endif  // DIALOGFIXEDSETPOINT_H
