#ifndef DIALOGCOOPERATION_H
#define DIALOGCOOPERATION_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"

namespace Ui {
class DialogCooperation;
}

class DialogCooperation : public QDialog {
  Q_OBJECT

 public:
  explicit DialogCooperation(QWidget *parent = 0);
  ~DialogCooperation();

 private:
  Ui::DialogCooperation *ui;
};

#endif  // DIALOGCOOPERATION_H
