#ifndef DIALOGROTATION_H
#define DIALOGROTATION_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"
namespace Ui {
class DialogRotation;
}

class DialogRotation : public QDialog {
  Q_OBJECT

 public:
  explicit DialogRotation(QWidget *parent = 0);
  ~DialogRotation();

 private:
  Ui::DialogRotation *ui;
};

#endif  // DIALOGROTATION_H
