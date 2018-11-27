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

 private slots:
  void on_PB_setup_clicked();

  void on_PB_reset_clicked();

  void on_PB_getcurrentposition_clicked();

private:
  Ui::DialogCooperation *ui;
  void initilizeDialogCooperation();
};

#endif  // DIALOGCOOPERATION_H
