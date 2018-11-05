#ifndef DIALOGBOX_H
#define DIALOGBOX_H

#include <QDialog>
#include "constants.h"
#include "globalvar.h"
#include "realtimedata.h"
namespace Ui {
class DialogBox;
}

class DialogBox : public QDialog {
  Q_OBJECT

 public:
  explicit DialogBox(QWidget *parent = 0);
  ~DialogBox();

 private slots:
  void on_PB_setupI_clicked();

  void on_PB_resetI_clicked();

  void on_PB_setupII_clicked();

  void on_PB_resetII_clicked();

 private:
  Ui::DialogBox *ui;
  void initializeDialogBox();
};

#endif  // DIALOGBOX_H
