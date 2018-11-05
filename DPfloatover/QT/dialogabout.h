#ifndef DIALOGABOUT_H
#define DIALOGABOUT_H

#include <QDialog>

namespace Ui {
class Dialogabout;
}

class Dialogabout : public QDialog
{
    Q_OBJECT

public:
    explicit Dialogabout(QWidget *parent = 0);
    ~Dialogabout();

private:
    Ui::Dialogabout *ui;
};

#endif // DIALOGABOUT_H
