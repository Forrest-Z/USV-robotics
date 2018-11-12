#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      logfilepath(QString::fromUtf8(logsavepath.c_str())),
      onPixRed(13, 13),
      onPixGreen(13, 13) {
  ui->setupUi(this);
  // this->setWindowState(Qt::WindowMaximized);
  // this->setStyleSheet("background-color: black;");
  // ui operation
  ui->PB_connection->setEnabled(true);
  ui->Projectname->setEnabled(true);
  ui->PB_start->setEnabled(false);
  ui->PB_enablePLC->setEnabled(false);
  ui->PB_suspend->setEnabled(false);

  GetCurrentPath();
  initializeLoglist();
  initializestatus();
  updatelogAndStatus();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_action2D_triggered() {
  myDisplay2DDialog = new Display2DDialog(this);
  myDisplay2DDialog->show();
}

void MainWindow::on_actionThruster_P_triggered() {
  myThrusterDiag = new ThrusterDiag(this);
  myThrusterDiag->show();
}

void MainWindow::on_PB_connection_clicked() {
  QString _Projectname = ui->Projectname->text();
  std::string s_projectname = _Projectname.toStdString();
  globalvar::_threadloop.setdbsavepath(s_projectname);
  // initialize the threaded loop
  globalvar::_threadloop.initializelooop();
  // connect to PN server
  globalvar::_threadloop.start_connnection_t();
  // start a thread for gamepad
  globalvar::_threadloop.updategamepad_t();
  // start a thread for motion caputre
  globalvar::_threadloop.updatemotioncapture_t();
  // start a thread for sqlite database
  globalvar::_threadloop.save2database_t();

  // ui operation
  ui->PB_connection->setEnabled(false);
  ui->Projectname->setEnabled(false);
  ui->PB_start->setEnabled(true);
  ui->PB_enablePLC->setEnabled(false);
  ui->PB_suspend->setEnabled(false);
}

void MainWindow::on_PB_start_clicked() {
  // start multithreads for each socket client
  globalvar::_threadloop.controller_t();
  // ui operation
  ui->PB_connection->setEnabled(false);
  ui->Projectname->setEnabled(false);
  ui->PB_start->setEnabled(false);
  ui->PB_enablePLC->setEnabled(true);
  ui->PB_suspend->setEnabled(true);
}

void MainWindow::on_PB_enablePLC_clicked() {
  // start a thread for send/receive using Profinet
  globalvar::_threadloop.send2allclients_pn_t();
  // ui operation
  ui->PB_connection->setEnabled(false);
  ui->Projectname->setEnabled(false);
  ui->PB_start->setEnabled(false);
  ui->PB_enablePLC->setEnabled(false);
  ui->PB_suspend->setEnabled(true);
}

void MainWindow::on_PB_suspend_clicked() {
  globalvar::_threadloop.closelooop();
}

void MainWindow::GetCurrentPath() {
  char *PWD;
  PWD = getenv("PWD");
  globalvar::pwd.append(PWD);
}

void MainWindow::initializeLoglist() {
  // create model
  _model = new QStringListModel(this);
  // Make data
  QStringList loglist;
  loglist << "Program started!";
  // populate our model
  _model->setStringList(loglist);
  // Glue model and view together
  ui->listView_log->setModel(_model);
}

void MainWindow::readfilebyline() {
  QFile inputfile(logfilepath);
  if (inputfile.open(QIODevice::ReadOnly)) {
    QTextStream in(&inputfile);
    QStringList loglist;
    while (!in.atEnd()) {
      QString line = in.readLine();
      loglist << line;
    }
    _model->setStringList(loglist);
  }
}

void MainWindow::updatelogAndStatus() {
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(readfilebyline()));
  connect(timer, SIGNAL(timeout()), this, SLOT(updatestatus()));
  timer->start(1000);
}

void MainWindow::on_RB_headholdI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(1);
}

void MainWindow::on_RB_PIDI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(2);
}

void MainWindow::on_RB_manualI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(0);
}

void MainWindow::on_RB_MPCI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(2);
}

void MainWindow::on_RB_headholdII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(1);
}

void MainWindow::on_RB_PIDII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(2);
}

void MainWindow::on_RB_manualII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(0);
}

void MainWindow::on_RB_MPCII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(2);
}

void MainWindow::on_RB_headholdIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(1);
}

void MainWindow::on_RB_PIDIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(2);
}

void MainWindow::on_RB_manualIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(0);
}

void MainWindow::on_RB_MPCIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(2);
}

void MainWindow::on_actionGamepad_G_triggered() {
  myDialogsetparameter = new Dialogsetparameter(this);
  myDialogsetparameter->show();
}

void MainWindow::on_actionSetpoints_triggered() {
  myDialogfixedsetpoint = new Dialogfixedsetpoint(this);
  myDialogfixedsetpoint->show();
}

void MainWindow::on_actionStraightLine_triggered() {
  myDialogstraightline = new Dialogstraightline(this);
  myDialogstraightline->show();
}

void MainWindow::on_actionRotation_triggered() {
  myDialogRotation = new DialogRotation(this);
  myDialogRotation->show();
}

void MainWindow::on_actionBox_triggered() {
  myDialogBox = new DialogBox(this);
  myDialogBox->show();
}

void MainWindow::on_actionCooperation_triggered() {
  myDialogCooperation = new DialogCooperation(this);
  myDialogCooperation->show();
}

void MainWindow::on_actionLicensing_triggered() {
  myDialogabout = new Dialogabout(this);
  myDialogabout->show();
}

void MainWindow::initializestatus() {
  onPixRed.fill(QColor(255, 111, 105));
  onPixGreen.fill(QColor(150, 206, 180));

  // QTM status
  ui->LB_QTML->setPixmap(onPixRed);
  ui->LB_QTMT->setText(QString("Offline"));

  // I joystick
  ui->LB_joystick1L->setPixmap(onPixRed);
  ui->LB_joystick1T->setText(QString("Offline"));
  // II joystick
  ui->LB_joystick2L->setPixmap(onPixRed);
  ui->LB_joystick2T->setText(QString("Offline"));

  // PN server
  ui->LB_PNL->setPixmap(onPixRed);
  ui->LB_PNT->setText(QString("Offline"));
}

void MainWindow::updatestatus() {
  // QTM status
  if (globalvar::_threadloop.getqtmstatus()) {
    ui->LB_QTML->setPixmap(onPixRed);
    ui->LB_QTMT->setText(QString("Offline"));
  } else {
    ui->LB_QTML->setPixmap(onPixGreen);
    ui->LB_QTMT->setText(QString("Online"));
  }
  // I joystick
  if (globalvar::_threadloop.getgamepadstatus_first()) {
    ui->LB_joystick1L->setPixmap(onPixRed);
    ui->LB_joystick1T->setText(QString("Offline"));
  } else {
    ui->LB_joystick1L->setPixmap(onPixGreen);
    ui->LB_joystick1T->setText(QString("Online"));
  }
  // II joystick
  if (globalvar::_threadloop.getgamepadstatus_second()) {
    ui->LB_joystick2L->setPixmap(onPixRed);
    ui->LB_joystick2T->setText(QString("Offline"));
  } else {
    ui->LB_joystick2L->setPixmap(onPixGreen);
    ui->LB_joystick2T->setText(QString("Online"));
  }
  // PN server
  if (globalvar::_threadloop.getpnserver_status()) {
    ui->LB_PNL->setPixmap(onPixRed);
    ui->LB_PNT->setText(QString("Offline"));
  } else {
    ui->LB_PNL->setPixmap(onPixGreen);
    ui->LB_PNT->setText(QString("Online"));
  }
}
void MainWindow::on_actionCoG_triggered()
{

}
