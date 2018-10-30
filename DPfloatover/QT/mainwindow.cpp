#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  // this->setWindowState(Qt::WindowMaximized);
  // this->setStyleSheet("background-color: black;");
  // ui operation
  ui->PB_connection->setEnabled(true);
  ui->Projectname->setEnabled(true);
  ui->PB_start->setEnabled(false);
  ui->PB_enablePLC->setEnabled(false);
  ui->PB_suspend->setEnabled(false);

  ui->RB_fixedpointI->setEnabled(false);
  ui->RB_straightlineI->setEnabled(false);
  ui->RB_fixedpointII->setEnabled(false);
  ui->RB_straightlineII->setEnabled(false);
  ui->RB_fixedpointIII->setEnabled(false);
  ui->RB_straightlineIII->setEnabled(false);

  GetCurrentPath();
  initializeLoglist();
  updatelog();
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
  //  QString _path = globalvar::pwd + "/data/log";
  QString _path = QString::fromUtf8(logsavepath.c_str());
  QFile inputfile(_path);
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

void MainWindow::updatelog() {
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(readfilebyline()));
  timer->start(1000);
}

void MainWindow::on_RB_headholdI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(1);
}

void MainWindow::on_RB_PIDI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(2);
  // ui operation
  ui->RB_fixedpointI->setEnabled(true);
  ui->RB_straightlineI->setEnabled(true);
}

void MainWindow::on_RB_manualI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(0);
  // ui operation
  ui->RB_fixedpointI->setEnabled(false);
  ui->RB_straightlineI->setEnabled(false);
}

void MainWindow::on_RB_MPCI_clicked() {
  globalvar::_threadloop.setcontrolmode_first(2);
}

void MainWindow::on_RB_headholdII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(1);
}

void MainWindow::on_RB_PIDII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(2);
  // ui operation
  ui->RB_fixedpointII->setEnabled(true);
  ui->RB_straightlineII->setEnabled(true);
}

void MainWindow::on_RB_manualII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(0);
  // ui operation
  ui->RB_fixedpointII->setEnabled(false);
  ui->RB_straightlineII->setEnabled(false);
}

void MainWindow::on_RB_MPCII_clicked() {
  globalvar::_threadloop.setcontrolmode_second(2);
}

void MainWindow::on_RB_headholdIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(1);
}

void MainWindow::on_RB_PIDIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(2);
  // ui operation
  ui->RB_fixedpointIII->setEnabled(true);
  ui->RB_straightlineIII->setEnabled(true);
}

void MainWindow::on_RB_manualIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(0);
  // ui operation
  ui->RB_fixedpointIII->setEnabled(false);
  ui->RB_straightlineIII->setEnabled(false);
}

void MainWindow::on_RB_MPCIII_clicked() {
  globalvar::_threadloop.setcontrolmode_third(2);
}

void MainWindow::on_RB_fixedpointI_clicked() {
  globalvar::_threadloop.setsetpointmode_first(1);
  globalvar::_threadloop.updatesetpoints_t();

  // ui operation
  ui->RB_fixedpointI->setEnabled(false);
  ui->RB_straightlineI->setEnabled(false);
}

void MainWindow::on_RB_straightlineI_clicked() {
  globalvar::_threadloop.setsetpointmode_first(2);
  globalvar::_threadloop.updatesetpoints_t();
  // ui operation
  ui->RB_fixedpointI->setEnabled(false);
  ui->RB_straightlineI->setEnabled(false);
}

void MainWindow::on_RB_fixedpointII_clicked() {
  globalvar::_threadloop.setsetpointmode_second(1);
  globalvar::_threadloop.updatesetpoints_t();

  // ui operation
  ui->RB_fixedpointII->setEnabled(false);
  ui->RB_straightlineII->setEnabled(false);
}

void MainWindow::on_RB_straightlineII_clicked() {
  globalvar::_threadloop.setsetpointmode_second(2);
  globalvar::_threadloop.updatesetpoints_t();

  // ui operation
  ui->RB_fixedpointII->setEnabled(false);
  ui->RB_straightlineII->setEnabled(false);
}

void MainWindow::on_RB_fixedpointIII_clicked() {
  globalvar::_threadloop.setsetpointmode_third(1);
  globalvar::_threadloop.updatesetpoints_t();

  // ui operation
  ui->RB_fixedpointIII->setEnabled(false);
  ui->RB_straightlineIII->setEnabled(false);
}

void MainWindow::on_RB_straightlineIII_clicked() {
  globalvar::_threadloop.setsetpointmode_third(2);
  globalvar::_threadloop.updatesetpoints_t();

  // ui operation
  ui->RB_fixedpointIII->setEnabled(false);
  ui->RB_straightlineIII->setEnabled(false);
}
