#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
//    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setupSimpleDemo(QCustomPlot *customPlot);
    void loadFile(const QString &fileName);
private slots:
    void on_actionOpen_triggered();

private:
    QString demoName;
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
