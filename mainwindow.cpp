#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setCentralWidget(new MouseGUI);
}

MainWindow::~MainWindow()
{
}

