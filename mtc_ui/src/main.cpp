
#include <QApplication>
#include <QQuickView>
#include <QQmlContext>
#include <QDebug>
#include <QThread>
#include <QObject>
#include <QDir>
#include <QStringList>
#include <QQmlContext>
#include <QDebug>

#include "ros/ros.h"

#include "yaml-cpp/yaml.h"

#include "handler.h"
#include "communicator.h"
#include "globals.h"



/*
    To add new elements to the UI:
    1. Add a new signal in Handler (e.g., `void newButtonClicked();`)
    2. Add a corresponding slot in Communicator (e.g., `void reportNewButtonClicked();`)
    3. Connect the signal to the slot in main.cpp using `QObject::connect()`
    4. In the QML file, add a new button and connect its `onClicked` signal to the appropriate Handler slot (e.g., `handler.onNew
*/

QStringList getLocoNamesFromConfig() {
    QStringList locoNames;

    QDir configDir(LOCO_CONFIG_DIR);
    QStringList filters;
    filters << "*.yaml" << "*.yml";  // only YAML files
    QFileInfoList fileList = configDir.entryInfoList(filters, QDir::Files);

    for (const QFileInfo &fileInfo : fileList) {
        // Option 1: just use the filename without extension
        // locoNames << fileInfo.baseName();

        // Option 2: optionally, parse YAML and use `name:` field
        YAML::Node node = YAML::LoadFile(fileInfo.absoluteFilePath().toStdString());
        locoNames << QString::fromStdString(node["name"].as<std::string>());
    }

    return locoNames;
}


int main(int argc, char **argv){
    qDebug() << "ENTERED MAIN";
    ros::init(argc, argv, "group_ui");
    qDebug() << "ROS INITIALIZED";
    QApplication app(argc, argv);
    qDebug() << "QT INITIALIZED";


    QThread *communicator_thread = new QThread();
    Handler *handler = new Handler();
    Communicator *communicator = new Communicator();



    communicator->moveToThread(communicator_thread);
    QObject::connect(communicator_thread, &QThread::started, communicator, &Communicator::run);
    QObject::connect(handler, &Handler::dirStopClicked, communicator, &Communicator::reportDirStopClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::dirForwardClicked, communicator, &Communicator::reportDirForwardClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::dirBackwardClicked, communicator, &Communicator::reportDirBackwardClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::sliderValueChanged, communicator, &Communicator::handleSpeedChange, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::point1ButtonClicked, communicator, &Communicator::reportPoint1ButtonClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::point2ButtonClicked, communicator, &Communicator::reportPoint2ButtonClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::shortWhistleClicked, communicator, &Communicator::reportShortWhistleClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::stationDepartClicked, communicator, &Communicator::reportStationDepartClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::stationArriveClicked, communicator, &Communicator::reportStationArriveClicked, Qt::QueuedConnection);
    QObject::connect(handler, &Handler::locoSelected, communicator, &Communicator::handleLocoSelected, Qt::QueuedConnection);

    communicator_thread->start();

    QStringList locoNames = getLocoNamesFromConfig();
    qDebug() << "Found locos:" << locoNames;
    handler->locoNames = locoNames;

    QQuickView view;

    view.rootContext()->setContextProperty("handler", handler);
    view.rootContext()->setContextProperty("locoNames", handler->locoNames);

    view.setResizeMode(QQuickView::SizeRootObjectToView);
    view.setSource(QUrl("qrc:///handler.qml"));
    view.show();


    return app.exec();
}

