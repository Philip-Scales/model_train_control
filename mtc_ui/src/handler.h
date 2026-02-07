#ifndef HANDLER_H
#define HANDLER_H

#include <QObject>

//Handler is responsible for handling user interactions from the UI. 
//It defines signals that are emitted when the user interacts with the UI 
//(e.g., clicking a button, changing a slider value). These signals are then connected to slots in the
// Communicator class, which translates them into ROS messages that are published to the appropriate topics.

class Handler : public QObject 
{
    Q_OBJECT

    Q_PROPERTY(double minSliderValue READ minSliderValue CONSTANT)
    Q_PROPERTY(double maxSliderValue READ maxSliderValue CONSTANT)
    Q_PROPERTY(double sliderIncrement READ sliderIncrement CONSTANT)
    Q_PROPERTY(double sliderValue READ sliderValue WRITE setSliderValue NOTIFY sliderValueChanged)

    //signals are emitted by the Handler when the corresponding UI element is interacted with. 
    //These signals are connected to slots in the Communicator, which then publish ROS messages to the appropriate topics.
signals: 
    void dirStopClicked();
    void dirForwardClicked();
    void dirBackwardClicked();
    void point1ButtonClicked();
    void point2ButtonClicked();

    //Could probably make these into a generic function which takes sound ID as argument.
    void shortWhistleClicked();
    void stationDepartClicked();
    void stationArriveClicked();

    void sliderValueChanged(double);

    void locoSelected(const QString &loco_name);

public:
    Handler(QObject *_parent = nullptr);
    QStringList locoNames;   // <-- ADD THIS

    //slots are functions that are called in response to signals. 
    //In this case, they are called when the corresponding UI element is interacted with (e.g., a button is clicked). 
    //The slots then emit the appropriate signal to notify the Communicator.
public slots: 
    void onDirStopClicked();
    void onDirForwardClicked();
    void onDirBackwardClicked();
    void onSliderPlusClicked();
    void onSliderMinusClicked();
    void onPoint1_button_clicked();
    void onPoint2_button_clicked();

    //Could probably make these into a generic function which takes sound ID as argument.
    void onShort_whistle_clicked();
    void onstation_depart_clicked();
    void onstation_arrive_clicked();

    double minSliderValue () const;
    double maxSliderValue () const;
    double sliderIncrement() const;
    double sliderValue() const;

    void setSliderValue(double _sliderValue);

    void onLocoSelected(int index);  // slot to handle combo box selection


private:
    bool emergencyStop = false;
    bool patrolOnWaypoints = false;

    double m_current_slider_value = 0.0;
    double m_min_slider_value = 0.00;
    double m_max_sldier_value = 1.0;
    double m_slider_increment = 1.0/255.0;

};

#endif // HANDLER_H
