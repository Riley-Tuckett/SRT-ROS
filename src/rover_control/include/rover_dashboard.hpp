#ifndef ROVER_DASHBOARD_HPP
#define ROVER_DASHBOARD_HPP

#include <QMainWindow>
#include <QLabel>
#include <QGroupBox>
#include <vector>
#include <utility>

class RoverVisualization;

// Main GUI window
class RoverDashboard : public QMainWindow {
    Q_OBJECT

public:
    explicit RoverDashboard(QWidget *parent = nullptr);
    
    void updateDriveData(const std::vector<double>& drive, const std::vector<double>& rotate);
    void updateArmData(const std::vector<double>& joints, const std::vector<double>& gripper);

private:
    QGroupBox* createDriveSection();
    QGroupBox* createArmSection();

    RoverVisualization *visualization_;
    std::vector<QLabel*> drive_labels_;
    std::vector<std::pair<QLabel*, QLabel*>> rotate_labels_;
    std::vector<QLabel*> joint_labels_;
    QLabel *gripper_label_;
    
    std::vector<double> drive_data_;
    std::vector<double> rotate_data_;
    std::vector<double> arm_joints_;
    std::vector<double> gripper_data_;
};

#endif // ROVER_DASHBOARD_HPP
