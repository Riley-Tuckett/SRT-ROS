#include "rover_dashboard.hpp"
#include "rover_visualization.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QGroupBox>
#include <QFrame>
#include <QPalette>
#include <QFont>
#include <cmath>

RoverDashboard::RoverDashboard(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("SRT Rover Control Dashboard");
    resize(1000, 800);

    // Set dark theme
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(30, 30, 30));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(45, 45, 45));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::Text, Qt::white);
    setPalette(darkPalette);

    // Central widget
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    
    // Header
    QLabel *header = new QLabel("🚀 SRT ROVER DASHBOARD", this);
    header->setAlignment(Qt::AlignCenter);
    QFont headerFont("Arial", 18, QFont::Bold);
    header->setFont(headerFont);
    header->setStyleSheet("color: #00ff00; background-color: #1e1e1e; padding: 10px;");
    mainLayout->addWidget(header);

    // Create horizontal layout for drive and arm data
    QHBoxLayout *dataLayout = new QHBoxLayout();

    // Drive system section
    QGroupBox *driveBox = createDriveSection();
    dataLayout->addWidget(driveBox);

    // Arm system section
    QGroupBox *armBox = createArmSection();
    dataLayout->addWidget(armBox);

    mainLayout->addLayout(dataLayout);

    // Visualization section
    QGroupBox *vizBox = new QGroupBox("Rover Visualization", this);
    vizBox->setStyleSheet("QGroupBox { color: #ffa500; font-weight: bold; font-size: 14px; }");
    QVBoxLayout *vizLayout = new QVBoxLayout();
    
    visualization_ = new RoverVisualization(this);
    vizLayout->addWidget(visualization_);
    
    vizBox->setLayout(vizLayout);
    mainLayout->addWidget(vizBox);

    setCentralWidget(centralWidget);
}

void RoverDashboard::updateDriveData(const std::vector<double>& drive, 
                                     const std::vector<double>& rotate) {
    drive_data_ = drive;
    rotate_data_ = rotate;
    
    // Update drive labels
    for (size_t i = 0; i < drive_labels_.size() && i < drive.size(); ++i) {
        QString text = QString::number(drive[i], 'f', 3);
        drive_labels_[i]->setText(text);
        
        // Color coding
        if (std::abs(drive[i]) > 0.5) {
            drive_labels_[i]->setStyleSheet("color: #00ff00; font-size: 16px; font-weight: bold;");
        } else if (std::abs(drive[i]) > 0.1) {
            drive_labels_[i]->setStyleSheet("color: #ffff00; font-size: 16px; font-weight: bold;");
        } else {
            drive_labels_[i]->setStyleSheet("color: #666666; font-size: 16px; font-weight: bold;");
        }
    }

    // Update rotation labels
    for (size_t i = 0; i < rotate_labels_.size() && i < rotate.size(); ++i) {
        QString valueText = QString::number(rotate[i], 'f', 3);
        double angleDeg = (rotate[i] / 250.0) * 90.0;
        QString angleText = QString::number(angleDeg, 'f', 1) + "°";
        
        rotate_labels_[i].first->setText(valueText);
        rotate_labels_[i].second->setText(angleText);
        
        if (std::abs(rotate[i]) > 10) {
            rotate_labels_[i].first->setStyleSheet("color: #00ff00; font-size: 16px; font-weight: bold;");
        } else {
            rotate_labels_[i].first->setStyleSheet("color: #666666; font-size: 16px; font-weight: bold;");
        }
    }

    // Update visualization
    visualization_->setWheelData(rotate, drive);
}

void RoverDashboard::updateArmData(const std::vector<double>& joints, 
                                   const std::vector<double>& gripper) {
    arm_joints_ = joints;
    gripper_data_ = gripper;

    // Update joint labels
    for (size_t i = 0; i < joint_labels_.size() && i < joints.size(); ++i) {
        QString text = QString::number(joints[i], 'f', 3);
        joint_labels_[i]->setText(text);
    }

    // Update gripper label
    if (!gripper.empty() && gripper_label_) {
        QString text = QString::number(gripper[0], 'f', 3);
        gripper_label_->setText(text);
        
        if (gripper[0] > 0.5) {
            gripper_label_->setStyleSheet("color: #00ff00; font-size: 16px; font-weight: bold;");
        } else {
            gripper_label_->setStyleSheet("color: #ff6666; font-size: 16px; font-weight: bold;");
        }
    }
}

QGroupBox* RoverDashboard::createDriveSection() {
    QGroupBox *box = new QGroupBox("Drive System", this);
    box->setStyleSheet("QGroupBox { color: #ffa500; font-weight: bold; font-size: 14px; }");
    QVBoxLayout *layout = new QVBoxLayout();

    // Motor drives
    QLabel *driveTitle = new QLabel("Motor Drive Values", this);
    driveTitle->setStyleSheet("color: #00ffff; font-weight: bold; font-size: 12px;");
    layout->addWidget(driveTitle);

    QGridLayout *driveGrid = new QGridLayout();
    QStringList motorNames = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
    
    for (int i = 0; i < motorNames.size(); ++i) {
        QFrame *frame = new QFrame(this);
        frame->setFrameStyle(QFrame::Box | QFrame::Raised);
        frame->setStyleSheet("background-color: #3d3d3d;");
        
        QVBoxLayout *frameLayout = new QVBoxLayout(frame);
        
        QLabel *nameLabel = new QLabel(motorNames[i], this);
        nameLabel->setAlignment(Qt::AlignCenter);
        nameLabel->setStyleSheet("color: #00ff00; font-weight: bold;");
        frameLayout->addWidget(nameLabel);
        
        QLabel *valueLabel = new QLabel("0.000", this);
        valueLabel->setAlignment(Qt::AlignCenter);
        valueLabel->setStyleSheet("color: #ffffff; font-size: 16px; font-weight: bold;");
        frameLayout->addWidget(valueLabel);
        drive_labels_.push_back(valueLabel);
        
        driveGrid->addWidget(frame, i / 2, i % 2);
    }
    layout->addLayout(driveGrid);

    // Wheel rotation
    QLabel *rotateTitle = new QLabel("Wheel Steering/Rotation", this);
    rotateTitle->setStyleSheet("color: #00ffff; font-weight: bold; font-size: 12px; margin-top: 10px;");
    layout->addWidget(rotateTitle);

    QGridLayout *rotateGrid = new QGridLayout();
    
    for (int i = 0; i < motorNames.size(); ++i) {
        QFrame *frame = new QFrame(this);
        frame->setFrameStyle(QFrame::Box | QFrame::Raised);
        frame->setStyleSheet("background-color: #3d3d3d;");
        
        QVBoxLayout *frameLayout = new QVBoxLayout(frame);
        
        QLabel *nameLabel = new QLabel(motorNames[i], this);
        nameLabel->setAlignment(Qt::AlignCenter);
        nameLabel->setStyleSheet("color: #00ff00; font-weight: bold;");
        frameLayout->addWidget(nameLabel);
        
        QLabel *valueLabel = new QLabel("0.000", this);
        valueLabel->setAlignment(Qt::AlignCenter);
        valueLabel->setStyleSheet("color: #ffffff; font-size: 16px; font-weight: bold;");
        frameLayout->addWidget(valueLabel);
        
        QLabel *angleLabel = new QLabel("0.0°", this);
        angleLabel->setAlignment(Qt::AlignCenter);
        angleLabel->setStyleSheet("color: #ffff00; font-size: 12px;");
        frameLayout->addWidget(angleLabel);
        
        rotate_labels_.push_back({valueLabel, angleLabel});
        rotateGrid->addWidget(frame, i / 2, i % 2);
    }
    layout->addLayout(rotateGrid);

    box->setLayout(layout);
    return box;
}

QGroupBox* RoverDashboard::createArmSection() {
    QGroupBox *box = new QGroupBox("Arm System", this);
    box->setStyleSheet("QGroupBox { color: #ffa500; font-weight: bold; font-size: 14px; }");
    QVBoxLayout *layout = new QVBoxLayout();

    // Arm joints
    QLabel *jointsTitle = new QLabel("Arm Joint Positions", this);
    jointsTitle->setStyleSheet("color: #00ffff; font-weight: bold; font-size: 12px;");
    layout->addWidget(jointsTitle);

    QStringList jointNames = {"Base", "Shoulder", "Elbow", "Wrist", "Roll"};
    
    for (const QString &name : jointNames) {
        QFrame *frame = new QFrame(this);
        frame->setFrameStyle(QFrame::Box | QFrame::Raised);
        frame->setStyleSheet("background-color: #3d3d3d;");
        
        QHBoxLayout *frameLayout = new QHBoxLayout(frame);
        
        QLabel *nameLabel = new QLabel(name, this);
        nameLabel->setStyleSheet("color: #00ff00; font-weight: bold; font-size: 12px;");
        nameLabel->setMinimumWidth(80);
        frameLayout->addWidget(nameLabel);
        
        QLabel *valueLabel = new QLabel("0.000", this);
        valueLabel->setStyleSheet("color: #ffffff; font-size: 16px; font-weight: bold;");
        valueLabel->setMinimumWidth(100);
        frameLayout->addWidget(valueLabel);
        joint_labels_.push_back(valueLabel);
        
        layout->addWidget(frame);
    }

    // Gripper
    QLabel *gripperTitle = new QLabel("Gripper State", this);
    gripperTitle->setStyleSheet("color: #00ffff; font-weight: bold; font-size: 12px; margin-top: 10px;");
    layout->addWidget(gripperTitle);

    QFrame *gripperFrame = new QFrame(this);
    gripperFrame->setFrameStyle(QFrame::Box | QFrame::Raised);
    gripperFrame->setStyleSheet("background-color: #3d3d3d;");
    
    QHBoxLayout *gripperLayout = new QHBoxLayout(gripperFrame);
    
    QLabel *gripperNameLabel = new QLabel("Gripper Position:", this);
    gripperNameLabel->setStyleSheet("color: #00ff00; font-weight: bold; font-size: 12px;");
    gripperLayout->addWidget(gripperNameLabel);
    
    gripper_label_ = new QLabel("0.000", this);
    gripper_label_->setStyleSheet("color: #ffffff; font-size: 16px; font-weight: bold;");
    gripperLayout->addWidget(gripper_label_);
    
    layout->addWidget(gripperFrame);
    layout->addStretch();

    box->setLayout(layout);
    return box;
}
