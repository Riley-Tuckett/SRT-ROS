#include "rover_visualization.hpp"
#include <QPainter>
#include <QFont>
#include <cmath>

RoverVisualization::RoverVisualization(QWidget *parent) : QWidget(parent) {
    setMinimumSize(600, 400);
    wheel_angles_ = {0.0, 0.0, 0.0, 0.0};
    drive_values_ = {0.0, 0.0, 0.0, 0.0};
}

void RoverVisualization::setWheelData(const std::vector<double>& angles, 
                                      const std::vector<double>& drives) {
    wheel_angles_ = angles;
    drive_values_ = drives;
    update();
}

void RoverVisualization::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int width = this->width();
    int height = this->height();
    int centerX = width / 2;
    int centerY = height / 2;

    // Draw rover body
    int bodyWidth = 200;
    int bodyHeight = 120;
    QRect body(centerX - bodyWidth/2, centerY - bodyHeight/2, bodyWidth, bodyHeight);
    
    painter.setPen(QPen(QColor(0, 255, 0), 3));
    painter.setBrush(QColor(77, 77, 77));
    painter.drawRect(body);

    // Draw direction arrow
    painter.setPen(QPen(QColor(255, 165, 0), 4));
    painter.setBrush(QColor(255, 165, 0));
    QPoint arrowStart(centerX, centerY);
    QPoint arrowEnd(centerX, centerY - 50);
    painter.drawLine(arrowStart, arrowEnd);
    
    // Arrow head
    QPolygon arrowHead;
    arrowHead << QPoint(centerX, centerY - 50)
              << QPoint(centerX - 8, centerY - 35)
              << QPoint(centerX + 8, centerY - 35);
    painter.drawPolygon(arrowHead);

    // Draw "FRONT" label
    painter.setPen(QColor(255, 165, 0));
    painter.setFont(QFont("Arial", 12, QFont::Bold));
    painter.drawText(centerX - 30, centerY - 60, "FRONT");

    // Wheel positions
    struct WheelPos { int x; int y; QString label; };
    std::vector<WheelPos> wheels = {
        {body.left() - 30, body.top() + 20, "FL"},
        {body.right() + 30, body.top() + 20, "FR"},
        {body.left() - 30, body.bottom() - 20, "RL"},
        {body.right() + 30, body.bottom() - 20, "RR"}
    };

    // Draw wheels
    for (size_t i = 0; i < wheels.size() && i < wheel_angles_.size(); ++i) {
        double angle = (wheel_angles_[i] / 250.0) * 90.0; // Convert to degrees
        double drive = drive_values_[i];
        
        // Wheel color based on drive value
        QColor wheelColor = std::abs(drive) > 0.1 ? QColor(0, 255, 0) : QColor(102, 102, 102);
        
        // Draw wheel circle
        int wheelRadius = 20;
        painter.setPen(QPen(Qt::white, 2));
        painter.setBrush(wheelColor);
        painter.drawEllipse(QPoint(wheels[i].x, wheels[i].y), wheelRadius, wheelRadius);

        // Draw rotation indicator
        painter.setPen(QPen(QColor(255, 255, 0), 3));
        double angleRad = angle * M_PI / 180.0;
        int lineLength = wheelRadius - 5;
        int endX = wheels[i].x + lineLength * std::sin(angleRad);
        int endY = wheels[i].y - lineLength * std::cos(angleRad);
        painter.drawLine(wheels[i].x, wheels[i].y, endX, endY);

        // Draw wheel label
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 10, QFont::Bold));
        painter.drawText(wheels[i].x - 15, wheels[i].y + wheelRadius + 20, wheels[i].label);

        // Draw angle value
        painter.setPen(QColor(255, 255, 0));
        painter.setFont(QFont("Arial", 9));
        QString angleText = QString::number(angle, 'f', 1) + "°";
        painter.drawText(wheels[i].x - 20, wheels[i].y - wheelRadius - 10, angleText);
    }
}
