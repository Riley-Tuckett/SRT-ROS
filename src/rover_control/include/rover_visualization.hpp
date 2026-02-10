#ifndef ROVER_VISUALIZATION_HPP
#define ROVER_VISUALIZATION_HPP

#include <QWidget>
#include <vector>

// Custom widget for rover top-down visualization
class RoverVisualization : public QWidget {
    Q_OBJECT

public:
    explicit RoverVisualization(QWidget *parent = nullptr);
    void setWheelData(const std::vector<double>& angles, const std::vector<double>& drives);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    std::vector<double> wheel_angles_;
    std::vector<double> drive_values_;
};

#endif // ROVER_VISUALIZATION_HPP
