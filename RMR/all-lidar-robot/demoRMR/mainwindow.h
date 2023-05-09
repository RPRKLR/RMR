#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
// #include<arpa/inet.h>
// #include<unistd.h>
// #include<sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
// #include "ckobuki.h"
// #include "rplidar.h"
#include <memory>
#include <queue>
#include "robot.h"
#include "map_loader.h"

namespace Ui
{
    class MainWindow;
}

typedef struct
{
    double x;
    double y;
} Point2d;

typedef struct
{
    Point2d coordinate;
} Obstacle;

typedef struct
{
    Point2d position;
    double theta;
    double v;
    double w;
} RobotState;

/// toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
    //  cv::VideoCapture cap;
    int created_map[500][500];
    int path_finding_map[150][150];
    int shortest_map[150][150];
    TMapArea map;
    int actIndex;
    //    cv::Mat frame[3];

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

    void goTranslate();
    void goRotate();
    void robotprocess();
    void laserprocess();
    double regulateSpeed(double error_distance);
    double regulateRotation(double error_angle);
    void floodAlgorithm(Point2d end_point);
    void readMap();
    void correctMap();
    double distance(Point2d p1, Point2d p2);
    double angleDifference(double a1, double a2);
    double clip(double value, double min_value, double max_value);
    std::vector<RobotState> generateMotionSamples(double x, double y, double theta, double v, double w);
    double evaluateTrajectory(/*double x, double y, double theta, double v, double w, */ RobotState end_state, Point2d goal_position);
    RobotState findBestTrajectory(double x, double y, double theta, double v, double w, Point2d goal_position);
    void dwa(double x, double y, double theta, double obstacles, Point2d goal_pos, std::vector<double> velocity_samples, std::vector<double> angular_velocity_samples, double max_linear_velocity, double max_angular_velocity, double max_linear_acceleration, double max_angular_acceleration);
    void findShortestPath(int start_x, int start_y);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();

    void getNewFrame();

    //    double calculateEuclidDistance(Point2d point1, Point2d point2);
    //    Point2d selectDirection(Point2d starting_point, Point2d goal_point, Point2d left_point, Point2d right_point);
    //    std::shared_ptr<Point2d> findObstacleEnd(int x, int y, std::shared_ptr<std::shared_ptr<int>> map, int dimension, int direction);
    //    std::shared_ptr<std::shared_ptr<Point2d>> checkColision(Point2d start_point, std::shared_ptr<std::shared_ptr<int>> map, int dimension, int range);
    //    void task2();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

private:
    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
    int updateLaserPicture;
    LaserMeasurement copyOfLaserData;
    std::string ipaddress;
    Robot robot;
    TKobukiData robotdata;
    int datacounter;
    QTimer *timer;
    short diff_in_left_encounter, diff_in_right_encounter;
    double current_angle = 0, left_wheel_distance, right_wheel_distance, current_x = 0, current_y = 0, angle_goal, distance_from_goal, rotation_speed = 0;
    // I used unsigned data for the old encounter data, so I do not have to deal with overflows
    unsigned short old_left_encounter, old_right_encounter;
    int speed = 0, position_index = 0;
    double x_goal[5] = {0, 0, 0, 0, 1};
    double y_goal[5] = {0.5, 1, 2.0, 2.5, 2.5};
    double forwardspeed;  // mm/s
    double rotationspeed; // omega/s
    bool mapping = true;
    bool nav = true;
    bool create_map = false;
    const double ROBOT_RADIUS = 0.01;
    const double MAX_LINEAR_VELOCITY = 250;
    const double MAX_ANGULAR_VELOCITY = M_PI / 4;
    const double MAX_LINEAR_ACCELERATION = 25;
    const double MAX_ANGULAR_ACCELERATION = (M_PI / 4 / 10);
    const double GOAL_THRESHOLD = 0.1;
    const double OBSTACLE_THRESHOLD = 0.01;
    const double DT = 0.1;
    bool is_navigating = true;
    std::vector<Obstacle> obstacles;
public slots:
    void setUiValues(double robotX, double robotY, double robotFi);
signals:
    void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); /// toto nema telo
};

#endif // MAINWINDOW_H
