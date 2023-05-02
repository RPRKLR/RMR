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

enum State
{
    DONE,
    GOALSEEK,
    WALLFOLLOW,
};

/// toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
    //  cv::VideoCapture cap;
    int created_map[500][500];
    int path_finding_map[150][150];
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
    void bug2(double x_goal_position, double y_goal_position, LaserMeasurement &sonars);
    double computeRWFRot(LaserMeasurement sonars);
    bool obstacleInWay(double angle_goal, LaserMeasurement sonars);
    double minRange(LaserMeasurement sonars, double start, double end);
    double getDistance(double x_original, double y_original, double x_goal, double y_goal);
    double computeTranslation(LaserMeasurement sonars);
    double computeGoalSeek(double goal_angle);
    double inToRange(double minimal_value, double current_value, double maximum_value);
    double min(double n1, double n2);
    void readMap();
    int findPath(Point2d start_position);
    void correctMap();

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
    double x_goal[3] = {0.5, 0.5, 1.0};
    double y_goal[3] = {0.0, 0.5, 1.0};
    double forwardspeed;  // mm/s
    double rotationspeed; // omega/s
    bool mapping = true;
    bool nav = false;
    State robot_state;
    static constexpr double at_goal_threshold = 0.2;
    int pole[100][2];
    bool create_map = false;
public slots:
    void setUiValues(double robotX, double robotY, double robotFi);
signals:
    void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); /// toto nema telo
};

#endif // MAINWINDOW_H
