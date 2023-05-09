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

struct Pointt {
    int x, y;
    Pointt() {}
    Pointt(int x, int y) : x(x), y(y) {}
};

struct Node {
    int x, y, f, g, h;
    Node* parent;

    Node(int x_, int y_, int g_, int h_, Node* parent_) :
        x(x_), y(y_), f(g_ + h_), g(g_), h(h_), parent(parent_) {}

    bool operator<(const Node& other) const {
        return f > other.f;
    }
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
    int dist[150][150];
    bool visited[150][150];
    int dx[4] = {1, 0, -1, 0}; // possible x-moves
    int dy[4] = {0, 1, 0, -1}; // possible y-moves
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
//    int findPath(Point2d start_position);
    void correctMap();
    // dwa TEST
    double distance(Point2d p1, Point2d p2);
    double angleDifference(double a1, double a2);
    double clip(double value, double min_value, double max_value);
    std::vector<RobotState> generateMotionSamples(double x, double y, double theta, double v, double w);
    double evaluateTrajectory(/*double x, double y, double theta, double v, double w, */ RobotState end_state, Point2d goal_position);
    RobotState findBestTrajectory(double x, double y, double theta, double v, double w, Point2d goal_position);
    // change the obstacles;
    double calculateCost(double x, double y, double theta, Point2d goal_pos, double obstacles);
    void dwa(double x, double y, double theta, double obstacles, Point2d goal_pos, std::vector<double> velocity_samples, std::vector<double> angular_velocity_samples, double max_linear_velocity, double max_angular_velocity, double max_linear_acceleration, double max_angular_acceleration);
//    std::vector<std::shared_ptr<Node>> aStar(Point2d start, Point2d goal, int map[150][150]);
//    bool isWall(int x, int y);
//    std::vector<std::shared_ptr<Node>> getNeighbors(int x, int y, int map[150][150]);
//    std::vector<std::shared_ptr<Node>> getPath(std::shared_ptr<Node> end_node);
//    void deleteNodes(vector<Node *> &nodes);
//    double euclidean_distance(std::pair<int, int> a, std::pair<int, int> b);
//    bool is_valid(int x, int y);
//    int heuristic(int x, int y, int goal_x, int goal_y);
//    void dijkstra(int start_x, int start_y, int goal_x,int goal_y);
    std::vector<Node*> aStar(Node* start, Node* goal, int map[150][150]);



    bool isValid(int x, int y);
    bool isWalkable(int x, int y, const vector<vector<int>>& map);
    int heuristic(int x1, int y1, int x2, int y2);
    vector<pair<int, int>> getPath(Node* node);
    vector<pair<int, int>> findPath(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal);



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
    static constexpr double at_goal_threshold = 0.2;
    int pole[100][2];
    bool create_map = false;
    const double ROBOT_RADIUS = 0.01;
    const double MAX_LINEAR_VELOCITY = 250;
    const double MAX_ANGULAR_VELOCITY = M_PI / 4;
    const double MAX_LINEAR_ACCELERATION = 25;
    const double MAX_ANGULAR_ACCELERATION = (M_PI / 4 / 10);
    const double GOAL_THRESHOLD = 0.1;
    const double OBSTACLE_THRESHOLD = 0.01;
    const double DT = 0.1;
    const int WALL_VALUE = 1;
    const int GOAL_VALUE = 2;
    const int NO_GO_VALUE = 900;
    const int GOAL = 2;
    std::vector<double> velocity_samples = {25, 50, 75, 100, 125, 150, 175, 200, 225, 250};
    std::vector<double> angular_velocity_samples = {-M_PI / 4, M_PI / 4};
    bool is_navigating = true;
    double goal_tolerance = 0.1;
    Point2d goal_pos = {0, 0};
    std::vector<Obstacle> obstacles;
public slots:
    void setUiValues(double robotX, double robotY, double robotFi);
signals:
    void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); /// toto nema telo
};

#endif // MAINWINDOW_H
