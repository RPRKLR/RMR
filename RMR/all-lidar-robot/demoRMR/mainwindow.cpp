#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <string>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include "map_loader.h"
#include <queue>
#include <cmath>
#include <unordered_set>

/// TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
///  AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
///  NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
///  POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
///  KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
///  AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
///  AK SA DOSTANES NA SKUSKU

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{

    // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress = "192.168.1.164"; //"192.168.1.15"; //"192.168.1.15"; // 192.168.1.11 127.0.0.1
    //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter = 0;
    //  timer = new QTimer(this);
    //    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex = -1;
    useCamera1 = false;

    datacounter = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    /// prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    ///  moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black); // cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine); // styl pera - plna ciara
    pero.setWidth(3);             // hrubka pera -3pixely
    pero.setColor(Qt::green);     // farba je zelena
    QRect rect;
    rect = ui->frame->geometry(); // ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0, 15);
    painter.drawRect(rect);

    if (useCamera1 == true && actIndex > -1) /// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout << actIndex << std::endl;
    }
    else
    {
        if (updateLaserPicture == 1) /// ak mam nove data z lidaru
        {
            updateLaserPicture = 0;

            painter.setPen(pero);
            // teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for (int k = 0; k < copyOfLaserData.numberOfScans /*360*/; k++)
            {
                int dist = copyOfLaserData.Data[k].scanDistance / 20;                                                                                              /// vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0)) + rect.topLeft().x();   // prepocet do obrazovky
                int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0)) + rect.topLeft().y(); // prepocet do obrazovky
                if (rect.contains(xp, yp))                                                                                                                         // ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp), 2, 2);
            }
        }
    }
}

/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

/// toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
///  vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{

    /// TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX
    CKobuki kobuki;

    if (datacounter == 0)
    {
        old_left_encounter = robotdata.EncoderLeft;
        old_right_encounter = robotdata.EncoderRight;
    }
    diff_in_left_encounter = robotdata.EncoderLeft - old_left_encounter;
    diff_in_right_encounter = robotdata.EncoderRight - old_right_encounter;

    left_wheel_distance = kobuki.tickToMeter * diff_in_left_encounter;
    right_wheel_distance = kobuki.tickToMeter * diff_in_right_encounter;

    double delta_fi = (right_wheel_distance - left_wheel_distance) / kobuki.b;

    double delta_s = (right_wheel_distance + left_wheel_distance) / 2;

    current_x += delta_s * cos(current_angle + (delta_fi / 2));
    current_y += delta_s * sin(current_angle + (delta_fi / 2));

    current_angle += delta_fi;

    if (current_angle > PI)
        current_angle -= 2 * PI;
    if (current_angle <= -PI)
        current_angle = current_angle + 2 * PI;
    datacounter++;

    old_left_encounter = robotdata.EncoderLeft;
    old_right_encounter = robotdata.EncoderRight;

    if (position_index < (sizeof(y_goal) / sizeof(y_goal[0])) && is_navigating == true)
    {
        Point2d current_position = {current_x, current_y};
        Point2d goal_position = {x_goal[position_index], y_goal[position_index]};
        if (distance(current_position, goal_position) > GOAL_THRESHOLD)
        {
            RobotState new_state = findBestTrajectory(current_x, current_y, current_angle, speed, rotation_speed, goal_position);
            speed = new_state.v;
            rotation_speed = new_state.w;
            if (rotation_speed < 0.1)
                goTranslate();
            else
                goRotate();
        }
        else
        {
            position_index++;
            std::cout << "At Goal" << std::endl;
        }
    }
    if (follow_created_path == true && path_following_index < path_points.size())
    {
        Point2d current_position = {current_x, current_y};
        Point2d goal_position = {path_points[path_following_index].x, path_points[path_following_index].y};
        if (distance(current_position, goal_position) > GOAL_THRESHOLD)
        {
            RobotState new_state = findBestTrajectory(current_x, current_y, current_angle, speed, rotation_speed, goal_position);
            speed = new_state.v;
            rotation_speed = new_state.w;
            if (rotation_speed < 0.1)
                goTranslate();
            else
                goRotate();
        }
        else
        {
            path_following_index++;
            std::cout << "At Goal" << std::endl;
        }
    }

    //    if (position_index < (sizeof(y_goal) / sizeof(y_goal[0])) && nav)
    //    {

    //        angle_goal = atan2(y_goal[position_index] - current_y, x_goal[position_index] - current_x);
    //        distance_from_goal = sqrt(pow(x_goal[position_index] - current_x, 2) + pow(y_goal[position_index] - current_y, 2));
    //        double delta_angle = angle_goal - current_angle;
    //        std::cout << distance_from_goal << std::endl;
    //        if (delta_angle > M_PI)
    //        {
    //            delta_angle -= M_PI * 2;
    //        }
    //        else if (delta_angle < -M_PI)
    //        {
    //            delta_angle += M_PI * 2;
    //        }
    //        std::cout << delta_angle << std::endl;
    //        if (distance_from_goal <= 0.05)
    //        {
    //            speed = 0;
    //            ++position_index;
    //            goTranslate();
    //        }
    //        else
    //        {
    //            //            if(delta_angle < 0)
    //            //            {
    //            //                rotation_speed = regulateRotation(delta_angle);
    //            //                goRotate();
    //            //            }
    //            //            else if (delta_angle > 0)
    //            //            {
    //            //                rotation_speed = regulateRotation(delta_angle);
    //            //                goRotate();
    //            //            }
    //            //            speed = regulateSpeed(distance_from_goal);
    //            //            goTranslate();
    //            //        }

    //            if (current_angle - angle_goal < -M_PI)
    //            {
    //                rotation_speed = regulateRotation((current_angle - angle_goal) + M_PI * 2);
    //                goRotate();
    //            }
    //            else if (current_angle - angle_goal > M_PI)
    //            {
    //                rotation_speed = regulateRotation((current_angle - angle_goal) - M_PI * 2);
    //                goRotate();
    //            }
    //            speed = regulateSpeed(distance_from_goal);
    //            goTranslate();
    //        } //            if (abs(angle_goal - current_angle) < 0.09 || abs(current_angle - angle_goal) > 2 * PI - 0.09)
    //          //            {
    //          //                rotation_speed = 0.0;
    //          //                speed = regulateSpeed(distance_from_goal);
    //          //                goTranslate();
    //          //            }
    //          //            else {
    //          //                if (current_angle < angle_goal && ((current_angle - angle_goal) < PI))
    //          //                    rotation_speed = regulateRotation(-1*(current_angle - angle_goal));
    //          //                else
    //          //                    rotation_speed = regulateRotation(current_angle - angle_goal);
    //          //                goRotate();
    //          //            }
    //          //            if (speed == 0)
    //          //            {
    //          //                if (abs(angle_goal - current_angle) < 0.09 || abs(current_angle - angle_goal) > 2 * PI - 0.09)
    //          //                    rotation_speed = 0;
    //          //                else if (angle_goal < current_angle && ((current_angle - angle_goal) < PI))
    //          //                    rotation_speed = regulateRotation(-abs(current_angle - angle_goal));
    //          //                else
    //          //                    rotation_speed = regulateRotation(abs(current_angle - angle_goal));
    //          ////            rotation_speed = 2;
    //          //                goRotate();

    //        //            }

    //        //            {
    //        //                if (distance_from_goal > 0.05)
    //        //                    speed = regulateSpeed(distance_from_goal);
    //        //                goTranslate();
    //        //            }
    //    }

    if (datacounter % 5)
    {

        /// ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        // ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        // ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        /// posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit uiValuesChanged(current_x, current_y, (current_angle * 180 / PI));
        /// toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        ///  prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        ///  vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
    }
    datacounter++;

    return 0;
}

/// toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
///  vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{

    memcpy(&copyOfLaserData, &laserData, sizeof(LaserMeasurement));
    // tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    //  ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    // obstacles.clear();
    int counter = 0;
    for (const auto &laser_scan : copyOfLaserData.Data)
    {
        if (counter % 5 == 0)
        {
            if (laser_scan.scanDistance > 100 && laser_scan.scanDistance < 3000 && rotation_speed == 0)
            {
                Point2d coordinate = {laser_scan.scanDistance / 1000 * cos(laser_scan.scanAngle),
                                      laser_scan.scanDistance / 1000 * sin(laser_scan.scanAngle)};
                //            std::cout << coordinate.x << " " << coordinate.y << std::endl;
                bool found_obstacle = false;
                for (const auto &obstacle : obstacles)
                {
                    double dx = obstacle.coordinate.x - coordinate.x;
                    double dy = obstacle.coordinate.y - coordinate.y;
                    double dist = sqrt(dx * dx + dy * dy);
                    if (dist < 0.1)
                    {
                        found_obstacle = true;
                        break;
                    }
                }
                if (!found_obstacle)
                {
                    Obstacle obstacle;
                    obstacle.coordinate = coordinate;
                    obstacles.push_back(obstacle);
                }
            }
        }
        counter++;
    }

    if (rotation_speed == 0 && mapping == true)
    {
        for (int i = 0; i < copyOfLaserData.numberOfScans; i++)
        {

            if (copyOfLaserData.Data[i].scanDistance < 300 || copyOfLaserData.Data[i].scanDistance > 3000 || (copyOfLaserData.Data[i].scanDistance > 640 && copyOfLaserData.Data[i].scanDistance < 700))
            {
                continue;
            }
            else
            //            if (copyOfLaserData.Data[i].scanDistance > 230 || copyOfLaserData.Data[i].scanDistance < 3000)
            {
                double scan_distance = copyOfLaserData.Data[i].scanDistance / 1000;
                int point_y = -(current_y + scan_distance * sin((360 - copyOfLaserData.Data[i].scanAngle) * PI / 180.0 + current_angle)) / 12 * 500 + 500 / 2 - 1;
                int point_x = (current_x + scan_distance * cos((360 - copyOfLaserData.Data[i].scanAngle) * PI / 180.0 + current_angle)) / 12 * 500 + 500 / 2 - 1;
                created_map[point_x][point_y] = 1;
            }
        }

        FILE *fp;
        int u, v;
        fp = fopen("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/map.txt", "w");
        for (u = 0; u < 500; u++)
        {
            for (v = 0; v < 500; v++)
            {
                if (created_map[v][u] == 0)
                    fprintf(fp, "_");
                else
                    fprintf(fp, "%d", created_map[v][u]);
            }
            if (created_map[v][u] == 0)
                fprintf(fp, "_\n");
            else
                fprintf(fp, "%d\n", created_map[v][u]);
        }
        fclose(fp);
    }
    updateLaserPicture = 1;
    update(); // tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

void MainWindow::on_pushButton_9_clicked() // start button
{

    forwardspeed = 0;
    rotationspeed = 0;
    // tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double)));

    /// setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    ///  lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress, 52999, 5299, /*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/ std::bind(&MainWindow::processThisLidar, this, std::placeholders::_1));
    robot.setRobotParameters(ipaddress, 53000, 5300, std::bind(&MainWindow::processThisRobot, this, std::placeholders::_1));

    /// ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();
}

void MainWindow::on_pushButton_2_clicked() // forward
{
    // pohyb dopredu
    for (int i = 0; i < 250; i++)
    {
        robot.setTranslationSpeed(i);
    }
}

void MainWindow::on_pushButton_3_clicked() // back
{
    robot.setTranslationSpeed(-250);
}

void MainWindow::on_pushButton_6_clicked() // left
{
    rotation_speed = 3.14159 / 4;
    robot.setRotationSpeed(rotation_speed);
}

void MainWindow::on_pushButton_5_clicked() // right
{
    rotation_speed = -3.14159 / 4;
    robot.setRotationSpeed(rotation_speed);
}

void MainWindow::on_pushButton_4_clicked() // stop
{
    speed = 0;
    robot.setTranslationSpeed(speed);
    rotation_speed = 0;
    robot.setRotationSpeed(rotation_speed);
}

void MainWindow::on_pushButton_clicked()
{
    if (useCamera1 == true)
    {
        useCamera1 = false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1 = true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{
}

double MainWindow::regulateSpeed(double error_distance)
{
    speed = 1000 * error_distance;
    if (speed > 100)
        speed = 100;
    return speed;
}

double MainWindow::regulateRotation(double error_angle)
{
    rotation_speed = 5 * error_angle;
    if (rotation_speed > PI / 4)
        rotation_speed = PI / 4;
    else if (rotation_speed < -PI / 4)
        rotation_speed = -PI / 4;
    return rotation_speed;
}

void MainWindow::goTranslate()
{
    robot.setTranslationSpeed(speed);
}

void MainWindow::goRotate()
{
    robot.setRotationSpeed(rotation_speed);
}

void MainWindow::readMap()
{
    float x1, x2, y1, y2;

    // walls
    for (int i = 0; i < map.wall.numofpoints - 1; ++i)
    {
        if (i + 1 < map.wall.numofpoints)
        {
            if (map.wall.points[i].point.x < map.wall.points[i + 1].point.x)
            {
                x1 = map.wall.points[i].point.x;
                x2 = map.wall.points[i + 1].point.x;
            }
            else
            {
                x1 = map.wall.points[i + 1].point.x;
                x2 = map.wall.points[i].point.x;
            }
            if (map.wall.points[i].point.y < map.wall.points[i + 1].point.y)
            {
                y1 = map.wall.points[i].point.y;
                y2 = map.wall.points[i + 1].point.y;
            }
            else
            {
                y1 = map.wall.points[i + 1].point.y;
                y2 = map.wall.points[i].point.y;
            }
        }
        x1 = (int)(x1 / 4);
        x2 = (int)(x2 / 4);
        y1 = (int)(y1 / 4);
        y2 = (int)(y2 / 4);
        for (int j = x1; j <= x2; ++j)
        {
            for (int k = y1; k <= y2; ++k)
            {
                path_finding_map[j][k] = 1;
            }
        }
    }
    for (int i = 0; i < 109; ++i)
        path_finding_map[0][i] = 1;

    // Obstacles
    for (int i = 0; i < map.numofObjects; ++i)
    {
        for (int j = 0; j < map.obstacle[i].numofpoints; ++j)
        {
            if (j + 1 < map.obstacle[i].numofpoints)
            {
                if (map.obstacle[i].points[j].point.x < map.obstacle[i].points[j + 1].point.x)
                {
                    x1 = map.obstacle[i].points[j].point.x;
                    x2 = map.obstacle[i].points[j + 1].point.x;
                }
                else
                {
                    x1 = map.obstacle[i].points[j + 1].point.x;
                    x2 = map.obstacle[i].points[j].point.x;
                }
                if (map.obstacle[i].points[j].point.y < map.obstacle[i].points[j + 1].point.y)
                {
                    y1 = map.obstacle[i].points[j].point.y;
                    y2 = map.obstacle[i].points[j + 1].point.y;
                }
                else
                {
                    y1 = map.obstacle[i].points[j + 1].point.y;
                    y2 = map.obstacle[i].points[j].point.y;
                }
            }
            x1 = (int)(x1 / 4);
            x2 = (int)(x2 / 4);
            y1 = (int)(y1 / 4);
            y2 = (int)(y2 / 4);

            for (int k = x1; k <= x2; ++k)
            {
                for (int l = y1; l <= y2; ++l)
                    path_finding_map[k][l] = 1;
            }
        }
    }
}

void MainWindow::floodAlgorithm(Point2d end_point)
{
    path_finding_map[int(end_point.x)][int(end_point.y)] = 2;
    bool is_there = false;
    int map_constant = 150;
    while (1)
    {
        is_there = false;
        for (int i = 0; i < map_constant; ++i)
        {
            for (int j = 0; j < map_constant; ++j)
            {
                if ((path_finding_map[i][j] != 0) && (path_finding_map[i][j] != 1) && path_finding_map[i][j] != 900)
                {
                    if (i - 1 >= 0)
                    {
                        if (path_finding_map[i - 1][j] == 0)
                        {
                            path_finding_map[i - 1][j] = path_finding_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                    if (i + 1 < map_constant)
                    {
                        if (path_finding_map[i + 1][j] == 0)
                        {
                            path_finding_map[i + 1][j] = path_finding_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                    if (j - 1 >= 0)
                    {
                        if (path_finding_map[i][j - 1] == 0)
                        {
                            path_finding_map[i][j - 1] = path_finding_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                    if (j + 1 < map_constant)
                    {
                        if (path_finding_map[i][j + 1] == 0)
                        {
                            path_finding_map[i][j + 1] = path_finding_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                }
            }
        }
        if (is_there == false)
        {
            break;
        }
    }
}

void MainWindow::correctMap()
{
    for (int i = 4; i < 35; ++i)
    {
        for (int j = 4; j < 35; ++j)
        {
            path_finding_map[i][j] = 0;
        }
    }

    for (int i = 0; i < 150; ++i)
    {
        for (int j = 0; j < 150; ++j)
        {
            if (path_finding_map[i][j] == 1)
            {
                if ((j + 1) < 149 && path_finding_map[i][j + 1] == 0)
                    path_finding_map[i][j + 1] = 900;
                if ((j + 2) < 149 && path_finding_map[i][j + 2] == 0)
                    path_finding_map[i][j + 2] = 900;
                if ((j + 3) < 149 && path_finding_map[i][j + 3] == 0)
                    path_finding_map[i][j + 3] = 900;
                if ((j - 1) > 0 && path_finding_map[i][j - 1] == 0)
                    path_finding_map[i][j - 1] = 900;
                if ((j - 2) > 0 && path_finding_map[i][j - 2] == 0)
                    path_finding_map[i][j - 2] = 900;
                if ((j - 3) > 0 && path_finding_map[i][j - 3] == 0)
                    path_finding_map[i][j - 3] = 900;
                if ((i + 1) < 149 && path_finding_map[i + 1][j] == 0)
                    path_finding_map[i + 1][j] = 900;
                if ((i + 2) < 149 && path_finding_map[i + 2][j] == 0)
                    path_finding_map[i + 2][j] = 900;
                if ((i + 3) < 149 && path_finding_map[i + 3][j] == 0)
                    path_finding_map[i + 3][j] = 900;
                if ((i - 1) > 0 && path_finding_map[i - 1][j] == 0)
                    path_finding_map[i - 1][j] = 900;
                if ((i - 2) > 0 && path_finding_map[i - 2][j] == 0)
                    path_finding_map[i - 2][j] = 900;
                if ((i - 3) > 0 && path_finding_map[i - 3][j] == 0)
                    path_finding_map[i - 3][j] = 900;
            }
        }
    }
    // File write implementation here
    std::string temp_str;
    std::ofstream file("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/corrected_map.txt");
    if (file.is_open())
    {
        for (int i = 0; i < 150; ++i)
        {
            for (int j = 0; j < 150; ++j)
            {
                std::string character = std::to_string(path_finding_map[i][j]);
                if (character == "900")
                    character = "A";
                temp_str += character;
            }
            file << temp_str << std::endl;
            temp_str.clear();
        }
        file.close();
    }
}

void MainWindow::on_pushButton_10_clicked()
{

    Point2d end_point = {18, 40};
    map_loader MapLoader;
    char filename[65] = "/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/priestor.txt";
    MapLoader.load_map(filename, map);
    readMap();
    correctMap();
    for (int i = 0; i < 150; ++i)
    {
        for (int j = 0; j < 150; ++j)
        {
            shortest_map[i][j] = path_finding_map[i][j];
        }
    }
    floodAlgorithm(end_point);
    std::string temp_str;
    std::ofstream file("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/flood.txt");
    if (file.is_open())
    {

        for (int i = 0; i < 150; ++i)
        {
            for (int j = 0; j < 150; ++j)
            {
                std::string character = std::to_string(path_finding_map[i][j]);
                if (character == "900")
                    character = "A";
                temp_str += character;
            }
            file << temp_str << std::endl;
            temp_str.clear();
        }
        file.close();
    }
    findShortestPath(10, 10);
    std::ofstream shortest_file("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/shortest_path.txt");
    if (shortest_file.is_open())
    {
        for (int i = 0; i < 150; ++i)
        {
            for (int j = 0; j < 150; ++j)
            {
                std::string character = std::to_string(shortest_map[i][j]);
                if (character == "900")
                    character = "A";
                temp_str += character;
            }
            shortest_file << temp_str << std::endl;
            temp_str.clear();
        }
        shortest_file.close();
    }
    std::ofstream path_file("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/path.txt");
    std::vector<Point2d> path = clearPath();
    path_points = path;
    for (auto &coordinate : path)
    {
        path_file << coordinate.x << " " << coordinate.y << "\n";
    }
    path_file.close();
    std::cout << "Done with the flood";
     follow_created_path = true;
}

void MainWindow::on_pushButton_11_clicked()
{
    create_map = true;
    ui->pushButton_11->setText("Create map");
}

double MainWindow::distance(Point2d p1, Point2d p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

double MainWindow::angleDifference(double a1, double a2)
{
    double diff = a2 - a1;
    if (diff > M_PI)
        diff -= 2 * M_PI;
    else if (diff < -M_PI)
        diff += 2 * M_PI;
    return diff;
}

double MainWindow::clip(double value, double min_value, double max_value)
{
    return std::min(std::max(value, min_value), max_value);
}

std::vector<RobotState> MainWindow::generateMotionSamples(double x, double y, double theta, double v, double w)
{
    std::vector<RobotState> samples;
    for (double linear_velocity = -MAX_LINEAR_VELOCITY; linear_velocity <= MAX_LINEAR_VELOCITY; linear_velocity += MAX_LINEAR_ACCELERATION)
    {
        for (double angular_velocity = -MAX_ANGULAR_VELOCITY; angular_velocity <= MAX_ANGULAR_VELOCITY; angular_velocity += MAX_ANGULAR_ACCELERATION)
        {
            RobotState sample_state;
            sample_state.v = clip(v + linear_velocity, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY); // -250 + (-250) * 0.1 = -275
            sample_state.w = clip(w + angular_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
            sample_state.position.x = x + sample_state.v * cos(theta) * 0.0001; // 10 + (-1) * cos(0) * 0.1 = 0.9  // 10 + 1 cos(0) * 0.1 = 1.1   // 10 + 0  * cos(0) * 0.1 = 10
            sample_state.position.y = y + sample_state.v * sin(theta) * 0.0001; // 10 + (-1) * sin(0) * 0.1 = 10  // 10 + 1 cos(0) * 0.1 = 1.1   // 10 + 0  * cos(0) * 0.1 = 10
            sample_state.theta = theta + sample_state.w * 0.0001;

            samples.push_back(sample_state);
        }
    }
    return samples;
}

double MainWindow::evaluateTrajectory(RobotState end_state, Point2d goal_position)
{
    double distance_to_goal = distance(end_state.position, goal_position);
    double distance_penalty = 1.0f * 10 / (1.0f + distance_to_goal);

    double obstacle_penalty = 1.0f;
    for (const auto &obstacle : obstacles)
    {
        double obstacle_distance = distance(end_state.position, obstacle.coordinate);
        if (obstacle_distance < OBSTACLE_THRESHOLD)
        {
            obstacle_penalty *= (obstacle_distance / OBSTACLE_THRESHOLD);
        }
    }
    double orientation_penalty = 1.0f - abs(angleDifference(end_state.theta, atan2(goal_position.y - end_state.position.y, goal_position.x - end_state.position.x))) / 4*M_PI;
    return distance_penalty * obstacle_penalty * orientation_penalty;
}

RobotState MainWindow::findBestTrajectory(double x, double y, double theta, double v, double w, Point2d goal_position)
{
    std::string temp_str;
    std::ofstream file("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/state_test.txt");

    double best_score = -INFINITY;
    RobotState best_state;
    std::vector<RobotState> samples = generateMotionSamples(x, y, theta, v, w);
    for (const auto &sample : samples)
    {
        double score = evaluateTrajectory(sample, goal_position);
        file << sample.position.x << " " << sample.position.y << " " << sample.theta << " " << sample.v << " " << sample.w << " " << score << std::endl;
        if (score > best_score)
        {
            best_score = score;
            best_state = sample;
        }
    }
    file.close();
    return best_state;
}

void MainWindow::findShortestPath(int start_x, int start_y)
{
    int counter = 1;
    int c_new = 1;
    int current_number = path_finding_map[start_x][start_y];
    while (current_number != 2)
    {
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if ((i == 0 && j == 0 )|| (i == -1 && j==-1) ||(i == 1 && j==-1) ||(i == -1 && j==1) ||(i == 1 && j==1))
                {
                    continue;
                }
                if (path_finding_map[start_x + i][start_y + j] < current_number)
                {
                    current_number = path_finding_map[start_x + i][start_y + j];
                    shortest_map[start_x + i][start_y + j] = counter;
                    Point2d temp_point;
                    temp_point.x = ((double)((start_x + i) * 4) / 100) - 0.4;
                    temp_point.y = ((double)((start_y + j) * 4) / 100) - 0.4;
                    path_points.push_back(temp_point);
                    c_new = counter++;
                    start_x += i;
                    start_y += j;
                }
                if (path_finding_map[start_x + i][start_y + j] == 2)
                {
                    current_number = path_finding_map[start_x + i][start_y + j];
                    shortest_map[start_x + i][start_y + j] = counter;
                    c_new = counter++;
                    Point2d temp_point;
                    temp_point.x = ((double)((start_x + i) * 4) / 100) - 0.4;
                    temp_point.y = ((double)((start_y + j) * 4) / 100) - 0.4;
                    path_points.push_back(temp_point);
                }
                if(c_new != counter)
                {
                    break;
                }
            }
            if(c_new != counter)
            {
                counter = c_new;
                break;
            }
        }
    }
}


std::vector<Point2d> MainWindow::clearPath()
{
    double last_element_angle = atan2(path_points.at(0).y - path_points.at(1).y, path_points.at(0).x -path_points.at(1).x);
    std::vector<Point2d> points;
    points.push_back({0,0});
    for(int i = 1 ; i < path_points.size() - 1; ++i)
    {

        double new_angle = atan2(path_points.at(i).y - path_points.at(i+1).y, path_points.at(i).x - path_points.at(i+1).x);
            if(last_element_angle == new_angle)
            {
            }
            else
            {
                points.push_back(path_points.at(i+1));
            }
            last_element_angle = new_angle;

    }
    points.push_back(path_points.back());
    return points;
}
