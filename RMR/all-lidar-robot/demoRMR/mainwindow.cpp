#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <string>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
// #include "map_loader.h"

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
    ipaddress = "192.168.1.15"; //"192.168.1.15"; // 192.168.1.11 127.0.0.1
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
    //    if (current_angle > 2 * PI)
    //        current_angle -= 2 * PI;
    //    if (current_angle < 0)
    //        current_angle += 2 * PI;

    if (current_angle > PI)
        current_angle -= 2 * PI;
    if (current_angle <= -PI)
        current_angle = current_angle + 2 * PI;
    datacounter++;

    old_left_encounter = robotdata.EncoderLeft;
    old_right_encounter = robotdata.EncoderRight;

    if (position_index < (sizeof(y_goal) / sizeof(y_goal[0])) && nav)
    {

        angle_goal = atan2(y_goal[position_index] - current_y, x_goal[position_index] - current_x);
        distance_from_goal = sqrt(pow(x_goal[position_index] - current_x, 2) + pow(y_goal[position_index] - current_y, 2));
        double delta_angle = angle_goal - current_angle;
        std::cout << distance_from_goal << std::endl;
        if (delta_angle > M_PI)
        {
            delta_angle -= M_PI * 2;
        }
        else if (delta_angle < -M_PI)
        {
            delta_angle += M_PI * 2;
        }
        std::cout << delta_angle << std::endl;
        if (distance_from_goal <= 0.05)
        {
            speed = 0;
            ++position_index;
            goTranslate();
        }
        else
        {
            //            if(delta_angle < 0)
            //            {
            //                rotation_speed = regulateRotation(delta_angle);
            //                goRotate();
            //            }
            //            else if (delta_angle > 0)
            //            {
            //                rotation_speed = regulateRotation(delta_angle);
            //                goRotate();
            //            }
            //            speed = regulateSpeed(distance_from_goal);
            //            goTranslate();
            //        }

            if (current_angle - angle_goal < -M_PI)
            {
                rotation_speed = regulateRotation((current_angle - angle_goal) + M_PI * 2);
                goRotate();
            }
            else if (current_angle - angle_goal > M_PI)
            {
                rotation_speed = regulateRotation((current_angle - angle_goal) - M_PI * 2);
                goRotate();
            }
            speed = regulateSpeed(distance_from_goal);
            goTranslate();
        } //            if (abs(angle_goal - current_angle) < 0.09 || abs(current_angle - angle_goal) > 2 * PI - 0.09)
          //            {
          //                rotation_speed = 0.0;
          //                speed = regulateSpeed(distance_from_goal);
          //                goTranslate();
          //            }
          //            else {
          //                if (current_angle < angle_goal && ((current_angle - angle_goal) < PI))
          //                    rotation_speed = regulateRotation(-1*(current_angle - angle_goal));
          //                else
          //                    rotation_speed = regulateRotation(current_angle - angle_goal);
          //                goRotate();
          //            }
          //            if (speed == 0)
          //            {
          //                if (abs(angle_goal - current_angle) < 0.09 || abs(current_angle - angle_goal) > 2 * PI - 0.09)
          //                    rotation_speed = 0;
          //                else if (angle_goal < current_angle && ((current_angle - angle_goal) < PI))
          //                    rotation_speed = regulateRotation(-abs(current_angle - angle_goal));
          //                else
          //                    rotation_speed = regulateRotation(abs(current_angle - angle_goal));
          ////            rotation_speed = 2;
          //                goRotate();

        //            }

        //            {
        //                if (distance_from_goal > 0.05)
        //                    speed = regulateSpeed(distance_from_goal);
        //                goTranslate();
        //            }
    }

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
    if (rotation_speed == 0 && mapping == true)
    {
        for (int i = 0; i < copyOfLaserData.numberOfScans; ++i)
        {

            if (copyOfLaserData.Data[i].scanDistance > 145)
            {
                double scan_distance = copyOfLaserData.Data[i].scanDistance / 10000;
                int point_y = -(current_y + scan_distance * sin((360 - copyOfLaserData.Data[i].scanAngle) * PI / 180.0 + current_angle)) / 12 * 500 + 500 / 2 - 1;
                int point_x = (current_x + scan_distance * cos((360 - copyOfLaserData.Data[i].scanAngle) * PI / 180.0 + current_angle)) / 12 * 500 + 500 / 2 - 1;
                created_map[point_x][point_y] = 1;
            }
        }
        std::string temp_str;
        std::ofstream file("/home/pdvorak/rmr_school/School/RMR/all-lidar-robot/map.txt");
        if (file.is_open())
        {

            for (int i = 0; i < 500; ++i)
            {
                for (int j = 0; j < 500; ++j)
                {

                    std::string character = std::to_string(created_map[i][j]);
                    if (character != "1")
                        character = "_";
                    temp_str += character;
                }
                file << temp_str << std::endl;
                temp_str.clear();
            }
            file.close();
        }
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
    robot.setTranslationSpeed(500);
}

void MainWindow::on_pushButton_3_clicked() // back
{
    robot.setTranslationSpeed(-250);
}

void MainWindow::on_pushButton_6_clicked() // left
{
    robot.setRotationSpeed(3.14159 / 2);
}

void MainWindow::on_pushButton_5_clicked() // right
{
    robot.setRotationSpeed(-3.14159 / 2);
}

void MainWindow::on_pushButton_4_clicked() // stop
{
    robot.setTranslationSpeed(0);
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
    if (speed > 500)
        speed = 500;
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

double MainWindow::calculateEuclidDistance(Point2d point1, Point2d point2)
{
    // Calculating Euclid distance between two points
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}

Point2d MainWindow::selectDirection(Point2d starting_point, Point2d goal_point, Point2d left_point, Point2d right_point)
{
    // Calculate the euclid distance from the start point to the left obstacle corner, and from the left obstacle corner to the goal point
    double distance1 = calculateEuclidDistance(starting_point, left_point) + calculateEuclidDistance(left_point, goal_point);
    // Same just with the right obstacle corner
    double distance2 = calculateEuclidDistance(starting_point, right_point) + calculateEuclidDistance(right_point, goal_point);

    if (distance1 < distance2)
    {
        left_point.distance = distance1;
        return left_point;
    }
    else
    {
        right_point.distance = distance2;
        return right_point;
    }
}
// std::shared_ptr<Point2d> MainWindow::findObstacleEnd(int x, int y, std::shared_ptr<std::shared_ptr<int>> map, int dimension, int direction)
//{
//    int i = y;
//    if (direction == 1) // checking to the right
//    {
//        while (map[x][i] == 1)
//        {
//            i++;
//            if (i > dimension)
//            {
//                return NULL;
//            }
//        }
//    }
//    else if (direction == -1)
//    {
//        while(map[x][i] == 1)
//        {
//            i--;
//            if (i < 0)
//            {
//                return NULL;
//            }
//        }
//    }
//    std::shared_ptr<Point2d> point = std::make_shared<Point2d>();
//    point->y = i;
//    point->x = x;
//    return point;
//}
// std::shared_ptr<std::shared_ptr<Point2d>> MainWindow::checkColision(Point2d start_point, std::shared_ptr<std::shared_ptr<int>> map, int dimension, int range)
//{
//    int x = start_point.x + range;
//    int y = start_point.y;
//    if (x < dimension && map[x][y] == 1)
//    {
//        std::cout << "Colision in front of the robot. Finding end of the obstacle\n";
//        std::shared_ptr<Point2d> point1 findObstacleEnd(x, y, map, dimension, -1);
//        std::shared_ptr<Point2d> point2 fintObstacleEnd(x, y, map, dimension, 1);
//        std::shared_ptr<std::shared_ptr<Point2d>> array = std::make_shared<std::make_shared<Point2d>>()[2];
//        array[0] = point1;
//        array[1] = point2;
//        return array;
//    }
//    else
//    {
//        std::cout << "No colision was detected";
//        return NULL;
//    }
//}
// void MainWindow::task2()
//{

//}

void MainWindow::floodAlgorithm(Point2d end_point)
{
    created_map[int(end_point.x)][int(end_point.y)];
    bool is_there;
    while (1)
    {
        for (int i = 0; i < 500; ++i)
        {
            for (int j = 0; j < 500; ++j)
            {
                if ((created_map[i][j] != 0) && (created_map[i][j] != 1) && created_map[i][j] != 900)
                {
                    if (i - 1 >= 0)
                    {
                        if (created_map[i - 1][j] == 0)
                        {
                            created_map[i - 1][j] = created_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                    if (i + 1 < 500)
                    {
                        if (created_map[i + 1][j] == 0)
                        {
                            created_map[i + 1][j] = created_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                    if (j - 1 >= 0)
                    {
                        if (created_map[i][j - 1] == 0)
                        {
                            created_map[i][j - 1] = created_map[i][j] + 1;
                            is_there = true;
                        }
                    }
                    if (j + 1 < 500)
                    {
                        if (created_map[i][j + 1] == 0)
                        {
                            created_map[i][j + 1] = created_map[i][j] + 1;
                            is_there = true;
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
}

// int MainWindow::findPath(int map[500][500], Point2d start_position)
//{
//     while (map[int(start_position.x)][int(start_position.y)] != 2)
//     {
//         if (map[int(start_position.x)][int(start_position.y)] == (map[int(start_position.x) + 1][int(start_position.y)]) + 1)
//         {
//             int temp = start_position.x;
//             while (map[temp][int(start_position.y)] == (map[temp + 1][int(start_position.y)]) + 1)
//             {
//                 temp += 1;
//             }
//         }
//     }
// }

double MainWindow::getDistance(double x_original, double y_original, double x_goal, double y_goal)
{
    return sqrt(pow(x_original - x_goal, 2) + pow(y_original - y_goal, 2));
}

double MainWindow::computeTranslation(LaserMeasurement sonars)
{
    double min_sonar_front;
    min_sonar_front = minRange(sonars, -M_PI / 4, M_PI / 4);
    if (min_sonar_front < 200)
        return 0;
    else
        return min(500, min_sonar_front - 200);
}

double MainWindow::min(double n1, double n2)
{
    if (n1 > n2)
        return n2;
    else
        return n1;
}

double MainWindow::computeGoalSeek(double goal_angle)
{
    if (abs(goal_angle) < M_PI / 10)
        return 0;
    else
        return goal_angle * 100;
}

double MainWindow::minRange(LaserMeasurement sonars, double start, double end)
{
    return 10;
}

bool MainWindow::obstacleInWay(double angle_goal, LaserMeasurement sonars)
{
    int min_sonar_value = 0;
    min_sonar_value = minRange(sonars, angle_goal - (M_PI / 4), angle_goal + (M_PI / 4));
    return (min_sonar_value < 200);
}

double MainWindow::computeRWFRot(LaserMeasurement sonars)
{
    double min_left, min_right, desired_turn;
    min_right = minRange(sonars, -M_PI / 2, 0);
    min_left = minRange(sonars, 0, M_PI / 2);
    if (max(min_right, min_left) < 200)
        return 400; // Hard left
    else
    {
        desired_turn = (400 - min_right) * 2;
        desired_turn = inToRange(-400, desired_turn, 400);
        return desired_turn;
    }
}

double MainWindow::inToRange(double minimal_value, double current_value, double maximum_value)
{
    if (current_value < minimal_value)
    {
        current_value = minimal_value;
    }
    if (current_value > maximum_value)
    {
        current_value = maximum_value;
    }
    return current_value;
}

void MainWindow::bug2(double x_goal_position, double y_goal_position, LaserMeasurement &sonars)
{
    bool at_goal = false;
    while (!at_goal)
    {
        double distance_to_goal = getDistance(current_x, current_y, x_goal_position, y_goal_position);
        double goal_angle = atan2(y_goal_position - current_y, x_goal_position - current_x);
        // ADD GOAL THRESHOLD AS A CONSTANT VARIABLE, NOW I AM DOING IT HERE
        if (distance_to_goal < at_goal_threshold)
        {
            std::cout << "At goal" << std::endl;
            speed = 0;
            rotation_speed = 0;
            at_goal = true;
            robot_state = DONE;
        }
        else
        {
            speed = computeTranslation(sonars);
            if (robot_state == GOALSEEK)
            {
                rotation_speed = computeGoalSeek(goal_angle);
                if (obstacleInWay(goal_angle, sonars))
                    robot_state = WALLFOLLOW;
            }
            if (robot_state == WALLFOLLOW)
            {
                rotation_speed = computeRWFRot(sonars);
                if (!obstacleInWay(goal_angle, sonars))
                    robot_state = GOALSEEK;
            }
        }
    }
}
