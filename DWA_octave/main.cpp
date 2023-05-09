#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <fstream>

using namespace std;

// Constants
const float ROBOT_RADIUS = 0.1;
const float MAX_SPEED = 1.0;
const float MAX_ANGULAR_SPEED = 1.0;
const float MAX_ACCELERATION = 1.0;
const float MAX_ANGULAR_ACCELERATION = 1.0;
const float DT = 0.1;
const float GOAL_THRESHOLD = 0.1;
const float OBSTACLE_THRESHOLD = ROBOT_RADIUS + 0.3;

// Data structures
struct Point
{
    float x;
    float y;
};

struct Obstacle
{
    float x;
    float y;
};

vector<Obstacle> obstacles;

struct RobotState
{
    Point position;
    float orientation;
    float linear_velocity;
    float angular_velocity;
};

// Helper functions
float distance(Point p1, Point p2)
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

float angle_difference(float a1, float a2)
{
    float diff = a2 - a1;
    if (diff > M_PI)
    {
        diff -= 2.0 * M_PI;
    }
    else if (diff < -M_PI)
    {
        diff += 2.0 * M_PI;
    }
    return diff;
}

float clip(float value, float min_value, float max_value)
{
    return min(max(value, min_value), max_value);
}

// DWA functions
vector<RobotState> generate_motion_samples(RobotState current_state)
{
    vector<RobotState> samples;

    for (float linear_velocity = -MAX_SPEED; linear_velocity <= MAX_SPEED; linear_velocity += MAX_ACCELERATION * DT)
    {
        for (float angular_velocity = -MAX_ANGULAR_SPEED; angular_velocity <= MAX_ANGULAR_SPEED; angular_velocity += MAX_ANGULAR_ACCELERATION * DT)
        {
            RobotState sample_state;
            sample_state.linear_velocity = clip(current_state.linear_velocity + linear_velocity * DT, -MAX_SPEED, MAX_SPEED);
            sample_state.angular_velocity = clip(current_state.angular_velocity + angular_velocity * DT, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
            sample_state.position.x = current_state.position.x + sample_state.linear_velocity * cos(current_state.orientation) * DT;
            sample_state.position.y = current_state.position.y + sample_state.linear_velocity * sin(current_state.orientation) * DT;
            sample_state.orientation = current_state.orientation + sample_state.angular_velocity * DT;

            samples.push_back(sample_state);
        }
    }

    return samples;
}

float evaluate_trajectory(RobotState start_state, RobotState end_state, Point goal_position)
{
    float distance_to_goal = distance(end_state.position, goal_position);
    float distance_penalty = 1.0 / (1.0 + distance_to_goal * 5);

    float obstacle_penalty = 1.0;
    for (int i = 0; i < obstacles.size(); i++)
    {
        Point obstacle;
        obstacle.x = obstacles.at(i).x;
        obstacle.y = obstacles.at(i).y;
        float obstacle_distance = distance(end_state.position, obstacle);
        if (obstacle_distance < OBSTACLE_THRESHOLD)
        {
            // std::cout << "obstacle close" << std::endl;
            obstacle_penalty *= (1000 * obstacle_distance / OBSTACLE_THRESHOLD);
        }
    }

    float orientation_penalty = 1 - abs(angle_difference(end_state.orientation, atan2(goal_position.y - end_state.position.y, goal_position.x - end_state.position.x))) / 4 * M_PI;

    return distance_penalty * obstacle_penalty * orientation_penalty;
}

RobotState find_best_trajectory(RobotState current_state, Point goal_position)
{
    float best_score = -INFINITY;
    RobotState best_state;
    vector<RobotState> samples = generate_motion_samples(current_state);

    for (int i = 0; i < samples.size(); i++)
    {
        float score = evaluate_trajectory(current_state, samples[i], goal_position);
        if (score > best_score)
        {
            best_score = score;
            best_state = samples[i];
        }
    }

    return best_state;
}

int main()
{
    // Read obstacles from file
    ifstream obstacle_file("obstacles.txt");
    if (obstacle_file.is_open())
    {
        while (!obstacle_file.eof())
        {
            Obstacle obstacle;
            obstacle_file >> obstacle.x >> obstacle.y;
            obstacles.push_back(obstacle);
        }
        obstacle_file.close();
    } // Set initial robot state
    RobotState current_state;
    current_state.position.x = 0.0;
    current_state.position.y = 0.0;
    current_state.orientation = 0.0;
    current_state.linear_velocity = 1.0;
    current_state.angular_velocity = 0.0;

    // Set goal position
    Point goal_position;
    goal_position.x = -7.0;
    goal_position.y = 8.0;

    ofstream file;
    file.open("positions.txt");
    // DWA loop
    while (distance(current_state.position, goal_position) > GOAL_THRESHOLD)
    {
        RobotState new_state = find_best_trajectory(current_state, goal_position);

        // Update robot state
        current_state = new_state;

        // Print current state
        cout << "Position: (" << current_state.position.x << ", " << current_state.position.y << ")" << endl;
        cout << "Orientation: " << current_state.orientation << endl;
        cout << "Linear velocity: " << current_state.linear_velocity << endl;
        cout << "Angular velocity: " << current_state.angular_velocity << endl;
        cout << endl;

        file << current_state.position.x << "\n"
             << current_state.position.y << "\n"
             << current_state.orientation << "\n";
    }
    file.close();
    cout << "Reached goal position!" << endl;
    system("./movement_simulation.sh");

    return 0;
}