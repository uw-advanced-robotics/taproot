#ifndef ROBOT_INFO
#define ROBOT_INFO

enum class RobotType { Hero, Engineer, Soldier, Drone, Sentry};

class RobotInfo {
    public:
    static RobotType getRobotType();
};

#endif