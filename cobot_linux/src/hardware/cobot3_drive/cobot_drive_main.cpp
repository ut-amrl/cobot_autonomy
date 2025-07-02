#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <sys/time.h>
#include <filesystem> 
#include <deque>

#include "drive.h"              
#include "terminal_utils.h"     
#include "proghelp.h"           
#include "configreader.h"
#include "popt_pp.h"
#include "pthread_utils.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "cobot_msgs/msg/cobot_drive_msg.hpp"
#include "cobot_msgs/msg/cobot_odometry_msg.hpp"
#include "cobot_msgs/msg/cobot_joystick_msg.hpp"

bool run = true;
bool flipEStop = false;
CobotDrive* cobotDrive;
float min_angular_velocity_request = RAD(5.0);
float min_angular_velocity_command = 0.0;
float motionScale = 1.0;
static const float MaxJoystickTransSpeed = 1.5; // 1.5
static const float MaxJoystickAngSpeed = 1.0 * M_PI; // 1.0 * M_PI

uint64_t _joystickButtons;
std::deque<float> _joystickAxes;
double _lastJoystickRcvT;
pthread_mutex_t _joystickMutex;

std::string expandUser(const std::string& path) {
    if (path.rfind("~/", 0) == 0) {
        const char* home = getenv("HOME");
        if (home) {
            return std::string(home) + path.substr(1);  // replace '~' with $HOME
        }
    }
    return path;
}

void LoadParameters() {
    WatchFiles watch_files;
    std::string relative_root = "~/cobot_autonomy/cobot_linux"; // TODO fix the hardcoded path
    std::string cfg_root = std::filesystem::absolute(expandUser(relative_root));
    cfg_root += "/";
    ConfigReader config(cfg_root.c_str());

    config.init(watch_files);
    config.addFile("../robot.cfg");
    config.addFile("config/drive.cfg");

    if(!config.readFiles()){
        printf("Failed to read config\n");
        exit(1);
    }

    {
        ConfigReader::SubTree c(config,"RobotConfig");
        bool error = false;
        error = error || !c.getBool("emergencyStopReversed",flipEStop);
        if(error){
        printf("Error Loading Debug Parameters!\n");
        exit(2);
        }
    }
    {
        ConfigReader::SubTree c(config,"Drive");
        bool error = false;
        error = error || !c.getReal("min_angular_velocity_request",
                                    min_angular_velocity_request);
        error = error || !c.getReal("min_angular_velocity_command",
                                    min_angular_velocity_command);
        if(error){
        printf("Error Loading Debug Parameters!\n");
        exit(2);
        }
    }
}

float joystickApplyDeadZone(float val)
{
    static const float DeadZone = 0.50;
    float x = val;
    if (fabs(x) < DeadZone)
        x = 0.0;
    else
        x = (x - DeadZone * sign(x)) / (1.0 - DeadZone);
    return x;
}

void joystickCallback(const std::shared_ptr<cobot_msgs::msg::CobotJoystickMsg> msg)
{
    static const bool debug = false;
    ScopedLock lock(_joystickMutex);
    _joystickButtons = msg->buttons;
    if(debug) printf("Joystick Rcv: 0x%X",(unsigned int)_joystickButtons);
    if(_joystickAxes.size()<msg->axes.size())
        _joystickAxes.resize(msg->axes.size());
    for(unsigned int i=0; i<msg->axes.size(); i++){
        _joystickAxes[i] = msg->axes[i];
        if(debug) printf(" %.2f",_joystickAxes[i]);
    }
    if(debug) printf("\n");
    _lastJoystickRcvT = GetTimeSec();
}

double getJoystickState(uint64_t& buttons, std::deque< float >& axes)
{
    ScopedLock lock(_joystickMutex);
    const double timestamp = _lastJoystickRcvT;
    buttons = _joystickButtons;
    axes = _joystickAxes;
    return timestamp;
}

void getSpaceMouseCommand(vector2f *transDesired,
                          float *rotDesired)
{
    static const double kJoystickTimeout = 0.25;
    static const bool debug = false;
    
    std::deque<float> axes;
    uint64_t buttons;
    const double timestamp = getJoystickState(buttons, axes);
    if (GetTimeSec() - timestamp > kJoystickTimeout)
    {
        transDesired->zero();
        *rotDesired = 0.0;
        return;
    }

    float x = axes[0];
    float y = axes[1];
    float r = -joystickApplyDeadZone(axes[2]);
    transDesired->set(x, y);
    *transDesired = transDesired->bound(1.0);
    *rotDesired = abs_bound(r, 1.0);

    if (debug)
    {
        printf("SpaceMouse: %.2f %.2f %.2f\n",
            transDesired->x, transDesired->y, axes[0]);
    }
}


void timerEvent(int sig) {
    const bool debug = false;
    static double tLast = GetTimeSec();
    static double tLastOdometry = GetTimeSec();
    if(debug){
        printf( "dT = %f\n", GetTimeSec()-tLast );
        tLast = GetTimeSec();
    }
    // cobotDrive->setSpeeds(0.5, 0.0, 0.0); // Example speeds, replace with actual logic
    vector2f desiredTransVel;
    float desiredRotVel;
    getSpaceMouseCommand(&desiredTransVel, &desiredRotVel);
    desiredTransVel = motionScale * MaxJoystickTransSpeed * desiredTransVel;
    desiredRotVel = motionScale * MaxJoystickAngSpeed * desiredRotVel;

    if (debug) {
        std::cout << "Desired Translation Velocity: " 
              << desiredTransVel.x << ", " 
              << desiredTransVel.y << std::endl;
        std::cout << "Desired Rotation Velocity: " 
                << desiredRotVel << std::endl;
    }

    float ang_vel_cmd = desiredRotVel;
    if (fabs(ang_vel_cmd) > min_angular_velocity_request &&
        fabs(ang_vel_cmd) < min_angular_velocity_command) {
        ang_vel_cmd = sign<float>(ang_vel_cmd) * min_angular_velocity_command;
    }
    cobotDrive->setSpeeds(desiredTransVel.x, desiredTransVel.y, ang_vel_cmd);
    cobotDrive->run();

    if( !run ){
        CancelTimerInterrupts();
    }
}

int main(int argc, char **argv) {
    CHECK_EQ(pthread_mutex_init(&_joystickMutex, NULL), 0);

    static const bool debug = true;
    int portNum = 0;
    // option table
    static struct poptOption options[] = {
        { "port-num", 'p', POPT_ARG_INT , &portNum,  0,
            "Laser Scanner serial port number", "NUM"},
        POPT_AUTOHELP
        { NULL, 0, 0, NULL, 0, NULL, NULL }
    };
    // parse options
    POpt popt(NULL,argc,(const char**)argv,options,0);
    int c;
    while((c = popt.getNextOpt()) >= 0){
    }

    LoadParameters();
    motor_properties_t motorProps;
    motorProps.xyFlipped = true;
    motorProps.encoderCountsPerMeter = vector2d(-1.0,1.0) * 30466.8;
    motorProps.encoderCountsPerRadian = 8701.6;
    motorProps.transMotionScale = vector2d(-1.0,1.0) * 17.9511592;
    motorProps.rotMotionScale = -3.88148392;
    cobotDrive = new CobotDrive(motorProps);

    char serialPort[256];
    sprintf(serialPort,"/dev/ttyUSB%d",portNum);
    if(debug) printf("Using port %s\n",serialPort);

    InitHandleStop(&run);
    AccelLimits transLimits, rotLimits;
    transLimits.set(1.0,2.0,2.5);
    rotLimits.set(1.0*M_PI,1.0*M_PI,1.5*M_PI);
    cobotDrive->setLimits(transLimits, rotLimits);
    cobotDrive->init(serialPort);
    //Interrupt frequency of 20 Hz
    if(!SetTimerInterrupt(50000, &timerEvent)){
        TerminalWarning( "Unable to set timer interrupt\n" );
        cobotDrive->close();
        return(1);
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Cobot3_Drive_Module");

    auto joystick_sub = node->create_subscription<cobot_msgs::msg::CobotJoystickMsg>(
        "/Cobot/Joystick", 10, joystickCallback);

    rclcpp::Rate rate(100);  // 100 Hz = 0.01s
    while (rclcpp::ok() && run) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    printf("closing.\n");
    cobotDrive->close();
    rclcpp::shutdown();
}