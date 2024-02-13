#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

class TeleopRobot
{
public:
    TeleopRobot();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;

};

TeleopRobot::TeleopRobot():linear_(0),angular_(0),l_scale_(2.0),a_scale_(0.025)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void TeleopRobot::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' keys to move the robot forward and backward.");
  puts("Use 'AD' keys to turn0 the robot left and right.");
  puts("Press 'Q' to quit.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_= 0;
    angular_= 0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_U:
        linear_ = l_scale_;
        dirty = true;
        break;
      case KEYCODE_D:
        linear_ = -l_scale_;
        dirty = true;
        break;
      case KEYCODE_L:
        angular_ = a_scale_;
        dirty = true;
        break;
      case KEYCODE_R:
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_E:
        angular_ = -a_scale_;
        dirty = true;
        break;

      case KEYCODE_Q:
        return;
    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = angular_;
    twist.linear.x = linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}


int kfd = 0;   // kfd 는 keyboard file descriptor의 약자 참고 ㅎㅎ
struct termios cooked, raw;

void quit(int sig)
{
    // 이 quit 함수는 quit(SIGINT) 이런식으로 많이 사용하며 SIGINT 는 일반적으로 ctrl + C 에 의해서 발생함 즉 이함수는 ctrl + C 가 되었을 때 ROS 프로그램을 종료시키고 터미널 설정을 복원하는 기능을 하는 함수
  tcsetattr(kfd, TCSANOW, &cooked);     // 이함수는 kfd(키보드에 대한 세팅을 cooked 모드로 즉시 변경하는 기능)(프로그램이 종료될 때 원래의 터미널 설정으로 복원시키는 기능)
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot");
  TeleopRobot teleop_robot;

  signal(SIGINT, quit);

  teleop_robot.keyLoop();
  
  return(0);
}