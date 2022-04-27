
#include <string.h>                          // To use: std::string.
#include <iostream>                          // To use: cout, endl.
#include <ros/ros.h>                         // To use: init, NodeHandle, Publisher.


#include "gnss_prep.h"
#include "my_typedef.h"
#include "config_io/config_io.h"


using hivelab_config_io::Config;

int main(int argc, char** argv)
{
  std::cout << "\033c";                                    // Clear the screen.
  
  if(!Config::SetConfigPath("/media/hove/Backup/ros_workspace/src/GNSS/config_gnss_prep.yaml", ":"))
    return -1;
  
  ros::init (argc, argv, "n_gnss_prep");                   // This has to be the first command!
  qxh::GnssPrep gnss_prep;
  ros::Rate loop_rate(1000);


  int32  tmp_cnt = 0;
  uint32 tmp_cnt_print = 0;
  while(ros::ok())
  {
    ros::spinOnce();                                       // ROS routine.
    
    // double a = ros::Time::now().toSec();
    if(gnss_prep.ReadCom() > 0)                           // If data len is valid.
      if(!gnss_prep.ParseData())                           // If data parse is successful.
        gnss_prep.PubGnssMsg();
    // double b = ros::Time::now().toSec();
    // std::cout << "time: " << (b-a)*1000 << std::endl;
    
    if(tmp_cnt++ % 50 == 0)
    {
      std::cout << "\rProgram n_gnss_prep running: " << tmp_cnt_print++ << "        \n";
      if(gnss_prep.is_t_cost_new_)
      {
        gnss_prep.is_t_cost_new_ = false;
        gnss_prep.PrintGnssInfo();
        std::cout << "\033[5A";                            // Move curser backwards for 5 lines.
      }
      std::cout << "\033[1A";                              // Move curser backwards for 1 line.
    }

    loop_rate.sleep();
  }
  
  std::cout << "\033c" << "\nProgram n_gnss_prep terminated!" << std::endl;

  return 0;
}

