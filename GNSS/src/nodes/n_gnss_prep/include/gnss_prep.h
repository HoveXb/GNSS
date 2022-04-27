/*******************************************************************/
/*                      Author: qxh                                */
/*                     Contact: qxh880507@163.com                  */
/*                 Last update: 2021-05-25                         */
/*******************************************************************/

/*
  Description: GNSS signal decode.
  
  Update: 20210605(qxh)
  1. Add func PubGnssMsg(), so as to print some key GNSS info in a floating pattern.
*/

#include <string.h>                          // To use: std::string, memset().
#include <ros/ros.h>                         // To use: init, NodeHandle, Publisher.
#include <algorithm>                         // To use: transform().


#include "my_typedef.h"
#include "qxh_com/qxh_com.h"
#include "msg_position/msg_position.h"
#include "msg_position/msg_position_long.h"



/* _<PACKAGE_NAME>_<FILE_NAME>_H_ */
#ifndef _GNSS_PREP_H_
#define _GNSS_PREP_H_ 


namespace qxh
{

struct GpchcData
{
  int32   gps_week;
  float64 gps_time;
  float64 heading;
  float64 pitch;
  float64 roll;
  float64 gyro_x;
  float64 gyro_y;
  float64 gyro_z;
  float64 acc_x;
  float64 acc_y;
  float64 acc_z;
  float64 latitude;
  float64 longitude;
  float64 altitude;
  float64 v_e;        // East.
  float64 v_n;        // North.
  float64 v_u;        // Up.
  float64 v_sum;
  int32   nsv1;       // Num of satellite on the main antenna.
  int32   nsv2;       // Num of satellite on the vice antenna.
  int32   status;
  
  inline void Clear(){ memset(&(this->gps_week), 0, sizeof(GpchcData)); }
};

}  // namespace qxh





namespace qxh
{


class GnssPrep
{
public:
  GnssPrep();
  ~GnssPrep();

public:
  int32 t_cost_;
  bool  is_t_cost_new_;
  
  
private:
  ros::NodeHandle nh_;
  ros::Publisher  pub_gnss_pos_;
  ros::Publisher  pub_gnss_pos_long_;
  
  const int32 kComPort;
  const int32 kParity;
  const int32 kBaudRate;
  const int32 kStopBitLen;
  
  const std::string kGnssDataType;
  
  QxhCom      com_;
  std::string gnss_str_raw_;
  std::string gnss_str_trim_;
  GpchcData   gpchc_data_;
  
  msg_position::msg_position_long msg_gnss_pos_long_;

  std::vector<std::string> Split(const std::string& sentence, const char delimiter);
  void ParseInspvaxa(const std::string& sentence);
  void ParseCorrimudatasa(const std::string& sentence);
  int32 ParseNovatel();
  int32 ParseXWYD();
  std::vector<std::string> GetLines();


  int32 ParseGpchc();
  int32 GpchcVerify();
  
public:
  void PrintGnssInfo(int32 _start_line);
  void PrintGnssInfo();
  
  
public:
  inline int32 ReadCom()
  {
    std::string tmp_str;
    int32 tmp_len = com_.ReadStr(tmp_str);
    gnss_str_raw_ += tmp_str;
    
    return tmp_len;
  }
  
  inline int32 ParseData()
  {
    std::string tmp_str_type = kGnssDataType;
    std::transform(tmp_str_type.begin(), tmp_str_type.end(), tmp_str_type.begin(), ::toupper);
    
    int32 tmp_return = -1;               // Default return. It means no valid data is parsed.
    if(tmp_str_type == "GPCHC")
      tmp_return = ParseGpchc();
    else if(tmp_str_type == "NOVATEL")
      tmp_return = ParseNovatel();
    else if(tmp_str_type == "XWYD")
      tmp_return = ParseXWYD();
    else                                 // Default opt.
      tmp_return = ParseGpchc();
    
    is_t_cost_new_ = !tmp_return;
    return tmp_return;
  }

  inline void PubGnssMsg(){ pub_gnss_pos_long_.publish(msg_gnss_pos_long_); }

};


}  // namespace qxh





#endif // _GNSS_PREP_H_
