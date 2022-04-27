/*******************************************************************/
/*                      Author: qxh                                */
/*                     Contact: qxh880507@163.com                  */
/*                 Last update: 2020-05-25                         */
/*******************************************************************/

/*
  Description: GNSS signal decode.
  Remember to authorize the com device.
*/


#include <vector>                               // To use: vector.
#include <iomanip>                              // To use: std::setprecision().
#include <iostream>                             // To use: cout.
#include <GeographicLib/UTMUPS.hpp>             // To use: UTMUPS::Forward().


#include "gnss_prep.h"
#include "my_typedef.h"
#include "qxh_com/qxh_com.h"
#include "config_io/config_io.h"
#include "msg_position/msg_position.h"



using hivelab_config_io::Config;
using std::stod;



namespace qxh
{

GnssPrep::GnssPrep() : kComPort     (Config::Get<int32>("COM", "kComPort")),
                       kParity      (Config::Get<int32>("COM", "kParity")),
                       kBaudRate    (Config::Get<int32>("COM", "kBaudRate")),
                       kStopBitLen  (Config::Get<int32>("COM", "kStopBitLen")),
                       kGnssDataType(Config::Get<std::string>("GNSS_TYPE", "kGnssDataType"))
{  
  t_cost_ = 0;
  is_t_cost_new_ = false;
  
  gpchc_data_.Clear();
  gnss_str_raw_.clear();
  gnss_str_trim_.clear();

  com_.OpenCom(kComPort);
  com_.SetCom (kParity, kBaudRate, kStopBitLen);
  
  pub_gnss_pos_ = nh_.advertise<msg_position::msg_position>("msg_gnss_prep",1);
  pub_gnss_pos_long_ = nh_.advertise<msg_position::msg_position_long>("msg_gnss_prep_long",1);
}


GnssPrep::~GnssPrep(){ com_.CloseCom(); }


int32 GnssPrep::GpchcVerify()
{
  size_t tmp_pos_begin = gnss_str_raw_.rfind("$GPCHC");    // Search for the last symbol "$GPCHC".
  size_t tmp_pos_end   = gnss_str_raw_.rfind("*");         // Search for the last symbol "*".
  
  if(tmp_pos_begin == std::string::npos || tmp_pos_end == std::string::npos || tmp_pos_begin > tmp_pos_end)
    return -1;

  gnss_str_trim_ = gnss_str_raw_.substr(tmp_pos_begin, tmp_pos_end + 2); // Copy one GNSS msg.
  
  if(tmp_pos_end + 3 < gnss_str_raw_.size())                             // Delete used info in gnss_str_raw_.
    gnss_str_raw_  = gnss_str_raw_.substr(tmp_pos_end + 3, gnss_str_raw_.size() - 1);
  else
    gnss_str_raw_.clear();
  
  return 0;
}


int32 GnssPrep::ParseGpchc()
{
  if(GpchcVerify())                                    // Data invalid.
    return -1;
  
  gpchc_data_.Clear();
 msg_gnss_pos_long_.header.stamp = ros::Time().fromSec(ros::Time::now().toSec());
  int32 tmp_begin = 0;
  int32 tmp_end   = 0;
  std::vector<std::string> tmp_buf;

  tmp_end = gnss_str_trim_.find(",", tmp_begin);       // Segment GNSS info in gnss_str_trim_ using divider ",".
  for(; tmp_end != std::string::npos; tmp_end = gnss_str_trim_.find(",", tmp_begin))
  {
    tmp_buf.emplace_back(gnss_str_trim_.substr(tmp_begin, tmp_end - 1));
    tmp_begin = tmp_end + 1;
  }

  gpchc_data_.gps_week  = (int)strtol(tmp_buf.at(1).c_str(), NULL, 0); // int32
  gpchc_data_.gps_time  = atof(tmp_buf.at(2).c_str());                 // float64
  
  gpchc_data_.heading   = atof(tmp_buf.at(3).c_str());
  gpchc_data_.pitch     = atof(tmp_buf.at(4).c_str());
  gpchc_data_.roll      = atof(tmp_buf.at(5).c_str());
  gpchc_data_.gyro_x    = atof(tmp_buf.at(6).c_str());
  gpchc_data_.gyro_y    = atof(tmp_buf.at(7).c_str());
  gpchc_data_.gyro_z    = atof(tmp_buf.at(8).c_str());
  gpchc_data_.acc_x     = atof(tmp_buf.at(9).c_str());
  gpchc_data_.acc_y     = atof(tmp_buf.at(10).c_str());
  gpchc_data_.acc_z     = atof(tmp_buf.at(11).c_str());
    
  gpchc_data_.latitude  = atof(tmp_buf.at(12).c_str());
  gpchc_data_.longitude = atof(tmp_buf.at(13).c_str());
  gpchc_data_.altitude  = atof(tmp_buf.at(14).c_str());
  gpchc_data_.v_e       = atof(tmp_buf.at(15).c_str());
  gpchc_data_.v_n       = atof(tmp_buf.at(16).c_str());
  gpchc_data_.v_u       = atof(tmp_buf.at(17).c_str());
  gpchc_data_.v_sum     = atof(tmp_buf.at(18).c_str());
  
  gpchc_data_.nsv1      = (int)strtol(tmp_buf.at(19).c_str(), NULL, 0);
  gpchc_data_.nsv2      = (int)strtol(tmp_buf.at(20).c_str(), NULL, 0);
  gpchc_data_.status    = (int)strtol(tmp_buf.at(21).c_str(), NULL, 0);
  
  msg_gnss_pos_long_.gps_week = gpchc_data_.gps_week;
  msg_gnss_pos_long_.gps_time = gpchc_data_.gps_time;
  
  msg_gnss_pos_long_.latitude = gpchc_data_.latitude;
  msg_gnss_pos_long_.longitude = gpchc_data_.longitude;
  msg_gnss_pos_long_.altitude = gpchc_data_.altitude;
  msg_gnss_pos_long_.v_e = gpchc_data_.v_e;
  msg_gnss_pos_long_.v_n = gpchc_data_.v_n;
  msg_gnss_pos_long_.v_u = gpchc_data_.v_u;
  msg_gnss_pos_long_.acc_x = gpchc_data_.acc_x;
  msg_gnss_pos_long_.acc_y = gpchc_data_.acc_y;
  msg_gnss_pos_long_.acc_z = gpchc_data_.acc_z;
  
  msg_gnss_pos_long_.roll = gpchc_data_.roll;
  msg_gnss_pos_long_.pitch = gpchc_data_.pitch;
  msg_gnss_pos_long_.yaw = gpchc_data_.heading;
  msg_gnss_pos_long_.roll_rate = gpchc_data_.gyro_x;
  msg_gnss_pos_long_.pitch_rate = gpchc_data_.gyro_y;
  msg_gnss_pos_long_.yaw_rate = gpchc_data_.gyro_z;
  
  float64 tmp_x, tmp_y;                                                  // UTM transform.
  int32   tmp_zone;
  bool    tmp_northp;
  GeographicLib::UTMUPS::Forward(gpchc_data_.latitude, gpchc_data_.longitude, tmp_zone, tmp_northp, tmp_x, tmp_y);
  if (gpchc_data_.status == 42)
  {
     msg_gnss_pos_long_.status = "RTK_fixed";	
  }
  else
  {
     std::cout << "No RTK!!!\n";
  }
  
  msg_gnss_pos_long_.x = tmp_x;
  msg_gnss_pos_long_.y = tmp_y;
  msg_gnss_pos_long_.z = msg_gnss_pos_long_.altitude;
  
  return 0;
}

std::vector<std::string> GnssPrep::GetLines()
{
  std::vector<std::string> lines;
  while(true)
  {
    auto end = gnss_str_raw_.find('\n');
    if (end != std::string::npos)
    {
      auto msg = gnss_str_raw_.substr(0, end);
      lines.push_back(msg);
      gnss_str_raw_ = gnss_str_raw_.substr(end+1, gnss_str_raw_.size());
    }
    else
    {
      break;
    }
  }
  return lines;
}

int32 GnssPrep::ParseNovatel()
{
  auto lines = GetLines();

  for (const auto &line : lines)
  {
//     std::cout << line << std::endl;
      if (line.substr(0, 14) == "%CORRIMUDATASA")
      {
        ParseCorrimudatasa(line);
      }
      else if(line.substr(0, 9) == "#INSPVAXA")
      {
        ParseInspvaxa(line);
      }
  }

  return 0;
}

void GnssPrep::ParseInspvaxa(const std::string& sentence)
{
  msg_gnss_pos_long_.header.stamp = ros::Time().fromSec(ros::Time::now().toSec() - 0.0318);
  std::vector<std::string> head_tail = Split(sentence, ';');
  std::vector<std::string> data = Split(head_tail.back(), ',');
  msg_gnss_pos_long_.status = data[1];
  msg_gnss_pos_long_.latitude = stod(data[2]);
  msg_gnss_pos_long_.longitude = stod(data[3]);
  msg_gnss_pos_long_.z = stod(data[4]);
  msg_gnss_pos_long_.v_n = stod(data[6]);
  msg_gnss_pos_long_.v_e = stod(data[7]);
  msg_gnss_pos_long_.v_u = stod(data[8]);
  msg_gnss_pos_long_.roll = stod(data[9]);
  msg_gnss_pos_long_.pitch = stod(data[10]);
  auto temp_yaw = stod(data[11]);
  msg_gnss_pos_long_.yaw = std::fmod(450 - temp_yaw, 360); // convert to right-handed around east.

  float64 tmp_x, tmp_y;                                                  // UTM transform.
  int32   tmp_zone;
  bool    tmp_northp;
  GeographicLib::UTMUPS::Forward(msg_gnss_pos_long_.latitude, msg_gnss_pos_long_.longitude, tmp_zone, tmp_northp, tmp_x, tmp_y);
  
  msg_gnss_pos_long_.x = tmp_x;
  msg_gnss_pos_long_.y = tmp_y;
}

void GnssPrep::ParseCorrimudatasa(const std::string& sentence)
{
    std::vector<std::string> head_tail = Split(sentence, ';');
    std::vector<std::string> data = Split(head_tail.back(), ',');
    uint32_t logging_rate = 10;
    const double  radian_to_degree = M_PI / 180;
    msg_gnss_pos_long_.pitch_rate = stod(data[2]) * logging_rate * radian_to_degree;
    msg_gnss_pos_long_.roll_rate = stod(data[3]) * logging_rate * radian_to_degree;
    msg_gnss_pos_long_.yaw_rate = stod(data[4]) * logging_rate * radian_to_degree;
    msg_gnss_pos_long_.acc_x = stod(data[5]) * logging_rate;
    msg_gnss_pos_long_.acc_y= stod(data[6]) * logging_rate;
    msg_gnss_pos_long_.acc_z = stod(data[7]) * logging_rate;
}

int32 GnssPrep::ParseXWYD()
{
  auto lines = GetLines();
  if (lines.empty())
  {
    return -1;
  }

  for (auto const& line : lines)
  {
    auto data = Split(line, ',');
    if (line.substr(0,6) == "$GPFPD")
    {
      msg_gnss_pos_long_.header.stamp = ros::Time().fromSec(ros::Time::now().toSec());
      //msg_gnss_pos_long_.status = data[1];
      msg_gnss_pos_long_.latitude = stod(data[6]);
      msg_gnss_pos_long_.longitude = stod(data[7]);
      msg_gnss_pos_long_.z = stod(data[8]);
      msg_gnss_pos_long_.v_n = stod(data[10]);
      msg_gnss_pos_long_.v_e = stod(data[9]);
      msg_gnss_pos_long_.v_u = stod(data[11]);
      msg_gnss_pos_long_.roll = stod(data[5]);
      msg_gnss_pos_long_.pitch = stod(data[4]);
      auto temp_yaw = stod(data[3]);
      msg_gnss_pos_long_.yaw = std::fmod(450 - temp_yaw, 360); // convert to right-handed around east.

      float64 tmp_x, tmp_y;                                                  // UTM transform.
      int32   tmp_zone;
      bool    tmp_northp;
      GeographicLib::UTMUPS::Forward(msg_gnss_pos_long_.latitude, msg_gnss_pos_long_.longitude, tmp_zone, tmp_northp, tmp_x, tmp_y);
      
      msg_gnss_pos_long_.x = tmp_x;
      msg_gnss_pos_long_.y = tmp_y;
    }
    else if (line.substr(0, 6) == "$GTIMU")
    {
      msg_gnss_pos_long_.roll_rate = stod(data[3]);
      msg_gnss_pos_long_.pitch_rate = stod(data[4]);
      msg_gnss_pos_long_.yaw_rate = stod(data[5]);
      msg_gnss_pos_long_.acc_x = stod(data[6]);
      msg_gnss_pos_long_.acc_y = stod(data[7]);
      msg_gnss_pos_long_.acc_z = stod(data[8]);
    }
  }
  return 0;
}


std::vector<std::string> GnssPrep::Split(const std::string &sentence, const char delimiter)
{
    auto tokens = std::vector<std::string>();
    std::istringstream iss{sentence};
    std::string token;
    while (std::getline(iss, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}


void GnssPrep::PrintGnssInfo(int32 _start_line)
{
  std::cout << "\033[" << _start_line << ";0H";  // Move the curser to line _start_line.
  
  std::cout << std::setprecision(16);
  std::cout << "Longitude: " << msg_gnss_pos_long_.longitude << "        \n";
  std::cout << "Latitude:  " << msg_gnss_pos_long_.latitude << "        \n";
  std::cout << "Altitude:  " << msg_gnss_pos_long_.altitude << "        \n\n";
  
  std::cout << "pos x:     " << msg_gnss_pos_long_.x << "        \n";
  std::cout << "pos y:     " << msg_gnss_pos_long_.y << "        \n";
  std::cout << "pos z:     " << msg_gnss_pos_long_.z << "        \n";
  std::cout << std::setprecision(6);
}


void GnssPrep::PrintGnssInfo()
{
  std::cout << std::setprecision(16);
  std::cout << "Longitude: " << msg_gnss_pos_long_.longitude << "        \n";
  std::cout << "Latitude:  " << msg_gnss_pos_long_.latitude << "        \n";
  std::cout << "Altitude:  " << msg_gnss_pos_long_.altitude << "        \n\n";
  
  std::cout << "pos x:     " << msg_gnss_pos_long_.x << "        \n";
  std::cout << "pos y:     " << msg_gnss_pos_long_.y << "        \n";
  std::cout << "pos z:     " << msg_gnss_pos_long_.z << "        \n";
  std::cout << std::setprecision(6);
}


}  // namespace qxh
