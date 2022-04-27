/*******************************************************************/
/*                      Author: qxh                                */
/*                     Contact: qxh880507@163.com                  */
/*                 Last update: 2020-05-25                         */
/*******************************************************************/

/*
  Description: Serial communication.
  Regards to : https://blog.csdn.net/caijiwyj/article/details/90314312
*/



#include <fcntl.h>                           // To use: open().
#include <unistd.h>                          // To use: read(), write().
#include <string.h>                          // To use: to_string(), memset(), string.
#include <iostream>                          // To use: cout.
#include <termios.h>                         // To use: struct termios, tcgetattr(), tcsetattr().


#include "qxh_com.h"


namespace qxh
{

QxhCom::QxhCom() : fd_(0), buf_len_(1024)
{
  buf_ = new char[buf_len_];
  memset(buf_, 0, buf_len_);
}


QxhCom::QxhCom(int _buf_len) : fd_(0), buf_len_(_buf_len)
{
  buf_ = new char[buf_len_];
  memset(buf_, 0, buf_len_);
}


QxhCom::~QxhCom(){ delete [] buf_; }


int QxhCom::OpenCom(int _port, int _flag)
{
  fd_ = open(("/dev/ttyUSB" + std::to_string(_port)).c_str(), _flag);  // Note that "xxx" will be automatically transformed into std:string.
  
  if(fd_ < 0)
    PrintErrInfo("Com port " + std::to_string(_port) + " open failed!");
  else
    std::cout << "Com port " + std::to_string(_port) + " is opened!\n";
  
  return (fd_ > 0) ? 0 : fd_;
}


int QxhCom::SetCom(int _parity, int _baud_rate, int _stop_bit_len)
{
  termios tmp_opt;                       // System struct containing configs.
  memset(&tmp_opt, 0, sizeof(tmp_opt));
  
  if(tcgetattr(fd_, &tmp_opt) != 0)      // Get current device config.
  {
    PrintErrInfo("Com set: get com config file failed!");
    return -1;
  }
  
  tmp_opt.c_cflag |= (CLOCAL | CREAD);  // Activate local data link. Start serial data reception. 
  tmp_opt.c_cflag &= ~CSIZE;            // Set data len as 8 bits. This line has to be added, so as to set CS8!
  tmp_opt.c_cflag |= CS8;               // Set data len as 8 bits.
  
  switch(_parity)                       // Set parity.
  {
    case 0:                             // No parity.
      tmp_opt.c_cflag &= ~PARENB;
      break;
    case 1:                             // Odd parity.
      tmp_opt.c_cflag |= PARENB;
      tmp_opt.c_cflag |= PARODD;
      tmp_opt.c_iflag |= (INPCK | ISTRIP);
      break;
    case 2:                             // Even parity.
      tmp_opt.c_iflag |= (INPCK | ISTRIP);
      tmp_opt.c_cflag |= PARENB;
      tmp_opt.c_cflag |= ~PARODD;
      break;
    default:                            // Default: no parity.
      _parity = 0;
      tmp_opt.c_cflag &= ~PARENB;
      PrintWarnInfo("Config variable 'parity' not valid. Use default value instead!");
  }
  
  unsigned int tmp_baud_rate = 0;       // Set baud rate.
  switch(_baud_rate)
  {
    case 9600:
      tmp_baud_rate = B9600;
      break;
    case 115200:
      tmp_baud_rate = B115200;
      break;
    case 230400:
      tmp_baud_rate = B230400;
      break;
    default:
      _baud_rate = 9600;
      tmp_baud_rate = B9600;
      PrintWarnInfo("Config variable 'baud rate' not valid. Use default value instead!");
  }
  cfsetispeed(&tmp_opt, tmp_baud_rate);  // Set input baud rate.
  cfsetospeed(&tmp_opt, tmp_baud_rate);  // Set output baud rate.
  
  switch(_stop_bit_len)                  // Set stop bit.
  {
    case 1:                              // Stop bit len = 1.
      tmp_opt.c_cflag &= ~CSTOPB;
      break;
    case 2:                              // Stop bit len = 2.
      tmp_opt.c_cflag |= CSTOPB;
      break;
    default:                             // Stop bit len = 1.
      _stop_bit_len = 1;
      tmp_opt.c_cflag &= ~CSTOPB;
      PrintWarnInfo("Config variable 'stop bit len' not valid. Use default value instead!");
  }
  
  tmp_opt.c_cc[VTIME] = 0;               // Default.
  tmp_opt.c_cc[VMIN]  = 0;               // Default.
  tcflush(fd_, TCIFLUSH);                // Clear previous data.
  
  if(tcsetattr(fd_, TCSANOW, &tmp_opt) != 0)      // Set device config. Effective now.
  {
    PrintErrInfo("Com set: set com config failed!");
    return -1;
  }

  std::cout << "Com port config: \n"
            << "   stop bit len: " << _stop_bit_len << "\n"
	    << "      baud rate: " << _baud_rate << "\n"
            << "         parity: " << _parity << "\n";
	    
  return 0;
}


int QxhCom::ReadStr(std::string& _data)
{
  int tmp_len = read(fd_, buf_, buf_len_);
  tmp_len = (tmp_len < 0) ? 0 : tmp_len;

  std::string tmp_str = buf_;
  _data.swap(tmp_str);
  memset(buf_, 0, tmp_len);
  
  return tmp_len;
}


}  // namespace qxh
