/*******************************************************************/
/*                      Author: qxh                                */
/*                     Contact: qxh880507@163.com                  */
/*                 Last update: 2021-05-25                         */
/*******************************************************************/

/*
  Description: Serial communication lib.
  Regards to : https://blog.csdn.net/caijiwyj/article/details/90314312
  
  Remember to use root run. Or add current username into dialout group in Ubuntu.
  gpasswd --add qxh dialout (To remove: gpasswd --del qxh dialout)
  Note that the group will be truely updated only after user re-login!
  Regards to : https://blog.csdn.net/u013992330/article/details/79102741
  
  Update: 20210605(qxh)
  1. Add func ReadStr(std::string& _data). Com data will be transformed into std::string.
  2. Change func ReadStr(std::string& _data) into ReadCom(char* _data, int _len = 1024).
     This func now reads and  copys com raw data into buffer _data, without any data type change.
*/

#include <vector>                            // To use: std::vector.
#include <fcntl.h>                           // To use: O_RDWR, O_NDELAY, O_NOCTTY, close().
#include <string.h>                          // To use: string.
#include <unistd.h>                          // To use: read(), write().
#include <iostream>                          // To use: std::cerr.



/* _<PACKAGE_NAME>_<FILE_NAME>_H_ */
#ifndef _QXH_COM_H_
#define _QXH_COM_H_ 


namespace qxh
{


class QxhCom
{
public:
  QxhCom();                                        // Use default buf_ len, i.e., len = 1024.
  QxhCom(int _buf_len);                            // Change the len of "buf_". Just in case U need it.
  ~QxhCom();

public:
  int OpenCom(int _port = 0, int _flag = O_RDWR | O_NOCTTY | O_NDELAY);
                                                   // Return 0: successful. Otherwise: failed.
                                                   // O_RDWR: r & w. O_RDONLY: r. O_WRONLY: w.
                                                   // O_NOCTTY: just a routine.
                                                   // O_NDELAY: nonblock. Or, you can use "O_NONBLOCK".
  
  int SetCom(int _parity, int _baud_rate, int _stop_bit_len); 
                                                   // Return 0: successful. Otherwise: failed.
                                                   // _parity: 0, no parity; 1, odd parity; 2, even parity.
                                                   // _stop_bit_len: 1, len = 1; 2, len = 2.

  int ReadStr(std::string& _data);
  
  inline int ReadCom(char* _data, int _len = 1024){ return read(fd_, _data, _len); }
  inline int WriteCom(std::string& _data){ return write(fd_, _data.c_str(), _data.length()); }
  inline int CloseCom(){ return close(fd_); }
  
private:
  int fd_;                                         // Com port ID.
  char* buf_;                                      // Store com msg for just a sec.
  
  int buf_len_;                                    // Len of "buf_".
  int parity_;                                     // 0: no parity. 1: odd parity. 2: even parity.
  int baud_rate_;                                  // 9600, 115200, 230400.
  int stop_bit_len_;                               // 1: 1 bit. 2: 2 bits.
  

  
private:
  inline void PrintWarnInfo(std::string _str){ std::cerr << "\033[33m" << "Com dev WARNING: " << _str << "\033[0m" << std::endl; }
  inline void PrintErrInfo (std::string _str){ std::cerr << "\033[31m" << "Com dev ERROR: "   << _str << "\033[0m" << std::endl; }


};


}  // namespace qxh





#endif // _QXH_COM_H_
