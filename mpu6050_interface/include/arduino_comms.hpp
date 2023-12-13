#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <cstring>
#include <sstream>
#include <cstdio>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  int read_imu_data(int16_t* qx, int16_t* qy, int16_t* qz, int16_t* qw, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write("i\r");
    try
    {
      for(int i = 0; i < 21; i++)
      {
        serial_conn_.ReadByte(imu_data[i], 1000);
      }
      status = 1;
    }
    catch(const LibSerial::ReadTimeout&)
    {
      *qx = 0;     
      *qy = 0;
      *qz = 0;
      *qw = 0;
      *ax = 0;
      *ay = 0;
      *az = 0;
      *gx = 0;
      *gy = 0;
      *gz = 0;
      status = -1;
      return status;
    }
    *qx = (int16_t&)imu_data;
    *qy = (int16_t&)imu_data[2];
    *qz = (int16_t&)imu_data[4];
    *qw = (int16_t&)imu_data[6];
    *ax = (int16_t&)imu_data[8];
    *ay = (int16_t&)imu_data[10];
    *az = (int16_t&)imu_data[12];
    *gx = (int16_t&)imu_data[14];
    *gy = (int16_t&)imu_data[16];
    *gz = (int16_t&)imu_data[18];
    return status;
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }

  void read_imu(double& qx, double& qy, double& qz, double& qw, int& ax, int& ay, int& az, int& gx, int& gy, int& gz)
  {
    std::string response = send_msg("i\r");
    std::string delimiter = " ";
    size_t start_pos = 0;

    for (int i = 0; i < 9; ++i) {
        size_t del_pos = response.find(delimiter, start_pos);

        // Assign each value directly to its respective variable
        switch (i) {
            case 0: 
              try {
                qx = std::stod(response.substr(start_pos, del_pos - start_pos).c_str());;
              } 
              catch (const std::invalid_argument&) 
              {
                std::cerr << "Argument is invalid\n";
                throw;
              } 
              break;
            case 1: qy = std::stod(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 2: qz = std::stod(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 3: qw = std::stod(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 4: ax = std::stoi(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 5: ay = std::stoi(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 6: az = std::stoi(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 7: gx = std::stoi(response.substr(start_pos, del_pos - start_pos).c_str()); break;
            case 8: gy = std::stoi(response.substr(start_pos, del_pos - start_pos).c_str()); break;
        }

        start_pos = del_pos + delimiter.length();
    }

    // Extract the last value
    gz = std::atoi(response.substr(start_pos).c_str());
    // const char delimitter[4] = " ";
    // char* token = strtok(response.c_str(), " ");
    // while( token != NULL ) 
    // {
    //   printf( " %s\n", token ); //printing each token
    //   token = strtok(NULL, " ");
    // }
  }

  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    uint8_t imu_data[21];
    int status = 0;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP