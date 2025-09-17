////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#include <driver_parameter.hpp>
#include <novatel_oem7_driver/oem7_receiver_if.hpp>
#include <oem7_receiver.hpp>

#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>


namespace novatel_oem7_driver
{
  /**
   * Serial port implementation.
   */
  class Oem7ReceiverPort: public Oem7Receiver
  {
  private:
    speed_t getBaudRate() {
        switch (port_baud_) { //Uses all configurable OEM7 Baud Rates
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            default: return B0; // B0 is used to indicate an unsupported baud rate
        }
    }


  protected:
    std::string port_name_;
    int port_baud_;
    speed_t port_baud_encoded_;

    /**
     * Gets parameters for a serial connection
     */
    bool initialize(rclcpp::Node& node)
    {
      DriverParameter<std::string> oem7_port_name("oem7_port_name", "", node);
      port_name_ = oem7_port_name.value();

      DriverParameter<int> oem7_port_baud("oem7_port_baud", 0, node);
      port_baud_ = oem7_port_baud.value();
      port_baud_encoded_ = getBaudRate();

      if (port_baud_encoded_ == B0){
        Oem7Receiver::initialize(node);
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Oem7Receiver: Baud rate not supported");
        return false;
      }

      return Oem7Receiver::initialize(node);     
    }

    /**
    * Creates a connection with the serial port
    */
    void endpoint_open()
    {
      RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Oem7Receiver: Port Connection Opening: " <<
                      "['" << port_name_ << "' , " << std::to_string(port_baud_) << "]");

      endpoint_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);

      struct termios tty;
      memset(&tty, 0, sizeof tty);  // Clear the structure

      // Gets current port settings
      if (tcgetattr(endpoint_, &tty) != 0) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Oem7Receiver: Failed To Get Port Settings");
      }

      // Sets the terminal to 'raw' settings
      cfmakeraw(&tty);

      // Sets a user baud specified rate
      if (port_baud_encoded_ != B0) {
        cfsetospeed(&tty, port_baud_encoded_);
        cfsetispeed(&tty, port_baud_encoded_);
      }

      if (tcsetattr(endpoint_, TCSANOW, &tty) != 0) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Oem7Receiver: Failed To Set Port Settings");
      }
    }

    /**
    * Reads from a connection
    */
    ssize_t endpoint_read(unsigned char* data, unsigned int data_size, std::string& err)
    {
      ssize_t bytes_read = ::read(
          endpoint_,
          reinterpret_cast<char*>(data),
          data_size        
      );
      err = std::string(::strerror(errno));
      return bytes_read;      
    } 

    /**
    * Writes to a connection
    */
    ssize_t endpoint_write(const unsigned char* data, const unsigned int data_len, std::string& err)
    {
      ssize_t bytes_sent = ::write(
        endpoint_,
        reinterpret_cast<const char*>(data),
        data_len
      );
      err = std::string(::strerror(errno));
      return bytes_sent;
    }

    /**
    * Closes and re-opens the endpoint
    */
    void endpoint_reconnect() 
    {
      this->endpoint_open_ = false;
      ::close(this->endpoint_);
      endpoint_open();
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverPort,     novatel_oem7_driver::Oem7ReceiverIf)

