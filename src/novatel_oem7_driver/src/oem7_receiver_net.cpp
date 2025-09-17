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

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace novatel_oem7_driver
{
  /**
   * Functionality for a network connection using sys/socket
   * This was designed under the assumption that the receiver is operating in a closed network and was not designed with security in mind.
   */
  class Oem7ReceiverNet: public Oem7Receiver
  {
  protected:

    // Stored connection parameters
    std::string recvr_ip_addr_;
    in_addr recvr_ip_addr_encoded_{};
    int recvr_port_;

    std::string port_name_;
    int port_baud_;
    speed_t port_baud_encoded_;

    /**
     * Gets parameters for a socket connection
     */
    bool initialize(rclcpp::Node& node) 
    {
      // Receiver IP address
      DriverParameter<std::string> oem7_ip_addr("oem7_ip_addr", "", node);
      recvr_ip_addr_ = oem7_ip_addr.value();
      ::inet_aton(recvr_ip_addr_.c_str(), &recvr_ip_addr_encoded_);

      // Port
      DriverParameter<int> oem7_port("oem7_port", 0, node);
      recvr_port_ = oem7_port.value();

      return Oem7Receiver::initialize(node);     
    }

    /**
     * Creates a connection with the socket
     */
    virtual void endpoint_open() = 0;

    /**
     * Controls reading from the endpoint. Virtual as implementation is specific to TCP/UDP
     */
    virtual ssize_t endpoint_read(unsigned char* data, unsigned int data_size, std::string& err) = 0;

    /**
     * Controls writing to the endpoint. Virtual as implementation is specific to TCP/UDP
     */    
     virtual ssize_t endpoint_write(const unsigned char* data, const unsigned int data_len, std::string& err) = 0;
};

  /**
   * Specific functionality for a TCP connection using sys/socket
   */
  class Oem7ReceiverTcp: public Oem7ReceiverNet
  {
    /**
     * Establishes a connection
     */
    void endpoint_open()
    {
      // IPv4 connection
      endpoint_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);

      sockaddr_in my_addr{};
      my_addr.sin_family = AF_INET; //IPv4 connection
      my_addr.sin_port = htons(this->recvr_port_); // Port to connect to
      my_addr.sin_addr = this->recvr_ip_addr_encoded_; // IP address to connect to

      RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Oem7Receiver: TCP Connection Opening: " <<
                      "['" << this->recvr_ip_addr_ << "' : " << std::to_string(this->recvr_port_) << "]");

      // Non blocking connect call
      ::connect(this->endpoint_, reinterpret_cast<sockaddr *>(&my_addr), sizeof(my_addr));
    }

    /**
     * Reads from a TCP endpoint
     */
    ssize_t endpoint_read(unsigned char* data, unsigned int data_size, std::string& err)
    {      
      errno = 0;
      ssize_t bytes_read = ::recv(
          this->endpoint_,
          reinterpret_cast<char*>(data),
          data_size,
          MSG_DONTWAIT | MSG_NOSIGNAL
      );
      err = std::string(::strerror(errno));
      return bytes_read;      
    }

    /**
     * Writes to a TCP endpoint
     */
    ssize_t endpoint_write(const unsigned char* data, const unsigned int data_len, std::string& err)
    {
      errno = 0;
      ssize_t bytes_sent = send(
        this->endpoint_,
        reinterpret_cast<const char*>(data),
        data_len,
        MSG_DONTWAIT | MSG_NOSIGNAL
      );
      err = std::string(::strerror(errno));
      return bytes_sent;
    }; 
  };

  /**
   * Specific functionality for a UDP connection using sys/socket
   */
  class Oem7ReceiverUdp: public Oem7ReceiverNet
  {
    sockaddr_in recvr_addr{};

    /**
     * Establishes a connection
     */
    void endpoint_open()
    {
      endpoint_ = socket(PF_INET, SOCK_DGRAM, 0);

      sockaddr_in my_addr{};
      my_addr.sin_family = AF_INET; //IPv4 connection
      my_addr.sin_port = htons(this->recvr_port_); // Port to connect to

      my_addr.sin_addr.s_addr = INADDR_ANY;
      
      RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Oem7Receiver: UDP Connection Opening: " <<
                      "['" << this->recvr_ip_addr_ << "' : " << std::to_string(this->recvr_port_) << "]");

      // Non blocking connect call
      if (::bind(this->endpoint_, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) == -1) {
        std::string err = std::string(::strerror(errno));
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                    "Oem7Receiver: Connection Failed to Open. Error= " << err);
      }

      // Sets the receiver connection information to send data to
      recvr_addr.sin_family = AF_INET;
      recvr_addr.sin_port = htons(this->recvr_port_);
      recvr_addr.sin_addr = this->recvr_ip_addr_encoded_;
    }

    /**
     * Reads from a connection
     */
    ssize_t endpoint_read(unsigned char* data, unsigned int data_size, std::string& err)
    {
      sockaddr_in sender_address{};
      socklen_t sender_address_len = sizeof(sender_address);

      ssize_t bytes_read = recvfrom(
          this->endpoint_,
          reinterpret_cast<char*>(data),
          data_size,
          MSG_DONTWAIT | MSG_NOSIGNAL, 
          reinterpret_cast<sockaddr *>(&sender_address),
      &sender_address_len);

      err = std::string(::strerror(errno));
       
      // Filters by the receiver's ip address
      if (sender_address.sin_addr.s_addr != this->recvr_ip_addr_encoded_.s_addr)
      {      
        return 0;
      } 

      return bytes_read; 
    }

    ssize_t endpoint_write(const unsigned char* data, const unsigned int data_len, std::string& err) 
    {
      ssize_t bytes_sent = ::sendto(
        this->endpoint_,
        reinterpret_cast<const char*>(data),
        data_len,
        MSG_DONTWAIT | MSG_NOSIGNAL,
        reinterpret_cast<struct sockaddr*>(&recvr_addr),
        sizeof(recvr_addr)
      );
      err = std::string(::strerror(errno));
      return bytes_sent;
    };
  };
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverTcp,     novatel_oem7_driver::Oem7ReceiverIf)
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverUdp,     novatel_oem7_driver::Oem7ReceiverIf)
