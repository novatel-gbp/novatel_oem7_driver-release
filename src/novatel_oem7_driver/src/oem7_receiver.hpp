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

#include <novatel_oem7_driver/oem7_receiver_if.hpp>

#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

namespace novatel_oem7_driver
{
  /**
    * Common functionality related to reading/writing from a realtime connection
    */
  class Oem7Receiver: public Oem7ReceiverIf
  {

    enum
    {
      DEFAULT_MAX_NUM_IO_ERRORS = 7
    };

  protected:

    static const int POLL_TIMEOUT = 1000;  // one second (in msec)

    int endpoint_;

    bool endpoint_open_;

    rclcpp::Node* node_;

    int max_num_io_errors_; ///< Number of consecutive io errors before declaring failure and quitting.
    int num_io_errors_; ///< Number of consecuitive io errors.

    /**
     * Check if there's a connection with the endpoint
     */
    virtual bool is_endpoint_open() {
      return endpoint_open_;
    }

    /**
     * Creates a connection with the endpoint
     */
    virtual void endpoint_open() = 0;

    /**
     * Read some data from the endpoint.
     */
    virtual ssize_t endpoint_read(unsigned char* data, unsigned int data_size, std::string& err) = 0;

    /**
     * Write some data to the endpoint
     */
    virtual ssize_t endpoint_write(const unsigned char* data, const unsigned int data_len, std::string& err) = 0;

    /**
    * Used to reconnect to the endpoint if a connection fails. May include no action or things like closing and reopening.
    */
    virtual void endpoint_reconnect() {}

    /**
     * Closes the endpoint; delay to avoid tight re-open loop
     */
    virtual void close()
    {
      ::close(endpoint_);
      endpoint_open_ = false;
      RCLCPP_WARN_STREAM(node_->get_logger(), "Oem7Receiver: Closed");
      rclcpp::Rate rate(1);  // 1 Hz = 1s pause
      rate.sleep();
    }

    /**
     * Controls opening the endpoint. Will not return until the endpoint is opened or ROS shuts down
     */
    void endpoint_try_open()
    {
      // Checks if the driver is already open
      if (endpoint_open_ || !rclcpp::ok()){
        return;
      }

      rclcpp::Rate rate(4);  // 4 Hz = 250ms pause

      // Connects to the endpoint
      endpoint_open_ = false;
      endpoint_open();

      // Sends a primer byte to the receiver every 250ms until one is successfully sent to know once connected.
      while (rclcpp::ok() && !endpoint_open_){

        static const std::string CONN_PRIMER("\r\n");

        static const unsigned char* PRIMER_CHAR = reinterpret_cast<const unsigned char*>(CONN_PRIMER.c_str());
        static unsigned int PRIMER_LEN = static_cast<unsigned int>(CONN_PRIMER.length());
        
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Oem7Receiver: Sending Primer Bytes");
        
        std::string err;
        ssize_t bytes_sent = endpoint_write(PRIMER_CHAR, PRIMER_LEN, err); 

        if (bytes_sent == 2) {
          RCLCPP_INFO_STREAM(node_->get_logger(),
            "Oem7Receiver: Connected to Receiver");   
          endpoint_open_ = true;
          num_io_errors_ = 0;
          break;
        };

        // Even in a successful connection, it is expected that writing to the receiver may fail for a couple of initial attempts
        // inappropriate ioctl for device err -> Likely indicates incorrect connection parameters
        num_io_errors_++;
        if (in_error_state()) {
          RCLCPP_WARN_STREAM(node_->get_logger(), "Oem7Receiver: Unsuccessful Attempt to Send Primer Bytes. Trying Again. Error:  " <<  err);
        }

        rate.sleep();

        // Reconnects to endpoint      
        endpoint_reconnect();
      }
    };

    /**
     * Monitors the number of errors that have occured
     */
    bool in_error_state()
    {
      if(num_io_errors_ >= max_num_io_errors_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Oem7Receiver: Max Num IO errors exceeded: " << max_num_io_errors_);
        return true;
      }
      else
      {
        return false;
      }
    }

    /**
     * Controls reading of data off of endpoint. Blocks until data is received, ROS shuts down or a read error
     */
    bool read_controller(unsigned char* data, unsigned int data_len, unsigned int& rlen, std::string& err) 
    {
      rlen = 0;
      // pollfd is to monitor endpoint_
      struct pollfd fds[1];
      fds[0].fd = endpoint_;
      fds[0].events = POLLIN;

      // Continously tries to read data until possible or if ROS shuts down
      while (rclcpp::ok()) {
        //Stays in do while loop until poll confirms data is present, ROS shuts down or there is an error with polling
        do {
          //Waits until data is polled or the poll times out
          RCLCPP_DEBUG_STREAM(node_->get_logger(), "Oem7Receiver: Polling");

          int retval = poll(fds, 1, this->POLL_TIMEOUT); 

          // Stores the error variable
          err = std::strerror(errno);

          if (retval < 0) {
            return false;
          } 

          if ((fds[0].revents & POLLERR) ||
            (fds[0].revents & POLLHUP) ||
            (fds[0].revents & POLLNVAL))
          {
            return false;
          }

        } while ((fds[0].revents & POLLIN) == 0 && rclcpp::ok());
      
        // After successful poll read from the connection
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Oem7Receiver: Poll Success. Reading From Receiver");

        ssize_t nbytes = endpoint_read(data, data_len, err);

        // If it fails, return
        if (nbytes < 0) {
          return false;
        }

        // If successful, return
        if (nbytes > 0) {
          rlen = static_cast<unsigned int>(nbytes);
          err.clear();
          return true;
        }

        // Else if no data was read (nbytes=0), try again
      }
      return false;
    };
    

  public:
    Oem7Receiver():
      max_num_io_errors_(DEFAULT_MAX_NUM_IO_ERRORS),
      num_io_errors_(0),
      endpoint_open_(false)
    {
    }

    virtual ~Oem7Receiver()
    {
    };

    /**
     * Initializes the node and parameters
     */
    virtual bool initialize(rclcpp::Node& node)
    {
      node_ = &node;

      node_->declare_parameter("oem7_max_io_errors", 0);
      max_num_io_errors_ = node_->get_parameter("oem7_max_io_errors").as_int();

      return true;
    }

    /**
     * Controls reading from the endpoint. If it fails retries continously until it fails too many times or the driver is shutdown
     */
    virtual bool read(unsigned char* data, unsigned int data_len, unsigned int& rlen)
    {
      while(rclcpp::ok() && !in_error_state())
      {
        endpoint_try_open(); // Makes sure we're connected

        std::string err;
        bool success = read_controller(data, data_len, rlen, err);

        if(success)
        {
          num_io_errors_ = 0; // Reset error counter

          return true;
        }
        // else: error condition

        num_io_errors_++;

        RCLCPP_ERROR_STREAM(node_->get_logger(),
              "Oem7Receiver: read error: " <<  err
                                           <<"; endpoint open: " << is_endpoint_open()
                                           <<" errors/max: " << num_io_errors_
                                           <<"/"             << max_num_io_errors_);
                                           
        close();
      }

      return false;
    }

    /**
     * Controls writing to the endpoint.
     */
    virtual bool write(const unsigned char* data, const unsigned int data_len)
    {
      rclcpp::Rate rate(4);  // 4 Hz = 250ms pause
      // Waits until the endpoint is known to be open or the endpoint closes
      while (rclcpp::ok() && !endpoint_open_ && !in_error_state()) {
        rate.sleep();      
      }

      if(in_error_state() || !rclcpp::ok())
        return false;

      std::string err;
      ssize_t bytes_sent = endpoint_write(data, data_len, err);

      if (bytes_sent == static_cast<ssize_t>(data_len)) {
        err.clear();
        return true;
      }

      num_io_errors_++;

      RCLCPP_ERROR_STREAM(node_->get_logger(),
            "Oem7Receiver: write error: " <<  err
                                          <<"; endpoint open: " << is_endpoint_open()
                                          <<" errors/max: " << num_io_errors_
                                          <<"/"             << max_num_io_errors_);
      
      close();
      return false;
    }
  };
}


