################################################################################
# Copyright (c) 2020 NovAtel Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#################################################################################

"""
Message Hz analysis.

Existing ros_comm hz tests anaylyzes a single topic at a time;
This script analyzes multiple topics in a Bag.
"""

import os
import traceback

import numpy as np

import rosbag
from docutils.nodes import topic



def get_topic_list(bag_name):
    """ Return a list of topics stored in this bag """
    bag = rosbag.Bag(bag_name, 'r')
    return bag.get_type_and_topic_info()[1].keys()

def make_msg_gen(bag_name, topic):
    """ Generates a sequence of messages in the topic from the bag """
    bag = rosbag.Bag(bag_name, 'r')
    for top, msg, time in bag.read_messages():
        if top == topic:
            yield time, msg

    
def make_timestamp_gen(msg_itr):
    for t, m in msg_itr:
        yield t.to_sec(), m.header.stamp.to_sec()


     
    
def analyze_topic_hz(name, topic, exp_int, output_csv):
  """
  Analyzes message interval for a particular topic.
  """

  bag_name = name + ".bag"
  bag_path = os.path.expanduser("~") + "/.ros/" + bag_name

  msg_gen      = make_msg_gen(bag_path, topic)
  ts_gen       = make_timestamp_gen(msg_gen)
  
  # Array of timestamps: Bag ts (recording time), Message ts (generation time), Delta: rec ts - gen ts
  ts_arr_ = [[bag_ts, msg_ts, bag_ts - msg_ts] for bag_ts, msg_ts in ts_gen] # sigh...

  if len(ts_arr_) < 3:
      print("Topic: {}: Insufficient data: {} samples".format(topic, len(ts_arr_)))
      return False
  
  # Convert to numpy array
  ts_arr = np.array(ts_arr_)
      
  # Delta: time between message generated by the driver and bag recording (i.e. publishign overhead)
  d_arr = ts_arr[:, -1] # Allow rows in the Delta column
  d_mean = np.mean(d_arr) 
  d_max  = np.max( d_arr)  
  d_std  = np.std( d_arr)
  
  ROWS = 0 # Axis
  # Interval: time between individual message handling: generation by driver; bag recording.
  int_arr  = np.diff(ts_arr[:,:-1], axis = ROWS) # Intervals between Bag timestampts, publish timestamps,
  # i.e. message arrival interval: ['bag interval', 'publish interval']
  
  int_mean = np.mean(int_arr, axis = ROWS) 
  int_max  = np.amax(int_arr, axis = ROWS)
  int_std  = np.std( int_arr, axis = ROWS)
  
  BAG = 0
  MSG = 1  
  print("topic: '{}', exp interval= {}. samples= {}: ".format(topic, exp_int, len(ts_arr_)))
  print("                                             mean,                  max,               stdev")
  print("       Bag Recording interval:               {}, {}, {}".format(int_mean[BAG], int_max[BAG], int_std[BAG]))
  print("       Message publish interval:             {}, {}, {}".format(int_mean[MSG], int_max[MSG], int_std[MSG]))
  print("       Bag Rec - Message Pub Delta:          {}, {}, {}".format(d_mean,        d_max,        d_std))
  
  if output_csv:
      # Dump into .csv for additional post-processing.
      topic_filename = topic.replace("/", "_")
      np.savetxt(name + "." + topic_filename + ".csv", int_arr, delimiter=",")

  
"""
Topic configuration; refer to std_init_commands.yaml.
"""
NS = "/novatel/oem7/"
topic_config = \
    {
    # Topic  :  Rate
    NS + "bestpos":  0.1,
    NS + "bestvel":  0.1,
    NS + "bestutm":  1.0,
    NS + "bestgnsspos": 1.0,
    NS + "ppppos":   1.0,
    NS + "terrastarinfo":     1.0,
    NS + "terrastarstatus":   1.0,
    NS + "time":     1.0,
    NS + "corrimu":  0.01,
    NS + "inspva":   0.02,
    NS + "inspvax":  1.0, 
    NS + "insstdev": 1.0,
    "/gps/gps":      0.02, # Same rate as INSPVA 
    "/gps/fix":      0.02, # Same rate as gps 
    "/gps/imu":      0.01 # Same as corrimu
    }    

    
    
def analyze_hz(bag_name, output_csv):
    """
    Tests message frequency.
    Analyzes message intervals based on Message timestamp and Bag recording timestamp.
    """
    
    for topic in topic_config:
        analyze_topic_hz(bag_name, topic, topic_config[topic], output_csv)
        print ""
        


