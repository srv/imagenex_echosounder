#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "TimeoutSerial.h"
#include <numeric>      // std::accumulate

class imagenex_echosounder {
 public:
 
  imagenex_echosounder(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
      : nh_(nh), nhp_(nhp), seq_(0)  
    {

    // Get configuration from rosparam server
    getConfig();
    ROS_INFO("Range: %f", range);
	  ROS_INFO("Profunditat min: %f", profundidad_min);
	  ROS_INFO("gain: %i", gain);
	  ROS_INFO("pulse_length: %i", pulse_length);
	  ROS_INFO("delay: %i", delay);
	  ROS_INFO("data_points: %i", data_points);
    ROS_INFO("profile: %i", profile);
    ROS_INFO("timeoutSerial: %i", timeoutSerial);
    ROS_INFO("timerDuration: %f", timerDuration);
    ROS_INFO("absorption: %f", absorption);
    ROS_INFO("range percentage: %f", range_percentage);
    ROS_INFO("sample_vector_size: %i", sample_vector_size);

    //Publishers
    profile_range_raw_pub_ = nhp_.advertise<sensor_msgs::Range>("profile_range_raw", 1); 
    data_bytes_raw_pub_ = nhp_.advertise<sensor_msgs::Range>("data_bytes_raw", 1);

    serial.open(115200); 
	  serial.setTimeout(boost::posix_time::seconds(timeoutSerial)); 

    timer_ = nh_.createTimer(ros::Duration(timerDuration), 
                             &imagenex_echosounder::timerCallback, 
                             this);
  	}

 private:
  void timerCallback(const ros::TimerEvent&) 
  {      
    //profile_minimum_range
    if(profile_minimum_range < 0) profile_minimum_range = 0;
    else if (profile_minimum_range > 250) profile_minimum_range = 250; 

    //gain        
    if(gain > 40) gain = 40;
    else if(gain < 0) gain = 0; 

	  // pulse_length
    if (pulse_length >255) pulse_length = 255;
    else if (pulse_length == 253) pulse_length = 254; 
	  else if (pulse_length < 1) pulse_length = 1;

    //Set the buffer_rx bytes number
    if (data_points==25 & profile==1) unsigned char buffer_rx[265];
    else (data_points==50 & profile==1) unsigned char buffer_rx[513];

    unsigned char buffer_tx[27];
      
    // Set the values of the Switch message. The Switch message is the message from computer to Echosounder to pull a range message from the Echosounder. 
    buffer_tx[0] = 0xFE;		        
    buffer_tx[1] = 0x44;			    
    buffer_tx[2] = 0x11;	    		
    buffer_tx[3] = range;		
    buffer_tx[4] = 0;
    buffer_tx[5] = 0;
    buffer_tx[6] = 0x43;				
    buffer_tx[7] = 0;
    buffer_tx[8] = gain;			
    buffer_tx[9] = 0;
    buffer_tx[10] = absorption;					
    buffer_tx[11] = 0;
    buffer_tx[12] = 0;
    buffer_tx[13] = 0;
    buffer_tx[14] = pulse_length;			
    buffer_tx[15] = profile_minimum_range*10;//set the profile_minimum_range in dm
    buffer_tx[16] = 0;
    buffer_tx[17] = 0;
    buffer_tx[18] = 0;					
    buffer_tx[19] = data_points;		
    buffer_tx[20] = 0;
    buffer_tx[21] = 0;
    buffer_tx[22] = profile;					
    buffer_tx[23] = 0;
    buffer_tx[24] = delay/2;			
    buffer_tx[25] = 0;					
    buffer_tx[26] = 0xFD;				
    
    
    serial.write(reinterpret_cast<char*>(buffer_tx), sizeof(buffer_tx)); // write the Switch message in the serial port
    try {
      serial.read(reinterpret_cast<char*>(buffer_rx), sizeof(buffer_rx)); // read the data buffer received in the serial port
    } catch (...)
    {
    }

    profile_range_high_byte = float((buffer_rx[9] & 0x7E) >> 1);
    profile_range_low_byte = float((buffer_rx[9] & 0x01) << 7)|(buffer_rx[8] & 0x7F));
    profile_range = 0.01 *float((profile_range_high_byte << 8)|profile_range_low_byte);

    data_bytes_high_byte = float((buffer_rx[11] & 0x7E) >> 1);
    data_bytes_low_byte = float((buffer_rx[11] & 0x01) << 7)|(buffer_rx[10] & 0x7F));
    data_bytes = float((data_bytes_high_byte << 8)|data_bytes_low_byte);


    // Publish data_bytes raw measurement
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "echosounder";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.1745329252; //10 degrees
    msg.min_range = profile_minimum_range;
    msg.max_range = range;
    msg.range = data_bytes;
    // Filter zeros
    if(data_bytes >= profile_minimum_range){
      data_bytes_raw_pub_.publish(msg);
    }
    
    // Publish profile_range raw measurement
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "echosounder";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.1745329252; //10 degrees
    msg.min_range = profile_minimum_range;
    msg.max_range = range;
    msg.range = profile_range;
    // Filter zeros
    if(profile_range >= profile_minimum_range){
      profile_range_raw_pub_.publish(msg);
    }
  
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher altitude_raw_pub_;
  ros::Publisher altitude_filtered_pub_;
  ros::Timer timer_;
  int64_t seq_;
  TimeoutSerial serial;
  double profile_minimum_range, profile_range_high_byte, profile_range_low_byte, profile_range, old_profile_range,range;
  double data_bytes_high_byte, data_bytes_low_byte, data_bytes,timerDuration;
  int gain, absorption,long_pulso,delay,data_points,profile,timeoutSerial; 

  void getConfig() {
    bool valid_config = true;

    valid_config = valid_config && ros::param::getCached("~range", range);
    valid_config = valid_config && ros::param::getCached("~profile_minimum_range", profile_minimum_range);
    valid_config = valid_config && ros::param::getCached("~gain", gain);
    valid_config = valid_config && ros::param::getCached("~pulse_length", pulse_length);
    valid_config = valid_config && ros::param::getCached("~delay", delay);
    valid_config = valid_config && ros::param::getCached("~data_points", data_points);
    valid_config = valid_config && ros::param::getCached("~profile", profile);
    valid_config = valid_config && ros::param::getCached("~timeoutSerial", timeoutSerial);
    valid_config = valid_config && ros::param::getCached("~timerDuration", timerDuration);
    valid_config = valid_config && ros::param::getCached("~absorption", absorption);

    // Shutdown if not valid
    if (!valid_config) {
        ROS_FATAL_STREAM("Shutdown due to invalid config parameters!");
        ros::shutdown();
    	}
	}
};


int main(int argc, char** argv){
  ros::init(argc, argv, "imagenex_echosounder"); 
  ros::NodeHandle nh; 
  ros::NodeHandle nhp("~"); 
  imagenex_echosounder ec(nh, nhp);
  ros::spin();
  return 0;
};
