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
    ROS_INFO("range percentage: %f", range_percentage);
    ROS_INFO("sample_vector_size: %i", sample_vector_size);

    //Publishers
    altitude_raw_pub_ = nhp_.advertise<sensor_msgs::Range>("altitude_raw", 1); 
    altitude_filtered_pub_ = nhp_.advertise<sensor_msgs::Range>("altitude_filtered", 1); 
    serial.open(115200); 
	  serial.setTimeout(boost::posix_time::seconds(timeoutSerial)); 

    timer_ = nh_.createTimer(ros::Duration(timerDuration), 
                             &imagenex_echosounder::timerCallback, 
                             this);
  	}

 private:
  void timerCallback(const ros::TimerEvent&) 
  {      
    // sanity checks according to the technical specifications included in the datasheet. 
    //profile_minimum_range
    if(profile_minimum_range < 0) profile_minimum_range = 0;
    else if (profile_minimum_range > 25.0) profile_minimum_range = 25.0; 

    //gain        
    if(gain > 40) gain = 40;
    else if(gain < 0) gain = 0; 

	  // pulse_length
    if (pulse_length >255) pulse_length = 255;
    else if (pulse_length == 253) pulse_length = 254; 
	  else if (pulse_length < 1) pulse_length = 1;
      
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
    buffer_tx[10] = 20;					
    buffer_tx[11] = 0;
    buffer_tx[12] = 0;
    buffer_tx[13] = 0;
    buffer_tx[14] = pulse_length;			
    buffer_tx[15] = profile_minimum_range/10; 
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
    profile_range = float((profile_range_high_byte << 8)|profile_range_low_byte);
    
    // Publish raw measurement
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "echosounder";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.1745329252; //10 degrees
    msg.min_range = profile_minimum_range;
    msg.max_range = range;
    msg.range = profile_range;
    ROS_INFO("Profunditat max: %f", msg.max_range);
    ROS_INFO("Profunditat min: %f", msg.min_range);
    ROS_INFO("Profunditat: %f", msg.range);
    altitude_raw_pub_.publish(msg);

    // Publish filtered measurement
    // Filter zeros

    double diff_ranges = (fabs(profile_range - old_profile_range) * 100) /profile_range; 
    if(sample_counter <= sample_vector_size){ // vector of samples to check if the sensor gets lost (range= 0.00) during a number of samples. 
      sample_vector.push_back(profile_range); 
      sample_counter++;
    }else{
      sample_counter = 0;
      average = accumulate( sample_vector.begin(), sample_vector.end(), 0.0) / sample_vector_size; 
      if (average ==0.00){  // got lost during sample_vector_size sensor samples. All samples = 0.00. 
        gotlost = true; // this varialbe is set after sample_vector_size samples equal to 0 have been detected.
      }
    }

    ROS_INFO("altitude filtered published. Diff ranges: %f, gotlost: %i, profunditat: %f, profunditat anterior %f ", diff_ranges, gotlost, profile_range,old_profile_range);	
    if (!gotlost and diff_ranges< range_percentage and profile_range != 0 and profile_range <= range and profile_range >= profile_minimum_range)
    {
      altitude_filtered_pub_.publish(msg);
      ////ROS_WARN("altitude filtered published");
      old_profile_range = profile_range; // updating the value of this variable only when the message is published 
      //avoids the possibility of having 2 or 3 consecutive outliers, assuming that the vehicle altitude will not change abruptly   
    } // on the other hand, if the sensor gets lost (value ==> 0.00) during a certain period, the new data could be 
      // far from the last stored and valid range, and this will prevent the publication of further range samples.
    if (gotlost and profile_range != 0 and profile_range <= range and profile_range >= profile_minimum_range) {
      altitude_filtered_pub_.publish(msg); // publish next range after recovered 
      old_profile_range = profile_range; // initialize the value of profundidad_anterior with the last published range   
    }
  old_profile_range = profile_range;
  } // an initialization proceduce in case of loss is needed to prevent this latter case.

  
  // declaration of private variables, buffers, etc.
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher altitude_raw_pub_;
  ros::Publisher altitude_filtered_pub_;
  ros::Timer timer_;
  int64_t seq_;
  TimeoutSerial serial;
  unsigned char buffer_rx[513];//The total number of bytes returned could be 13, 265 or 513 deppending on data_points and profile parameters
  unsigned char buffer_tx[27];
  double profile_minimum_range, profile_range_high_byte, profile_range_low_byte, profile_range, old_profile_range;
  double range; 
  int gain; 
  double absorcion;
  int long_pulso; 
  int delay; 
  int data_points;
  int profile;
  int timeoutSerial; 
  double timerDuration; 
  double range_percentage;
  int sample_vector_size;
  std::vector<double> sample_vector; 
  int sample_counter=0;
  double average;
  bool gotlost = false;

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
    valid_config = valid_config && ros::param::getCached("~range_percentage", range_percentage);
    valid_config = valid_config && ros::param::getCached("~sample_vector_size", sample_vector_size);
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
