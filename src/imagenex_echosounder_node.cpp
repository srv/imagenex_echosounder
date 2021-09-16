#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "TimeoutSerial.h"
#include <numeric>      // std::accumulate

class imagenex_echosounder {
 public:
 
  imagenex_echosounder(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
      : nh_(nh), nhp_(nhp), seq_(0)  
    {

    // Get configuration from rosparam server
    getConfig();
    ROS_INFO("Profunditat max: %f", profile_maximum_range);
	  ROS_INFO("Profunditat min: %f", profile_minimum_range);
    ROS_INFO("profile (ON - 1, OFF - 0)): %d", profile);
	  ROS_INFO("gain: %i", gain);
	  ROS_INFO("pulse_length: %i", pulse_length);
	  ROS_INFO("delay: %i", delay);
	  ROS_INFO("data_points: %i", data_points);
    ROS_INFO("timeoutSerial: %i", timeoutSerial);
    ROS_INFO("absorption: %i", absorption);
    ROS_INFO("range percentage: %f", range_percentage);
    ROS_INFO("sample_vector_size: %i", sample_vector_size);
    ROS_INFO("devname: %s", devname.c_str());
    ROS_INFO("num_samples_not_published: %i", num_samples_not_published);
    

    //Publishers
    profile_range_raw_pub_ = nhp_.advertise<sensor_msgs::Range>("profile_range_raw", 1); 
    data_bytes_raw_pub_ = nhp_.advertise<sensor_msgs::Range>("data_bytes_raw", 1);
    profile_range_filtered_pub_ = nhp_.advertise<sensor_msgs::Range>("profile_range_filtered", 1); 
    profile_rx_raw = nhp_.advertise<std_msgs::UInt8MultiArray>("profile_rx_raw", 1); 
    profile_rx_raw_float = nhp_.advertise<std_msgs::Float32MultiArray>("profile_rx_raw_float", 1);

    //Set serial
    serial.open(devname,115200); // open serial port and set it velocity.    
	  serial.setTimeout(boost::posix_time::seconds(timeoutSerial)); 

    //Check params
    checkParams();

    //Build TX
    buildTX();

    //Iterate
    iterate();
  	}

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher profile_range_raw_pub_;
  ros::Publisher profile_range_filtered_pub_;
  ros::Publisher data_bytes_raw_pub_;
  ros::Publisher profile_rx_raw;
  ros::Publisher profile_rx_raw_float;
  ros::Timer timer_;
  TimeoutSerial serial;
  
  int64_t seq_;
  double profile_minimum_range, profile_maximum_range, profile_range_high_byte, profile_range_low_byte, profile_range, old_profile_range,range, previous_profile_range;
  double data_bytes_high_byte, data_bytes_low_byte, data_bytes, timerDuration, idle_time, fov;
  int gain, absorption,long_pulso,delay,data_points,profile,timeoutSerial,num_samples_not_published;
  int bad_sample_counter=-1; 
  int DataBytes12, DataBytes13, DataBytes14, DataBytes15, DataBytes16;
  int DataBytes17, DataBytes18, DataBytes19, DataBytes20, DataBytes21;
  std::string devname, frame_id;
  std::vector<double> sample_vector; 
  int sample_counter=0;
  double average, range_percentage; 
  bool gotlost = false;
  int sample_vector_size, pulse_length,loop_time_step;
  unsigned char buffer_tx[27];
  unsigned char buffer_rx[513];
  bool firstime=true;

  void checkParams() {
    //profile_minimum_range
    if(profile_minimum_range < 0) profile_minimum_range = 0;
    else if (profile_minimum_range > 25.0) profile_minimum_range = 25.0; 

    //gain        
    if(gain > 40) gain = 40;
    else if(gain < 0) gain = 0; 

	  // pulse_length
    if (pulse_length >255) pulse_length = 255;
    else if (pulse_length == 253) pulse_length = 254; // 253 is reserved for end of frame change to 254
	  else if (pulse_length < 1) pulse_length = 1;

    if(delay/2 == 253) delay = 254*2; // 253 is reserved for end of frame change to 254
	}

  void buildTX() {

    // Set the values of the Switch message. The Switch message is the message from computer to Echosounder to pull a range message from the Echosounder. 
    buffer_tx[0] = 0xFE;		        
    buffer_tx[1] = 0x44;			    
    buffer_tx[2] = 0x11;	    		
    buffer_tx[3] = profile_maximum_range;		
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
    buffer_tx[22] = profile;					//Profile: 0=OFF, ON=1 --> IPX output. Parametrized by FBF 29/06/2021. By default it was at OFF.
    buffer_tx[23] = 0;
    buffer_tx[24] = delay/2;			
    buffer_tx[25] = 0;					
    buffer_tx[26] = 0xFD;			
    firstime=true;	
	}


  void write() {
	  serial.write(reinterpret_cast<char*>(buffer_tx), sizeof(buffer_tx));
	}

  void read() {
	 try {
      serial.read(reinterpret_cast<char*>(buffer_rx), sizeof(buffer_rx));
      ROS_INFO("Read succsesfully");
    } 
    catch (const std::exception &exc){
     ROS_WARN("Read exception");
    }
	}

  void print() {
    ROS_INFO("ASCII M or G ? : %c", buffer_rx[1]);
    if (buffer_rx[1]=='M' or buffer_rx[1]=='G') 
    {
      DataBytes12=static_cast<unsigned int>(buffer_rx[12]);
      DataBytes13=static_cast<unsigned int>(buffer_rx[13]);
      DataBytes14=static_cast<unsigned int>(buffer_rx[14]);
      DataBytes15=static_cast<unsigned int>(buffer_rx[15]);
      DataBytes16=static_cast<unsigned int>(buffer_rx[16]);

      DataBytes17=static_cast<unsigned int>(buffer_rx[17]);
      DataBytes18=static_cast<unsigned int>(buffer_rx[18]);
      DataBytes19=static_cast<unsigned int>(buffer_rx[19]);
      DataBytes20=static_cast<unsigned int>(buffer_rx[20]);

      ROS_INFO("Data Bytes: %i %i %i %i %i ", DataBytes12, DataBytes13, DataBytes14, DataBytes15, DataBytes16);
      ROS_INFO("Data Bytes: %i %i %i %i %i ", DataBytes17, DataBytes18, DataBytes19, DataBytes20, DataBytes21);
    }

	}

  void computeProfile() {
    profile_range = 0.01 * float(((buffer_rx[9] & 0x7F) << 7) | (buffer_rx[8] & 0x7F)); // this decodification is equal to the one specified in the datasheet
    data_bytes = float(((buffer_rx[11] & 0x7F) << 7) | (buffer_rx[10] & 0x7F)); // this decodification is equal to the one specified in the datasheet
    ROS_INFO(" profile_range: %f",profile_range);
    ROS_INFO("previous_profile_range %f", previous_profile_range);
    ROS_INFO(" data_bytes: %f",data_bytes);
    if (firstime){
      previous_profile_range=profile_range;
      firstime=false;
    }

	}

  void filterProfile() {
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "echosounder";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.1745329252; //10 degrees
    msg.min_range = profile_minimum_range;
    msg.max_range = profile_maximum_range;
    msg.range = profile_range;
    
    // TODO: review code and clean comments, move publisher to publish()
    double diff_ranges = (fabs(profile_range - previous_profile_range) * 100)/(std::max(profile_range,previous_profile_range)); // in percentage of the highest value
    if(sample_counter <= sample_vector_size){ // vector of samples to check if the sensor gets lost (range= 0.00) during a number of samples. 
      sample_vector.push_back(profile_range); 
      sample_counter++;
      average = accumulate( sample_vector.begin(), sample_vector.end(), 0.0) / sample_vector_size; // computes de mean. 
      if (average ==0.00){  // got lost during sample_vector_size sensor samples. All samples = 0.00. IMPORTANT: Sample = 0 means that the ecosounder is lost or OUT OF RANGE.  
        gotlost = true; // this variable is set after sample_vector_size samples equal to 0 have been detected.
        ROS_WARN("Echosounder Got Lost during: %i samples", sample_vector_size);
      }
    }
    if(sample_counter > sample_vector_size) sample_counter=0;
      
    // got lost if 10 consecutive samples =0 
    /* verify the diff_ranges respect the profundidad and profundidad_anterior and why */

    // if the echosounder is not lost, and the difference in range between consecutive samples, and "profundidad" fullfills all requirements , publish the "filtered" sample. 
    // IMPORTANT: do not consider altitudes of exactly 0.5 since they are outliers caused when the sensor approximates to its lowest range. DEPTHS LOWER THAN 0.5 ARE NOT RELIABLE. MINIMUM RANGE DETECTABLE = 0.5 M
    ROS_INFO(" diff_ranges: %f",diff_ranges);
    ROS_INFO(" gotlost: %i",gotlost);

    if (!gotlost and (diff_ranges<range_percentage) and (profile_range != 0) and (profile_range <= profile_maximum_range) and (profile_range > profile_minimum_range)) {
      bad_sample_counter=-1;
      profile_range_filtered_pub_.publish(msg);
      ROS_INFO("ecosound no lost, altitude filtered published");
      previous_profile_range=profile_range; // updating the value of this variable only when the message is published 
      //avoids the possibility of having 2 or 3 consecutive outliers, assuming that the vehicle altitude will not change abruptly   
     
    // on the other hand, if the sensor gets lost (value ==> 0.00) during a certain period, the new data could be 
    // far from the last stored and valid range, and this will prevent the publication of further range samples WHEN IT RECOVERS. So, lets publish new range data after the sensor recovers. 
    // how will we know that the sensor has recovered ? it gotlost ("profundidad = 0"), but the "profundidad" is again <> 0 (recovered !!!), and in the range of accepted values. 
    }else if (gotlost and (profile_range != 0) and (profile_range <= profile_maximum_range) and (profile_range >= profile_minimum_range)) {
      ROS_INFO("got lost, profile_range recovered, publish filtered range again");
      profile_range_filtered_pub_.publish(msg); // publish next range after recovered 
      previous_profile_range=profile_range; // initialize the value of profundidad_anterior with the last published range
      gotlost=false;  
      bad_sample_counter=-1;

    }else{
      //previous_profile_range not updated
        bad_sample_counter++;
        ROS_INFO("Elseee 1!!!!");

      if((bad_sample_counter>num_samples_not_published) and (profile_range != 0) and (profile_range <= profile_maximum_range) and (profile_range >= profile_minimum_range) ){
        bad_sample_counter=-1;
        previous_profile_range=profile_range;
        ROS_INFO("UPDATING previous_profile rangeeeeeeeeeeeeeeeeeeeeee");
      }  
    }
	}

  void publish() {
    // Publish RX
    std_msgs::UInt8MultiArray rx_arr;
    std_msgs::Float32MultiArray rx_arr_float;
    for (int i=0; i<=513;i++){
      rx_arr.data.push_back(buffer_rx[i]);
      rx_arr_float.data.push_back(float(buffer_rx[i]));
    }
    profile_rx_raw.publish(rx_arr);
    profile_rx_raw_float.publish(rx_arr_float);

    // Publish range raw
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = fov; //10 degrees
    msg.min_range = profile_minimum_range;
    msg.max_range = profile_maximum_range;
    msg.range = profile_range;
    profile_range_raw_pub_.publish(msg);
	}

  void sleep() {
    ros::Duration(idle_time).sleep();  
	}

  void iterate() {
    while (ros::ok()){
      write();
      read();
      print();
      computeProfile();
      filterProfile();
      publish();
      sleep();
    } 
	}

  void getConfig() {
    bool valid_config = true;

    valid_config = valid_config && ros::param::getCached("~profile_maximum_range", profile_maximum_range);
    valid_config = valid_config && ros::param::getCached("~profile_minimum_range", profile_minimum_range);
    valid_config = valid_config && ros::param::getCached("~gain", gain);
    valid_config = valid_config && ros::param::getCached("~pulse_length", pulse_length);
    valid_config = valid_config && ros::param::getCached("~delay", delay);
    valid_config = valid_config && ros::param::getCached("~data_points", data_points);
    valid_config = valid_config && ros::param::getCached("~profile", profile);
    valid_config = valid_config && ros::param::getCached("~timeoutSerial", timeoutSerial);
    valid_config = valid_config && ros::param::getCached("~absorption", absorption);
    valid_config = valid_config && ros::param::getCached("~range_percentage", range_percentage);
    valid_config = valid_config && ros::param::getCached("~sample_vector_size", sample_vector_size);
    valid_config = valid_config && ros::param::getCached("~devname", devname);
    valid_config = valid_config && ros::param::getCached("~idle_time", idle_time);
    valid_config = valid_config && ros::param::getCached("~frame_id", frame_id);
    valid_config = valid_config && ros::param::getCached("~fov", fov);
    valid_config = valid_config && ros::param::getCached("~num_samples_not_published", num_samples_not_published);
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
