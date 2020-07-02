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
    ROS_INFO("Profunditat max: %f", profundidad_max);
	  ROS_INFO("Profunditat min: %f", profundidad_min);
	  ROS_INFO("ganancia: %i", ganancia);
	  ROS_INFO("longitud_pulso: %i", long_pulso);
	  ROS_INFO("delay: %i", delay);
	  ROS_INFO("data_points: %i", data_points);
    ROS_INFO("timeoutSerial: %i", timeoutSerial);
    ROS_INFO("timerDuration: %f", timerDuration);
    ROS_INFO("range percentage: %f", range_percentage);

    altitude_raw_pub_ = nhp_.advertise<sensor_msgs::Range>("altitude_raw", 1); 
    altitude_filtered_pub_ = nhp_.advertise<sensor_msgs::Range>("altitude_filtered", 1); 
    serial.open(115200); // open serial port and set it velocity. 
	  serial.setTimeout(boost::posix_time::seconds(timeoutSerial)); // set the timeout of the serial port 

    timer_ = nh_.createTimer(ros::Duration(timerDuration), // Create a ROS timer with timerDuration.
                             &imagenex_echosounder::timerCallback, 
                             this);
    profundidad_anterior = 0.0; 
  	}

 private:
  void timerCallback(const ros::TimerEvent&) 
  {      
    // sanity checks according to the technical specifications included in the datasheet. 

    if(profundidad_min < 0) profundidad_min = 0;
    else if (profundidad_min > 25.0) profundidad_min = 25.0; 
    //eco sounder kit manual, pg 23: Byte 15 Profile Minimum Range Minimum range for profile point digitization
    //0 – 250 ; 0 to 25 meters in 0.1 meter increments
    // Byte 15 = min range in meters / 10
            
    if(ganancia > 40) ganancia = 40;
    else if(ganancia < 0) ganancia = 0; //Start Gain: 0 to 40dB in 1 dB increments, see data sheet
    
	// # longitud_pulso --> Byte 14 Pulse Length, Length of acoustic transmit pulse. 1-255. 1 to 255 micro sec in 1 micro sec increments

    if (long_pulso >255) long_pulso = 255;
    else if (long_pulso == 253) long_pulso = 254; // Do not Comment this setting. It is mandatory.
	  else if (long_pulso < 1) long_pulso = 1;
            
    if(delay/2 == 253) delay = 508; 
    //see Datasheet: The echo sounder can be commanded to pause (from 0 to 510 msec) before sending its return data to allow the commanding program
    // enough time to setup for serial reception of the return data. 0 to 255 in 2 msec increments Byte 24 = delay_in_milliseconds/2 Do not use a value of 253!
      
    // Set the values of the Switch message. The Switch message is the message from computer to Echosounder to 
    // pull a range message from the Echosounder. 
    
    buffer_tx[0] = 0xFE;		        //Switch Data Header (1st Byte)
    buffer_tx[1] = 0x44;			    //Switch Data Header (2nd Byte)
    buffer_tx[2] = 0x11;	    		//Head ID
    buffer_tx[3] = profundidad_max;		//Range: 5,10,20,30,40,50 in meters
    buffer_tx[4] = 0;
    buffer_tx[5] = 0;
    buffer_tx[6] = 0x43;				//Master/Slave: Slave mode only (0x43)
    buffer_tx[7] = 0;
    buffer_tx[8] = ganancia;			//Start Gain: 0 to 40dB in 1 dB increments, ecosound kit manual ,pg 23
    buffer_tx[9] = 0;
    buffer_tx[10] = 20;					//Absorption: 20 = 0.2dB/m for 675kHz
    buffer_tx[11] = 0;
    buffer_tx[12] = 0;
    buffer_tx[13] = 0;
    buffer_tx[14] = long_pulso;			//Pulse Length: 100 microseconds
    buffer_tx[15] = profundidad_min*10; //Minimun Range: 0-25m in 0.1 increments
    buffer_tx[16] = 0;
    buffer_tx[17] = 0;
    buffer_tx[18] = 0;					//External Trigger Control
    buffer_tx[19] = data_points;		//Data Points: 25=250 points 'IMX'
    buffer_tx[20] = 0;
    buffer_tx[21] = 0;
    buffer_tx[22] = 0;					//Profile: 0=OFF, 1=IPX output
    buffer_tx[23] = 0;
    buffer_tx[24] = delay/2;			//Switch Delay: (delay in milliseconds)/2
    buffer_tx[25] = 0;					//Frequency: 0=675kHz
    buffer_tx[26] = 0xFD;				//Termination Byte - always 0xFD
    
    
    serial.write(reinterpret_cast<char*>(buffer_tx), sizeof(buffer_tx)); // write the Switch message in the serial port
    try {
      serial.read(reinterpret_cast<char*>(buffer_rx), sizeof(buffer_rx)); // read the data buffer received in the serial port
    } catch (...)
    {
    }
	
    profundidad = 0.01 * float(((buffer_rx[9] & 0x7F) << 7) | (buffer_rx[8] & 0x7F)); 
    
    // Publish raw measurement
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "echosounder";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.1745329252; //10 grados
    msg.min_range = profundidad_min;
    msg.max_range = profundidad_max;
    msg.range = profundidad;
    ROS_INFO("Profunditat max: %f", msg.max_range);
    ROS_INFO("Profunditat min: %f", msg.min_range);
    ROS_INFO("Profunditat: %f", msg.range);
    altitude_raw_pub_.publish(msg);

    // Publish filtered measurement
    // Filter zeros

    double diff_ranges = fabs(profundidad - profundidad_anterior) * 100 /profundidad; // in percentage
    // double diff_ranges = fabs(profundidad - profundidad_anterior); // in absolute value in m
    if(sample_counter <= sample_vector_size){ // vector of samples to check if the sensor gets lost (range= 0.00) during a number of samples. 
      sample_vector.push_back(profundidad); 
      sample_counter++;
    }else{
      sample_counter=0;
      average = accumulate( sample_vector.begin(), sample_vector.end(), 0.0) / sample_vector_size; 
      if (average ==0.00){  // got lost during sample_vector_size sensor samples. All samples = 0.00. 
        gotlost = true; // this varialbe is set after sample_vector_size samples equal to 0 have been detected.
        ROS_WARN("Echosounder Got Lost during: %i samples", sample_vector_size);
      }
    }


    if (!gotlost and diff_ranges< range_percentage and profundidad != 0 and profundidad <= profundidad_max and profundidad >= profundidad_min)
    {
      altitude_filtered_pub_.publish(msg);
      profundidad_anterior=profundidad; // updating the value of this variable only when the message is published 
      //avoids the possibility of having 2 or 3 consecutive outliers, assuming that the vehicle altitude will not change abruptly   
    } // on the other hand, if the sensor gets lost (value ==> 0.00) during a certain period, the new data could be 
      // far from the last stored and valid range, and this will prevent the publication of further range samples.
    if (gotlost and profundidad != 0 and profundidad <= profundidad_max and profundidad >= profundidad_min) {
      altitude_filtered_pub_.publish(msg); // publish next range after recovered 
      profundidad_anterior=profundidad; // initialize the value of profundidad_anterior with the last published range
      }
  } // an initialization proceduce in case of loss is needed to prevent this latter case.
  
  
  // declaration of private variables, buffers, etc.
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher altitude_raw_pub_;
  ros::Publisher altitude_filtered_pub_;
  ros::Timer timer_;
  int64_t seq_;
  
  TimeoutSerial serial;
  
  unsigned char buffer_rx[265];
  unsigned char buffer_tx[27];
  double profundidad, profundidad_anterior;
  double profundidad_min;// minimum distance detected by the sensor.
  double profundidad_max; // maximum range: 5, 10, 20
  int ganancia; // amplifies the received signal
  double absorcion;
  int long_pulso; // pulse lenght. Greated lenghts for greater distances.
  int delay; 
  int data_points;
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

    valid_config = valid_config && ros::param::getCached("~profundidad_maxima", profundidad_max);
    valid_config = valid_config && ros::param::getCached("~profundidad_minima", profundidad_min);
    valid_config = valid_config && ros::param::getCached("~ganancia", ganancia);
    valid_config = valid_config && ros::param::getCached("~longitud_pulso", long_pulso);
    valid_config = valid_config && ros::param::getCached("~delay", delay);
    valid_config = valid_config && ros::param::getCached("~data_points", data_points);
    valid_config = valid_config && ros::param::getCached("~timeoutSerial", timeoutSerial);
    valid_config = valid_config && ros::param::getCached("~timerDuration", timerDuration);
    valid_config = valid_config && ros::param::getCached("~range_percentage", range_percentage);
    valid_config = valid_config && ros::param::getCached("~range_percentage", sample_vector_size);
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
