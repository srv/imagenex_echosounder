#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "TimeoutSerial.h"

class Ecosonda {
 public:
 
  Ecosonda(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
      : nh_(nh), nhp_(nhp), seq_(0)  {
	// B: ¿Parametros iniciales?
    nhp_.param("profundidad_minima", profundidad_min, 0.5);
    nhp_.param("profundidad_maxima", profundidad_max, 5.0);
    nhp_.param("ganancia", ganancia, 6);
    nhp_.param("longitud_pulso", long_pulso, 100);
    nhp_.param("delay", delay, 0);

    range_pub_ = nhp_.advertise<sensor_msgs::Range>("/ecosonda/range", 1); // B: ¿Anuncia un topic/tema de un mensaje? Es usado a la hora de crear el mensaje. Supongo que crea el topic del mensaje.
    
    serial.open(115200); // B: Se abre un puerto serial y se le asigna una velocidad. ¿Que puerto se abre? ¿COM1?
	serial.setTimeout(boost::posix_time::seconds(5)); // B: No lo entiendo... ¿Es un delay? 

    timer_ = nh_.createTimer(ros::Duration(1.0), // B: CreateTimer crea un temporizador. El temporizador usado es el propio de ROS. El temporizador dura 1 s.
                             &Ecosonda::timerCallback, // B: Se llama al método privado timerCallback.
                             this);
  }

 private:
  // B: ¿Se llama a este método cada vez que el timer de ROS llega al final y ocurre un "TimerEvent&"?
  
  void timerCallback(const ros::TimerEvent&) {
	 // B: Posible Breakpoint... Pasaremos por aquí cada segundo...
	// B: Pre-tratamiento de los datos
	// B: ¿Por que se hacen estos redondeos tan brutales? 
	if(profundidad_max < 7.5) profundidad_max = 5.0;
    else if (profundidad_max < 15.0) profundidad_max = 10.0;
    else if (profundidad_max < 25.0) profundidad_max = 20.0;
    else if (profundidad_max < 35.0) profundidad_max = 30.0;
    else if (profundidad_max < 45.0) profundidad_max = 40.0;
    else profundidad_max = 50.0;
            
    if(profundidad_min < 0) profundidad_min = 0;
    else if (profundidad_min > 25.0) profundidad_min = 25.0; // B: ¿Por que el mínimo es de 25.0 m? 
            
    if(ganancia > 40) ganancia = 40;
    else if(ganancia < 0) ganancia = 0;
    
	// B: El rango del pulso es de 1 a 255. Pero ¿por que se considera 253=254?
    if (long_pulso >255) long_pulso = 255;
    else if (long_pulso == 253) long_pulso = 254;
	else if (long_pulso < 1) long_pulso = 1;
            
    if(delay/2 == 253) delay = 508; // B: Ajuste para que el delay de 253 sea el del 254.
    
	// B: una vez que los datos son pre-tratados los metemos en los buffers
	
	buffer_tx[0] = 0xFE;		        //Switch Data Header (1st Byte)
	buffer_tx[1] = 0x44;			    //Switch Data Header (2nd Byte)
    	buffer_tx[2] = 0x11;	    		//Head ID
	buffer_tx[3] = profundidad_max;		//Range: 5,10,20,30,40,50 in meters
	buffer_tx[4] = 0;
	buffer_tx[5] = 0;
	buffer_tx[6] = 0x43;				//Master/Slave: Slave mode only (0x43)
	buffer_tx[7] = 0;
	buffer_tx[8] = ganancia;			//Start Gain: 0 to 40dB in 1 dB increments
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
	buffer_tx[19] = 25;					//Data Points: 25=250 points 'IMX'
	buffer_tx[20] = 0;
	buffer_tx[21] = 0;
	buffer_tx[22] = 0;					//Profile: 0=OFF, 1=IPX output
	buffer_tx[23] = 0;
	buffer_tx[24] = delay/2;			//Switch Delay: (delay in milliseconds)/2
	buffer_tx[25] = 0;					//Frequency: 0=675kHz
	buffer_tx[26] = 0xFD;				//Termination Byte - always 0xFD
	
	
	// B: ¿Mejora la lectura humana de los datos y los metemos en los Buffers rx?
    	// B: ¿Por que escribe y lee automáticamente si despues los datos se envian mediante un mensaje?
    	serial.write(reinterpret_cast<char*>(buffer_tx), sizeof(buffer_tx)); // B: Escribe el contenido en caracteres y el tamaño del Buffer Tx en el serial 
	try {
	serial.read(reinterpret_cast<char*>(buffer_rx), sizeof(buffer_rx)); // B: Lee el contenido en caracteres y el tamaño del Buffer Rx en el serial ¿Por que? ¿Para que lo pueda leer una persona bien?
	} catch (...){
	}
	profundidad = 0.01 * float(((buffer_rx[9] & 0x7F) << 7) | (buffer_rx[8] & 0x7F)); //B: ¿Por que se lee la posición 9 del buffer si siempre es 0?
	
	
	// B: Se crea la trama/mensaje para enviar la información.
	sensor_msgs::Range msg; // B: ¿Se llama al método Range msg para obtener el topic?
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "ecosonda";
	msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	msg.field_of_view = 0.1745329252; //10 grados
	msg.min_range = profundidad_min;
	msg.max_range = profundidad_max;
	msg.range = profundidad;
	ROS_INFO("Profunditat max: %f", msg.max_range);
	ROS_INFO("Profunditat min: %f", msg.min_range);
	ROS_INFO("Profunditat: %f", msg.range);
    	range_pub_.publish(msg);
    

  }
  
  
  // B: Declaraciones de variables, buffers, etc.
  
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher range_pub_;
  ros::Timer timer_;
  int64_t seq_;
  
  TimeoutSerial serial;
  
  unsigned char buffer_rx[265];
  unsigned char buffer_tx[27];
  double profundidad;
  double profundidad_min;//Distancia mínima la cual la ecosonda digitaliza la señal analógica recibida.
			// Digitalizar la señal sirve para obtener un perfil del fondo marino y poder cuantificarlo.
  double profundidad_max;
  int ganancia; //Amplifica la señal obtenida pero también el ruido (También llamado sensibilidad)
  double absorcion;
  int long_pulso; //Longitud de las ondas que envía la ecosonda. Mayores longitudes son para fondos profundos.
  int delay; //Delay para enviar los datos recogidos por la ecosonda por el puerto serie.
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ecosonda"); //B: Inicializa el nodo de ros 
  ros::NodeHandle nh; //B: Arranca el nodo publico de ros
  ros::NodeHandle nhp("~"); //B: Arranca un nodo privado. ¿El nodo privado es copia del primero?
  Ecosonda ec(nh, nhp);
  ros::spin();
  return 0;
};
