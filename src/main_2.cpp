/*
	RFT Force Torque sensor. - serial communication version
    
	Simple stand-alone ROS node that takes data from RFT sensor 
	and Publishes it ROS topic	
	
	website: www.robotous.com
	 e-mail: support@robotous.com
	
	Development & Test Evironment
	  - Ubuntu 16.04 LTS
	  - ROS Kinetic
*/

/*
	ver. 0.0.0, 2017.11.29
	Ver. 0.0.1, 2017.12.18
*/
#define ROS_RFT_SERIAL_SW_VER	"VER 0.0.1(Read Only)"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <mutex>

#include "RFT_COM.h"

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "rft_sensor_serial/rft_operation_2.h"	// this file will be automatically generated during build operation.
												// catkin_ws -> devel -> include -> rft_sensor_serial

/*
	Definitions
*/
#define RFT_SERVICE_OK  			(0)
#define RFT_SERVICE_RQST_TIMEOUT 	(0xF0)

/*
	Global Variables
*/
std::string g_com_port_2 		= "/dev/ttyUSB0";	// default serial device name...
int 		g_baud_rate 		= 921600; //921600;//115200;			// default baud-rate serial device...
float		g_force_divider 	= 50.0f;			// for more information to refer the RFT sensor manual
float		g_torque_divider 	= 2000.0f;			// for more information to refer the RFT sensor manual
std::mutex  g_com_port_mutex;
char* serial_number; 

CRT_RFT_UART RFT_SENSOR_2;

/*
	Funcitions
*/
void init_param_2(ros::NodeHandle *n);
bool rft_operation_service(rft_sensor_serial::rft_operation_2::Request &req, rft_sensor_serial::rft_operation_2::Response &res);
uint8_t rft_response_wait(uint8_t opType);
uint8_t rft_response_display(uint8_t opType);
uint8_t rft_send_command(rft_sensor_serial::rft_operation_2::Request &req);

/*
	The main function of RFT Serial Node.....
	- you can send the operation command using rqt -> service caller
	- you can see the force/torque data using rqt_plot
	  * rqt_plot /RFT_FORCE/wrench/force
	  * rqt_plot /RFT_FORCE/wrench/torque
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "rft_sensor_serial_2");
	ROS_INFO("Initializing rft_sensor_serial_2 Node");
	ros::NodeHandle nh_2;
	
	// initialize parameters
	init_param_2( &nh_2 );
	
	// initialize COM port
	if( RFT_SENSOR_2.openPort( (char*)g_com_port_2.c_str(), g_baud_rate ) == false )
	{
		ROS_ERROR("COM Port Open Error");
		return 0;
	}
	
	// initialize force/torque divider
	RFT_SENSOR_2.m_RFT_IF_PACKET.setDivider(g_force_divider, g_torque_divider);
	
	//service
	ros::ServiceServer op_service = nh_2.advertiseService("rft_serial_op_service_2", rft_operation_service);
	//publisher
	geometry_msgs::WrenchStamped ft_data;
	ros::Publisher rft_publisher = nh_2.advertise<geometry_msgs::WrenchStamped>("RFT_FORCE_2", 1);
	
	ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
	
	ROS_INFO("RFT Force/Torque Sensor <Serial> is ready!!!!");
	
	bool isSensorOk = false;
	unsigned long seq = 0;
	while(ros::ok())
	{
		std::unique_lock<std::mutex> lock(g_com_port_mutex);
		isSensorOk = RFT_SENSOR_2.readWorker();
		lock.unlock();
		
		if( (RFT_SENSOR_2.m_nCurrMode == CMD_FT_CONT) && isSensorOk )
		{
			seq++;
			ft_data.header.stamp = ros::Time::now();
			ft_data.header.seq = seq;
			ft_data.header.frame_id = serial_number;
			ft_data.wrench.force.x = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[0];
			ft_data.wrench.force.y = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[1];
			ft_data.wrench.force.z = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[2];
			ft_data.wrench.torque.x = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[3];
			ft_data.wrench.torque.y = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[4];
			ft_data.wrench.torque.z = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[5];
			rft_publisher.publish(ft_data);
			
			//ROS_INFO("Published.... %ld", seq);
		}
		
		usleep(10);
	}
	
	return 0;
}

/*
	intialize parameters.
*/
void init_param_2(ros::NodeHandle *n)
{
	// parameter serial port device
	if(!n->getParam("RFT_COM_PORT_2", g_com_port_2))
	{
		ROS_WARN("RFT Serial port is not defined, The port is initialized with %s", g_com_port_2.c_str());
		n->setParam("RFT_COM_PORT_2", g_com_port_2.c_str()); // set param
		n->getParam("RFT_COM_PORT_2", g_com_port_2);    // checking 
	}
	ROS_INFO("RFT Serial port: %s", g_com_port_2.c_str());
	
	// parameter serial port device
	if(!n->getParam("RFT_COM_BAUD_2", g_baud_rate))
	{
		ROS_WARN("The baud-rate of RFT Serial port is not defined, The baud-rate is initialized with %d", g_baud_rate);
		n->setParam("RFT_COM_BAUD_2", g_baud_rate); // set param
		n->getParam("RFT_COM_BAUD_2", g_baud_rate);    // checking 
	}
	ROS_INFO("RFT Serial port baud-rate: %d", g_baud_rate);
	
	// parameter force/torque divider
	if(!n->getParam("RFT_FORCE_DEVIDER_2", g_force_divider))
	{
		ROS_WARN("The force divider of RFT sensor is not defined, The force divider is initialized with %f", g_force_divider);
		n->setParam("RFT_FORCE_DEVIDER_2", g_force_divider); // set param
		n->getParam("RFT_FORCE_DEVIDER_2", g_force_divider);    // checking 
	}
	ROS_INFO("Force Divider of RFT sensor: %f", g_force_divider);

	if(!n->getParam("RFT_TORQUE_DEVIDER_2", g_torque_divider))
	{
		ROS_WARN("The torque divider of RFT sensor is not defined, The torque divider is initialized with %f", g_torque_divider);
		n->setParam("RFT_TORQUE_DEVIDER_2", g_torque_divider); // set param
		n->getParam("RFT_TORQUE_DEVIDER_2", g_torque_divider);    // checking 
	}
	ROS_INFO("Torque Divider of RFT sensor: %f", g_torque_divider);	
	
	// version setting... only to check the s/w version.
	n->setParam("RFT_SERIAL_SW_VER_2", ROS_RFT_SERIAL_SW_VER);
}

/*
	service operation...
*/
bool rft_operation_service(rft_sensor_serial::rft_operation_2::Request &req, rft_sensor_serial::rft_operation_2::Response &res)
{
	//ROS_INFO("RCVD SERVICE REQUEST: %d, %d, %d, %d", req.opType, req.param1, req.param2, req.param3);

	uint8_t commandSend = RFT_SERVICE_OK;
		
	std::unique_lock<std::mutex> lock(g_com_port_mutex);
	commandSend = rft_send_command( req );
	lock.unlock();
	
	if( commandSend == RFT_SERVICE_OK )
	{
		//ROS_INFO("RESPONSE WAIT");
		// Set bias & Stop command doesn't have response packet.
		if( (req.opType != CMD_SET_BIAS) && (req.opType != CMD_FT_CONT_STOP) ) 
		{
			res.result = rft_response_wait(req.opType);
		}
	}
	else
	{
		res.result = commandSend;
	}
	
	return true;
}

/*
	Send the received operation command
	The detail of the operaion of the RFT Sensor, Please refer to the manual
*/
uint8_t rft_send_command(rft_sensor_serial::rft_operation_2::Request &req)
{
	uint8_t result = RFT_SERVICE_OK;
	switch( req.opType )
	{
		case CMD_GET_PRODUCT_NAME:
			ROS_INFO("RCVD SERVICE REQUEST: get product name");
			RFT_SENSOR_2.rqst_ProductName();
		break;

		case CMD_GET_SERIAL_NUMBER:		
			ROS_INFO("RCVD SERVICE REQUEST: get serial number");
			RFT_SENSOR_2.rqst_SerialNumber();
		break;

		case CMD_GET_FIRMWARE_VER:	
			ROS_INFO("RCVD SERVICE REQUEST: get firmware version");
			RFT_SENSOR_2.rqst_Firmwareverion();
		break;

		case CMD_SET_ID:					// Only CAN version
			ROS_INFO("RCVD SERVICE REQUEST: set ID - is not supported");
			result = NOT_SUPPORTED_CMD;
		break;

		case CMD_GET_ID:					// Only CAN version
			ROS_INFO("RCVD SERVICE REQUEST: get ID - is not supported");
			result = NOT_SUPPORTED_CMD;
		break;

		case CMD_SET_COMM_BAUDRATE:			// Only UART version, CAN : 1Mbps Fixed
			ROS_INFO("RCVD SERVICE REQUEST: set baud-rate");
			RFT_SENSOR_2.set_Comm_Speed(req.param1);		
		break;

		case CMD_GET_COMM_BAUDRATE:	
			ROS_INFO("RCVD SERVICE REQUEST: get baud-rate");
			RFT_SENSOR_2.rqst_CommSpeed();	
		break;

		case CMD_SET_FT_FILTER:	
			ROS_INFO("RCVD SERVICE REQUEST: set filter type");
			RFT_SENSOR_2.set_FT_Filter_Type(req.param1, req.param2);			
		break;

		case CMD_GET_FT_FILTER:				
			ROS_INFO("RCVD SERVICE REQUEST: get filter type");
			RFT_SENSOR_2.rqst_FT_Filter_Type();	
		break;

		case CMD_FT_ONCE:	
			ROS_INFO("RCVD SERVICE REQUEST: get force/torque once");
			RFT_SENSOR_2.rqst_FT();			
		break;

		case CMD_FT_CONT:	
			ROS_INFO("RCVD SERVICE REQUEST: get force/torque cont.");
			RFT_SENSOR_2.rqst_FT_Continuous();
		break;

		case CMD_FT_CONT_STOP:			
			ROS_INFO("RCVD SERVICE REQUEST: stop force/torque  - There is no response packet");
			RFT_SENSOR_2.rqst_FT_Stop();
		break;

		case CMD_RESERVED_1:	
			result = NOT_SUPPORTED_CMD;		
		break;

		case CMD_RESERVED_2:				
			result = NOT_SUPPORTED_CMD;
		break;

		case CMD_SET_CONT_OUT_FRQ: 	
			ROS_INFO("RCVD SERVICE REQUEST: set output frq.");
			RFT_SENSOR_2.set_FT_Cont_Interval(req.param1);		
		break;

		case CMD_GET_CONT_OUT_FRQ: 
			ROS_INFO("RCVD SERVICE REQUEST: get output frq.");
			RFT_SENSOR_2.rqst_FT_Cont_Interval();			
		break;

		case CMD_SET_BIAS:		
			ROS_INFO("RCVD SERVICE REQUEST: set bias - There is no response packet and keep the last received commands....");
			RFT_SENSOR_2.set_FT_Bias(req.param1);			
		break;

		case CMD_GET_OVERLOAD_COUNT:
			ROS_INFO("RCVD SERVICE REQUEST: get overload count");
			RFT_SENSOR_2.rqst_FT_OverloadCnt();		
		break;
	default:
		result = NOT_SUPPORTED_CMD;
	break;
	}	

	return result;
}

/*
	Display the response packet
	The detail of the response packet of the RFT Sensor, Please refer to the manual
*/
uint8_t rft_response_display(uint8_t opType)
{
	uint8_t result = RFT_SERVICE_OK;
	switch ( opType )
	{
	case CMD_GET_PRODUCT_NAME:
		ROS_INFO("%s", RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_product_name);
		break;
	case CMD_GET_SERIAL_NUMBER:
		serial_number = RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_serial_number;
		ROS_INFO("%s", RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_serial_number);
		break;
	case CMD_GET_FIRMWARE_VER:
		ROS_INFO("%s", RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_firmware_version);
		break;
	case CMD_SET_ID:
		result = NOT_SUPPORTED_CMD;
		break;
	case CMD_GET_ID:
		result = NOT_SUPPORTED_CMD;
		break;
	case CMD_SET_COMM_BAUDRATE:
		ROS_INFO("Cmd Type: %d, Result: %d, Err. Code: %d", RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_cmd, RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_result, RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_errcode );
		result = RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_errcode;
		break;
	case CMD_GET_COMM_BAUDRATE:
		ROS_INFO("Baud: %d, new Baud: %d", RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_curr_comm_baudrate, RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_set_comm_baudrate);
		break;
	case CMD_SET_FT_FILTER:
		ROS_INFO("Cmd Type: %d, Result: %d, Err. Code: %d", RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_cmd, RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_result, RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_errcode );
		result = RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_errcode;
		break;
	case CMD_GET_FT_FILTER:
		ROS_INFO("filter type: %d, sub. setting: %d", RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_filter_type, RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_filter_setting_value);
		break;
	case CMD_FT_ONCE:
		ROS_INFO("%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f",
		RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[0], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[1], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[2],
		RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[3], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[4], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdForce[5] );	
		break;
	case CMD_FT_CONT:
		// This response packet is published.. see the while() of main() function.
		ROS_INFO("RFT Force Toque topic will be published...");
		break;
	case CMD_FT_CONT_STOP:
		// THERE IS NO - RESPONSE PACKET
		break;
	case CMD_RESERVED_1:
		result = NOT_SUPPORTED_CMD;
		break;
	case CMD_RESERVED_2:
		result = NOT_SUPPORTED_CMD;
		break;
	case CMD_SET_CONT_OUT_FRQ:
		ROS_INFO("Cmd Type: %d, Result: %d, Err. Code: %d", RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_cmd, RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_result, RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_errcode );
		result = RFT_SENSOR_2.m_RFT_IF_PACKET.m_response_errcode;
		break;
	case CMD_GET_CONT_OUT_FRQ:
		ROS_INFO("output frq: %d", RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvd_tx_frq);
		break;
	case CMD_SET_BIAS:
		// THERE IS NO - RESPONSE PACKET
		break;
	case CMD_GET_OVERLOAD_COUNT:
		ROS_INFO("overload ount: %d, %d, %d, %d, %d, %d", 
		RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdOverloadCnt[0], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdOverloadCnt[1], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdOverloadCnt[2],
		RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdOverloadCnt[3], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdOverloadCnt[4], RFT_SENSOR_2.m_RFT_IF_PACKET.m_rcvdOverloadCnt[5] );
		break;

	default:
		result = NOT_SUPPORTED_CMD;
		break;
	}	
	
	return result;
}

/*
	Wait the response packet from the RFT sensor
*/
uint8_t rft_response_wait(uint8_t opType)
{
	int result = RFT_SERVICE_OK;
	int waitTimeOut = 0;
	
	bool isRcvd = false;
	do
	{
		if( waitTimeOut >= 50 ) // 50
		{
			ROS_WARN("RCVD SERVICE TIMEOUT");
			isRcvd = true;
			result = RFT_SERVICE_RQST_TIMEOUT;
		}
		
		if( RFT_SENSOR_2.m_bIsRcvd_Response_Pkt)
		{
			isRcvd = true;
			rft_response_display(opType);
		}
		
		waitTimeOut++;
		usleep(10000);
		
	}while( isRcvd == false );
	
	return result;
}


