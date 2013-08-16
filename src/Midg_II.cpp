/**
 *
 *  \file	Midg_II.cpp
 *  \brief      Microbotics Midg II controller.
 *              Parses data from Midg and posts to Midg, Imu and GPS (fix)
 *              topics.
 *  \author     Chris Bessent <cmbq76>
 *  \author     Jeff Schmidt <jschmidt@clearpathrobotics.com>
 *  \license	BSD
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to code@clearpathrobotics.com 
 *
 */


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include "midg/IMU.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

/***********************************************************
* Other includes
***********************************************************/
#include "Midg_II.h"
#include <list>
#include "Math.h"
#include "drivers/midgPacket.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <sstream>

/***********************************************************
* Global variables
***********************************************************/
double avg_mag_x = 0;
double avg_mag_y = 0;
double heading = 0;
double reading_count = 0;
string port_name;

// sensor_msgs::NavSatFix
const bool FIX_COVARIANCE = false;
const double FIXED_COVARIANCE_VALUE = 5000;

const double g = 9.799096177;

/***********************************************************
* Function prototypes
***********************************************************/
int  Open_MIDG_Connection();
void Process_MIDG_Packets( int );

/***********************************************************
* Message Callbacks
***********************************************************/
midg::IMU               midgimu_msg;
sensor_msgs::Imu	imu_msg;
sensor_msgs::NavSatFix  navSatFix_msg;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Midg");
    ros::NodeHandle n;

    ros::Publisher midgimu_pub = n.advertise<midg::IMU>( "/midg", 1000 );
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>( "/imu", 1000 );
    ros::Publisher navSatFix_pub = n.advertise<sensor_msgs::NavSatFix>( "/fix", 1000 );
    
    imu_msg.header.frame_id = "imu_link";
    navSatFix_msg.header.frame_id = "midg_link";

    // Grab the port name from the launch file.
    ros::param::param<std::string>("~port", port_name, "/dev/ttyUSB0");

    /***********************************************************
    * Midg initialization
    ***********************************************************/
    int fd = Open_MIDG_Connection();

    while( ros::ok() )
    {
        Process_MIDG_Packets( fd );

        imu_msg.header.stamp = ros::Time::now();

        midgimu_pub.publish( midgimu_msg );
        imu_pub.publish( imu_msg );
        navSatFix_pub.publish( navSatFix_msg );
    }
}

int Open_MIDG_Connection()
{
    //------------------------------------------------------------------
    // Create serial socket port
    //------------------------------------------------------------------

    int fd;
    struct termios newtio;

    tcgetattr(fd, &newtio);

    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY );
    if (fd <0) {perror(port_name.c_str()); exit(-1); }

    memset( &newtio, 0x00, sizeof(newtio) );

    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | IGNBRK;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0 & !ECHO;

    /* set input mode */
    newtio.c_lflag &= ~ICANON;

    newtio.c_cc[VTIME]    = 1.0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 250; /* blocking read until 1 chars received */

    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
}


void Process_MIDG_Packets( int fd )
{
    static list<double>  GPSPV_LONG_DATA;
    static list<double>  GPSPV_LAT_DATA;

    msg_NAVPV       msg_navpv;
    msg_GPSPV       msg_gpspv;
    msg_NAVSENSE    msg_navsense;
    msg_UTCTIME     msg_utctime;
    msg_NAVHDG      msg_navhdg;
    msg_IMUMAG      msg_imumag;

    const unsigned int sample_size = 10;

    ROS_DEBUG("getonepacket");
    midgPacket msg_temp = getonepacket( fd );

    switch( msg_temp.messageID )
    {
        case MIDG_MESSAGE_NAVPVDATA:
            ROS_DEBUG("midg_message_navpvdata");
            msg_navpv = msg_temp.handle_msg_NAVPV();

            if( !msg_navpv.positionvalid )
            {
                midgimu_msg.position_valid = false;
                for(int i=0; i<9; ++i)
                    navSatFix_msg.position_covariance[i] = 0;
            }
            else
            {
		if( GPSPV_LONG_DATA.size() >= sample_size )
                {
                    GPSPV_LONG_DATA.pop_front();
                }

                GPSPV_LONG_DATA.push_back( msg_navpv.xPos );
                midgimu_msg.longitude = average( GPSPV_LONG_DATA );
                navSatFix_msg.longitude = midgimu_msg.longitude;

                if( GPSPV_LAT_DATA.size() >= sample_size )
                {
                    GPSPV_LAT_DATA.pop_front();
                }
                GPSPV_LAT_DATA.push_back( msg_navpv.yPos );
                
                //midgimu_msg.heading = atan2( GPSPV_LAT_DATA.back() - GPSPV_LAT_DATA.front(), GPSPV_LONG_DATA.back() - GPSPV_LONG_DATA.front() );

                midgimu_msg.longitude = msg_navpv.xPos;
                midgimu_msg.latitude  = msg_navpv.yPos;
                midgimu_msg.altitude  = msg_navpv.zPos;
                midgimu_msg.position_valid = true;
                
                navSatFix_msg.longitude = msg_navpv.xPos;
                navSatFix_msg.latitude  = msg_navpv.yPos;
                navSatFix_msg.altitude  = msg_navpv.zPos;
                navSatFix_msg.position_covariance[0] = FIXED_COVARIANCE_VALUE;
                navSatFix_msg.position_covariance[4] = FIXED_COVARIANCE_VALUE;
                navSatFix_msg.position_covariance[8] = FIXED_COVARIANCE_VALUE;
            }
            break;

        case MIDG_MESSAGE_GPSPVDATA:
            ROS_DEBUG("midg_message_gpspvdata");
            msg_gpspv = msg_temp.handle_msg_GPSPV();

            if( !msg_gpspv.gpsfixvalid )
            {
                midgimu_msg.position_valid = false;
                for(int i=0; i<9; ++i)
                    navSatFix_msg.position_covariance[i] = 0;
                midgimu_msg.heading_valid = false;
                midgimu_msg.position_accuracy = msg_gpspv.positionaccuracy;
            }
            else
            {
                //----------------------------------------------------
                // Average GPS position over N samples
                //----------------------------------------------------
                if( GPSPV_LONG_DATA.size() >= sample_size )
                {
                    GPSPV_LONG_DATA.pop_front();
                }
                GPSPV_LONG_DATA.push_back( msg_gpspv.GPS_PosX );
                midgimu_msg.longitude = average( GPSPV_LONG_DATA );
                navSatFix_msg.longitude = midgimu_msg.longitude;

                if( GPSPV_LAT_DATA.size() >= sample_size )
                {
                    GPSPV_LAT_DATA.pop_front();
                }
                GPSPV_LAT_DATA.push_back( msg_gpspv.GPS_PosY );
                midgimu_msg.latitude = average( GPSPV_LAT_DATA );
                navSatFix_msg.latitude = midgimu_msg.latitude;

		//midgimu_msg.heading = atan2( GPSPV_LAT_DATA.back() - GPSPV_LAT_DATA.front(), GPSPV_LONG_DATA.back() - GPSPV_LONG_DATA.front() );

                midgimu_msg.longitude = msg_gpspv.GPS_PosX;
                midgimu_msg.latitude  = msg_gpspv.GPS_PosY;
                midgimu_msg.altitude  = msg_gpspv.GPS_PosZ;
                midgimu_msg.position_valid = true;
                midgimu_msg.position_accuracy = msg_gpspv.positionaccuracy;
                
                navSatFix_msg.longitude = msg_gpspv.GPS_PosX;
                navSatFix_msg.latitude  = msg_gpspv.GPS_PosY;
                navSatFix_msg.altitude  = msg_gpspv.GPS_PosZ;
                float temp[] = {msg_gpspv.positionaccuracy, 0, 0,
                                0, msg_gpspv.positionaccuracy, 0,
                                0, 0, msg_gpspv.positionaccuracy};
                if(FIX_COVARIANCE)
                    temp[0] = FIXED_COVARIANCE_VALUE;
                    temp[4] = FIXED_COVARIANCE_VALUE;
                    temp[8] = FIXED_COVARIANCE_VALUE;
                for(int i=0; i<9; ++i)
                    navSatFix_msg.position_covariance[i] = temp[i];
            }
            break;

        case MIDG_MESSAGE_NAVSENSEDATA:
            ROS_DEBUG("midg_message_navsensedata");
            msg_navsense = msg_temp.handle_msg_NAVSENSE();

            //~ //------------------------------------------------------
            //~ // Modify yaw for robot system
            //~ //
            //~ // IN: North = 0, East = +M_PI_2, West = -M_PI_2,
            //~ //     South = +-M_PI
            //~ //
            //~ // OUT: East = 0, North = +M_PI_2, West = +M_PI,
            //~ //      South = +3*M_PI_2
            //~ //------------------------------------------------------
            //~ msg_navsense.yaw *= -1;
            //~ msg_navsense.yaw += M_PI_2;
            //~ while( msg_navsense.yaw < 0 )
            //~ {
                //~ msg_navsense.yaw += 2*M_PI;
            //~ }
            //~ while( msg_navsense.yaw > 2*M_PI )
            //~ {
                //~ msg_navsense.yaw -= 2*M_PI;
            //~ }

            //------------------------------------------------------
            // Modify angular rate for robot system (convert to radians)
            //------------------------------------------------------
            msg_navsense.xAngRate *= M_PI/180.;
            msg_navsense.yAngRate *= M_PI/180.;
            msg_navsense.zAngRate *= M_PI/180.;

            //~ midgimu_msg.heading = msg_navsense.yaw;
            midgimu_msg.angular_rate = msg_navsense.zAngRate;

            // Convert angular velocity from NED to ENU
            imu_msg.angular_velocity.x = msg_navsense.yAngRate;
            imu_msg.angular_velocity.y = msg_navsense.xAngRate;
            imu_msg.angular_velocity.z = -( msg_navsense.zAngRate );

            // Convert acceleration from NED to ENU
            imu_msg.linear_acceleration.x = msg_navsense.yAccel / g;
            imu_msg.linear_acceleration.y = msg_navsense.xAccel / g;
            imu_msg.linear_acceleration.z = -( msg_navsense.zAccel / g );

            // Convert orientation from NED to ENU
            imu_msg.orientation.x = msg_navsense.Qy;
            imu_msg.orientation.y = msg_navsense.Qx;
            imu_msg.orientation.z = -( msg_navsense.Qz );
            imu_msg.orientation.w = msg_navsense.Qw;

            break;

        case MIDG_MESSAGE_UTCTIME:
            ROS_DEBUG("midg_message_utctime");
            msg_utctime = msg_temp.handle_msg_UTCTIME();

            midgimu_msg.gps_time = ( msg_utctime.timestamp - 15 );
            navSatFix_msg.header.stamp = ros::Time( midgimu_msg.gps_time );
            break;

        case MIDG_MESSAGE_NAVHDGDATA:
            ROS_DEBUG("midg_message_navhdgdata");
            msg_navhdg = msg_temp.handle_msg_NAVHDG();
            
            //~ msg_navhdg.magHeading *= (M_PI/180);
            //~ //------------------------------------------------------
            //~ // Modify yaw for robot system
            //~ //
            //~ // IN: North = 0, East = +M_PI_2, West = -M_PI_2,
            //~ //     South = +-M_PI
            //~ //
            //~ // OUT: East = 0, North = +M_PI_2, West = +M_PI,
            //~ //      South = +3*M_PI_2
            //~ //------------------------------------------------------
            //~ msg_navhdg.magHeading *= -1;
            //~ msg_navhdg.magHeading += M_PI_2;
            //~ while( msg_navhdg.magHeading < 0 )
            //~ {
                //~ msg_navhdg.magHeading += 2*M_PI;
            //~ }
            //~ while( msg_navhdg.magHeading > 2*M_PI )
            //~ {
                //~ msg_navhdg.magHeading -= 2*M_PI;
            //~ }
 
            //~ midgimu_msg.heading = msg_navhdg.magHeading;
            midgimu_msg.speed = msg_navhdg.sog;
            break;

	case MIDG_MESSAGE_IMUMAG:
		msg_imumag = msg_temp.handle_msg_IMUMAG();
		if(reading_count <10)
		{
			avg_mag_y += msg_imumag.mag_y_sine*0.1;
			avg_mag_x += msg_imumag.mag_x_sine*0.1;
			reading_count++;
		}
		else
		{
			reading_count = 0;
			heading = atan2(avg_mag_y,avg_mag_x);
			heading *= -1;
			heading += M_PI_2;
			if(heading < 0 )
			{
				heading += 2*M_PI;
			}
			while( heading > 2*M_PI )
			{			
				heading -= 2*M_PI;
			}
			//midgimu_msg.heading = avg_mag_x;
			//midgimu_msg.gps_time = avg_mag_y;
			avg_mag_x = 0;
			avg_mag_y = 0;
			midgimu_msg.heading = heading;
		}	
		
		break;
    default:
            ROS_DEBUG("Midg default message");
        break;
    }  /* switch( msg_temp.messageID ) */
}  /* Process_MIDG_Packets() */
