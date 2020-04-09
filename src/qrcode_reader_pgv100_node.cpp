#include "ros/ros.h"
#include <SerialPort.h>
#include <thirabot_msgs/QRDetectResult.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <string>

class QRCodeReaderPGV100
{
    public:
        QRCodeReaderPGV100()
        : pnh_("~")
        {
        }

        bool init()
        {
            // get serial port name: ex) /dev/ttyUSB0
            std::string portName;
            if(!pnh_.getParam("port_name", portName))
            {
                ROS_ERROR("[%s] Failed to get port name. Please set the parameter ~port_name", ros::this_node::getName().c_str());
                return false;
            }

            // get baudrate
            int baudrate;
            if(!pnh_.getParam("baudrate", baudrate)) {
                ROS_ERROR("[%s] Failed to get baudrate. Please set the parameter ~baudrate", ros::this_node::getName().c_str());
                return false;
            }

            // libserial
            serial_port_ = boost::make_shared<SerialPort>(portName);
            serial_port_->Open();

            switch(baudrate)
            {
                case 115200:
                    serial_port_->SetBaudRate(SerialPort::BAUD_115200);
                    break;
                case 57600:
                    serial_port_->SetBaudRate(SerialPort::BAUD_57600);
                    break;
                case 19200:
                    serial_port_->SetBaudRate(SerialPort::BAUD_19200);
                    break;
                default:
                    ROS_ERROR("Failed to set baudrate. This baudrate is not support yet...");
                    return false;
            }


            serial_port_->SetParity(SerialPort::PARITY_EVEN);
            ros::Duration(0.5).sleep();

            send_type_to_scan();
            ros::Duration(0.5).sleep();
            while(true)
            {
                SerialPort::DataBuffer recv_packet(1);
                try
                {
                    serial_port_->Read(recv_packet, 1, 100);
                }
                catch (SerialPort::ReadTimeout e)
                {
                    ROS_INFO("Buffer initialized successfully");
                    break;
                }

            }

            double rate = 0.0;
            pnh_.param<double>("rate", rate, 25.0);

            // init ROS publisher
            pub_scan_result_ = nh_.advertise<thirabot_msgs::QRDetectResult>("qrcode_scan_result", 1);
            loop_timer_ = nh_.createTimer(ros::Duration(1.0/rate), &QRCodeReaderPGV100::callback, this);
            ROS_INFO("[%s] Initialized successfully.", ros::this_node::getName().c_str());
            return true;
        }

        ~QRCodeReaderPGV100()
        {
            if(serial_port_->IsOpen())
                serial_port_->Close();
        }

    private:
        void callback(const ros::TimerEvent& event)
        {
            send_request_to_scan();
            ros::Duration(0.02).sleep();

            // Receive the packet from PGV100
            SerialPort::DataBuffer recv_packet(21);
            try
            {
                serial_port_->Read(recv_packet, 21, 5);
            }
            catch (SerialPort::ReadTimeout e)
            {
                ROS_WARN("[%s] ReadTimeout occurred. check the communication line or device status.", ros::this_node::getName().c_str());
                return;
            }

            // Calculate the checksum by XOR [0:19]
            int8_t checksum = recv_packet[0];
            for(size_t i = 1; i < 20; i++)
            {
                checksum ^= recv_packet[i];
            }

            // Compare with calculated checksum and received checksum.
            if(checksum != recv_packet[20])
            {
                ROS_WARN("[%s] Checksum mismatched...", ros::this_node::getName().c_str());
                return;
            }


            // Parse and save the result from received packet
            thirabot_msgs::QRDetectResult result_msg = thirabot_msgs::QRDetectResult();

            if(recv_packet[1] & 0x40)
            {
                result_msg.is_detected = true;
                ROS_DEBUG("QR code detected.");

                int32_t xps = ((int32_t)(recv_packet[2] & 0x07) << 21 |
                                ((int32_t)(recv_packet[3] & 0x7f) << 14) |
                                ((int32_t)(recv_packet[4] & 0x7f) << 7) |
                                (recv_packet[5] & 0x7f));

                if(xps & 0x40000) // MSB is set, it is negative value
                {
                    xps |= 0xff800000;
                }
                ROS_DEBUG("XPS_: %d", xps);
                result_msg.pose.position.x = xps / 1000.0;


                int16_t yps = ((int16_t)(recv_packet[6] & 0x7f) << 7) |
                                (recv_packet[7] & 0x7f);

                if(yps & 0x2000) // MSB is set, it is negative value
                {
                    yps |= 0xC000;
                }
                ROS_DEBUG("YPS_: %d", yps);
                result_msg.pose.position.y = yps / 1000.0;


                int16_t ang = ((int16_t)(recv_packet[10] & 0x7f) << 7) |
                                (recv_packet[11] & 0x7f);

                if(ang & 0x2000) // MSB is set, it is negative value
                {
                    ang |= 0xC000;
                }
                ang = 360 - ang/2;
                ROS_DEBUG("ANG_: %d", ang);


                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, ang / 180.0 * M_PI);
                result_msg.pose.orientation.x = q[0];
                result_msg.pose.orientation.y = q[1];
                result_msg.pose.orientation.z = q[2];
                result_msg.pose.orientation.w = q[3];

                uint32_t tag = ((uint32_t)(recv_packet[14] & 0x07) << 21 |
                                ((uint32_t)(recv_packet[15] & 0x7f) << 14) |
                                ((uint32_t)(recv_packet[16] & 0x7f) << 7) |
                                (recv_packet[17] & 0x7f));

                ROS_DEBUG("TAG_: %d", tag);
                result_msg.detected_text = std::to_string(tag);
            }
            else
            {
                result_msg.is_detected = false;
                ROS_WARN("[%s]QR code not detected.", ros::this_node::getName().c_str());                
            }
            
            result_msg.header.stamp = ros::Time::now();
            pub_scan_result_.publish(result_msg);
        }

        void send_request_to_scan()
        {
            SerialPort::DataBuffer req_packet(2);
            req_packet[0] = 0xC8;
            req_packet[1] = ~req_packet[0];

            if(serial_port_->IsOpen())
            {
                serial_port_->Write(req_packet);
            }
        }

        void send_type_to_scan()
        {
            // Straight ahead       =   0xEC, 0x13
            // Follow Left          =   0xE8, 0x17
            // Follow Right         =   0xE4, 0x1B
            // No lane is selected  =   0xE0, 0x1F

            SerialPort::DataBuffer req_type(2);

            req_type[0] = 0xec;
            req_type[1] = ~req_type[0];

            if(serial_port_->IsOpen())
            {
                serial_port_->Write(req_type);
            }
        }
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        boost::shared_ptr<SerialPort> serial_port_;

        ros::Publisher pub_scan_result_;
        ros::Timer loop_timer_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrcode_reader_pgv100");

    QRCodeReaderPGV100 m;
    if(!m.init())
        return -1;

    ros::spin();

    return 0;
}