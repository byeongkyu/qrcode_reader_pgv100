#include "ros/ros.h"
#include <libserial/SerialPort.h>
#include <thirabot_msgs/QRDetectResult.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <string>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>


#include "angles/angles.h"

class QRCodeReaderPGV100
{
    public:
        QRCodeReaderPGV100()
        {
        }

        bool init()
        {
            ros::NodeHandle pnh_("~");

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

            pnh_.param<int>("angle_offset", angle_offset_, 0);
            ROS_INFO("[%s] Set angle offset to %d", ros::this_node::getName().c_str(), angle_offset_);

            // libserial
            serial_port_ = boost::make_shared<LibSerial::SerialPort>();
            serial_port_->Open(portName);

            switch(baudrate)
            {
                case 115200:
                    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                    break;
                case 57600:
                    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_57600);
                    break;
                case 19200:
                    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_19200);
                    break;
                default:
                    ROS_ERROR("Failed to set baudrate. This baudrate is not support yet...");
                    return false;
            }

            serial_port_->SetParity(LibSerial::Parity::PARITY_EVEN);
            serial_port_->FlushIOBuffers();
            ros::Duration(0.5).sleep();

            send_type_to_scan();
            serial_port_->FlushIOBuffers();

            double rate = 0.0;
            pnh_.param<double>("rate", rate, 50.0);

            // init ROS publisher
            pub_scan_result_ = nh_.advertise<thirabot_msgs::QRDetectResult>("qrcode_scan_result", 10);
            pub_diagnostic_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
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

            // Receive the packet from PGV100
            LibSerial::DataBuffer recv_packet(21);
            try
            {
                serial_port_->Read(recv_packet, 21, 100);
            }
            catch (LibSerial::ReadTimeout e)
            {
                ROS_WARN("[%s] ReadTimeout occurred. check the communication line or device status.", ros::this_node::getName().c_str());
                serial_port_->FlushIOBuffers();
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
                serial_port_->FlushIOBuffers();
                checksum_state_ = false;
                return;
            }
            checksum_state_ = true;

            // Parse and save the result from received packet
            thirabot_msgs::QRDetectResult result_msg = thirabot_msgs::QRDetectResult();

            if(recv_packet[1] & 0x40)
            {
                scan_detected_ = true;
                result_msg.is_detected = scan_detected_;
                ROS_DEBUG("QR code detected.");

                int32_t xps = ((int32_t)(recv_packet[2] & 0x07) << 21 |
                                ((int32_t)(recv_packet[3] & 0x7f) << 14) |
                                ((int32_t)(recv_packet[4] & 0x7f) << 7) |
                                (recv_packet[5] & 0x7f));

                if(xps & 0x40000) // MSB is set, it is negative value
                {
                    xps |= 0xff800000;
                }
                ROS_DEBUG("XPS_: %d", -1 * xps);
                scan_pos_x_ = (xps / 10.0) / 1000.0;
                result_msg.pose.position.x = scan_pos_x_;

                int16_t yps = ((int16_t)(recv_packet[6] & 0x7f) << 7) |
                                (recv_packet[7] & 0x7f);

                if(yps & 0x2000) // MSB is set, it is negative value
                {
                    yps |= 0xC000;
                }
                ROS_DEBUG("YPS_: %d", -1 * yps);
                scan_pos_y_ = (yps / 10.0) / 1000.0;
                result_msg.pose.position.y = scan_pos_y_;


                int16_t ang = ((int16_t)(recv_packet[10] & 0x7f) << 7) |
                                (recv_packet[11] & 0x7f);

                if(ang & 0x2000) // MSB is set, it is negative value
                {
                    ang |= 0xC000;
                }
                ang = ang - angle_offset_;
                scan_angle_ = angles::normalize_angle((ang / 10.0) / 180.0 * M_PI);
                ROS_DEBUG("ANG_: %d", ang - angle_offset_);


                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, scan_angle_);
                result_msg.pose.orientation.x = q[0];
                result_msg.pose.orientation.y = q[1];
                result_msg.pose.orientation.z = q[2];
                result_msg.pose.orientation.w = q[3];

                uint32_t tag = ((uint32_t)(recv_packet[14] & 0x07) << 21 |
                                ((uint32_t)(recv_packet[15] & 0x7f) << 14) |
                                ((uint32_t)(recv_packet[16] & 0x7f) << 7) |
                                (recv_packet[17] & 0x7f));

                ROS_DEBUG("TAG_: %d", tag);
                scan_code_num_ = tag;
                result_msg.detected_text = std::to_string(scan_code_num_);
            }
            else
            {
                scan_detected_ = false;
                result_msg.is_detected = scan_detected_;
                scan_pos_x_ = 0;
                scan_pos_y_ = 0;
                scan_angle_ = 0;
                scan_code_num_ = 0;

                ROS_DEBUG("[%s]QR code not detected.", ros::this_node::getName().c_str());
            }

            result_msg.header.stamp = ros::Time::now();
            update_diagnostics();

            pub_scan_result_.publish(result_msg);
        }

        void send_request_to_scan()
        {
            LibSerial::DataBuffer req_packet(2);
            req_packet[0] = 0xC8;
            req_packet[1] = ~req_packet[0];

            if(serial_port_->IsOpen())
            {
                serial_port_->Write(req_packet);
                serial_port_->DrainWriteBuffer();
            }
        }

        void send_type_to_scan()
        {
            // Straight ahead       =   0xEC, 0x13
            // Follow Left          =   0xE8, 0x17
            // Follow Right         =   0xE4, 0x1B
            // No lane is selected  =   0xE0, 0x1F

            LibSerial::DataBuffer req_type(2);

            req_type[0] = 0xec;
            req_type[1] = ~req_type[0];

            if(serial_port_->IsOpen())
            {
                serial_port_->Write(req_type);
                serial_port_->DrainWriteBuffer();
            }
        }

        void update_diagnostics()
        {
            thirabot_msgs::QRDetectResult result_msg = thirabot_msgs::QRDetectResult();

            diagnostic_msgs::DiagnosticArray dia_array;
            diagnostic_msgs::DiagnosticStatus scan_status;
            diagnostic_msgs::KeyValue scan_result;

            std::string str_is_detected = std::to_string(scan_detected_);
            std::string str_pos_x = std::to_string(scan_pos_x_);
            std::string str_pos_y = std::to_string(scan_pos_y_);
            std::string str_angle = std::to_string(scan_angle_ * 180 / M_PI);
            std::string str_code_num = std::to_string(scan_code_num_);
            std::string str_checksum = std::to_string(checksum_state_);

            scan_status.level = diagnostic_msgs::DiagnosticStatus::OK;
            scan_status.message = "Running";

            scan_status.hardware_id = "pgv100";
            scan_status.name = "qrcode_reader_pgv100_node";

            scan_result.key = "code number";
            scan_result.value = str_code_num;
            scan_status.values.push_back(scan_result);

            scan_result.key = "code detected";
            scan_result.value = str_is_detected;
            scan_status.values.push_back(scan_result);

            scan_result.key = "pos_x";
            scan_result.value = str_pos_x;
            scan_status.values.push_back(scan_result);

            scan_result.key = "pos_y";
            scan_result.value = str_pos_y;
            scan_status.values.push_back(scan_result);

            scan_result.key = "angle";
            scan_result.value = str_angle;
            scan_status.values.push_back(scan_result);

            scan_result.key = "check sum";
            scan_result.value = str_checksum;
            scan_status.values.push_back(scan_result);

            dia_array.status.push_back(scan_status);
            pub_diagnostic_.publish(dia_array);
        }

    private:
        ros::NodeHandle nh_;
        boost::shared_ptr<LibSerial::SerialPort> serial_port_;

        ros::Publisher pub_scan_result_;
        ros::Publisher pub_diagnostic_;
        ros::Timer loop_timer_;
        
        bool scan_detected_;
        bool checksum_state_;
        double scan_pos_x_;
        double scan_pos_y_;
        double scan_angle_;
        int angle_offset_;        
        uint32_t scan_code_num_;
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