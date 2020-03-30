#include "ros/ros.h"
#include <SerialPort.h>
#include <thirabot_msgs/QRDetectResult.h>

class QRCodeReaderPGV100
{
    public:
        QRCodeReaderPGV100()
        : pnh_("~")
        {
        }

        bool init()
        {
            std::string portName;
            // get serial port name: ex) /dev/ttyUSB0
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

            double rate = 0.0;
            pnh_.param<double>("rate", rate, 10.0);

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
            // Send request code to PGV100
            send_request_to_scan();

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

            // Parse the received packet





        }

        void send_request_to_scan()
        {
            SerialPort::DataBuffer req_packet(2);
            req_packet[0] = 0xC8;
            req_packet[1] = 0x37;

            if(serial_port_->IsOpen())
            {
                serial_port_->Write(req_packet);
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