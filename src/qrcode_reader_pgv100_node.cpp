#include "ros/ros.h"
#include <SerialPort.h>
#include <thirabot_msgs/QRDetectResult.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PI  3.14159265

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

            pnh_.setParam("port_name", "/dev/ttyUSB0");
            if(!pnh_.getParam("port_name", portName))
            {   
                ROS_ERROR("[%s] Failed to get port name. Please set the parameter ~port_name", ros::this_node::getName().c_str());
                return false;
            }

            // get baudrate
            int baudrate;
            
            pnh_.setParam("baudrate", 115200);
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
            // Send request code to PGV100
            int code_num, dpos_x, dpos_y, ang_degree, ang_cnt;
            int xor_chk_count = 0;
            double ang_radian = 0.0;
            bool code_detect = false;

            char *status_bin_1, *status_bin_2 = {0, };
            char *x_bin_1, *x_bin_2, *x_bin_3, *x_bin_4 =  {0, };
            char *y_bin_1, *y_bin_2 = {0, };
            char *ag_bin_1, *ag_bin_2 = {0, };
            char *code_num_1, *code_num_2, *code_num_3, *code_num_4 = {0, };

            char x_pos[28] = {0, };
            char y_pos[14] = {0, };
            char tag_num[28] = {0, };
            char ag_pos[14] = {0,};

            send_request_to_scan();
            ros::Duration(0.02).sleep();
            // Receive the packet from PGV100
            SerialPort::DataBuffer recv_packet(21);
            try
            {
                serial_port_->Read(recv_packet, 21, 5);
                /*
                for(size_t i = 0; i < recv_packet.size(); i++)
                {
                    ROS_INFO("Hex[%d] = %ld", i, recv_packet[i]);
                }
                printf("\n");
                */
            }
            catch (SerialPort::ReadTimeout e)
            {
                ROS_WARN("[%s] ReadTimeout occurred. check the communication line or device status.", ros::this_node::getName().c_str());
                return;
            }

            // Parse the received packet
            //check data by XOR value
            int chk_data = 0;
            while(true)
            {
                chk_data = chk_data ^ recv_packet[xor_chk_count];

                if(xor_chk_count == 19){
                    xor_chk_count = 0;
                    printf("\n");
                    break;                    
                }
                xor_chk_count++;    
            }
            if(chk_data == recv_packet[20])
            { 
                status_bin_2 = intToBinary(recv_packet[1]);
                if(status_bin_2[0] == '1')
                    code_detect = true;

                ROS_INFO("code_detect = %s", code_detect ? "true" : "false");

                x_bin_1 = intToBinary(recv_packet[2]);     
                strcat(x_pos, x_bin_1);    
                x_bin_2 = intToBinary(recv_packet[3]);
                strcat(x_pos, x_bin_2); 
                x_bin_3 = intToBinary(recv_packet[4]);
                strcat(x_pos, x_bin_3); 
                x_bin_4 = intToBinary(recv_packet[5]);
                strcat(x_pos, x_bin_4);

                dpos_x = binaryToInt(x_pos);
                if(dpos_x > 2000.0)                         // Use for mark up (-) value
                    dpos_x = dpos_x - pow(2,24) - 1;

                ROS_INFO("dpos_x = %d", dpos_x);

                y_bin_1 = intToBinary(recv_packet[6]);
                strcat(y_pos, y_bin_1);
                y_bin_2 = intToBinary(recv_packet[7]);
                strcat(y_pos, y_bin_2);                  

                dpos_y = binaryToInt(y_pos);
                if(dpos_y > 2000.0)                         // Use for mark up (-) value
                    dpos_y = dpos_y - pow(2,14) - 1;

                ROS_INFO("dpos_y = %d", dpos_y);

                ag_bin_1 = intToBinary(recv_packet[10]);
                strcat(ag_pos, ag_bin_1);
                ag_bin_2 = intToBinary(recv_packet[11]);
                strcat(ag_pos, ag_bin_2);
                ang_degree = binaryToInt(ag_pos);

                ang_degree = ang_degree / 2;
                if(ang_degree == 0.0)
                    ang_degree = 0.0;      
                else if(90.0 > ang_degree && ang_degree > 0.0){
                    ang_cnt = 180.0 - ang_degree;
                    ang_degree = ang_degree + (2 * ang_cnt);
                } 
                else if(180.0 > ang_degree && ang_degree >= 90.0){
                    ang_cnt = 180.0 - ang_degree;
                    ang_degree = ang_degree + (2 * ang_cnt);
                }
                else if(270.0 > ang_degree && ang_degree >= 180.0){
                    ang_cnt = ang_degree - 180.0;
                    ang_degree = ang_degree - (2 * ang_cnt);  
                }     
                else if(360.0 > ang_degree && ang_degree >= 270.0){
                    ang_cnt = ang_degree - 180.0;
                    ang_degree = ang_degree - (2 * ang_cnt);
                }
                //degree to radian
                ang_radian = ang_degree * (PI/180.0);
                ROS_INFO("ang_degree = %d", ang_degree);
                ROS_INFO("ang_radian = %f", ang_radian);

                code_num_1 = intToBinary(recv_packet[14]);
                strcat(tag_num, code_num_1);                
                code_num_2 = intToBinary(recv_packet[15]);
                strcat(tag_num, code_num_2);                
                code_num_3 = intToBinary(recv_packet[16]);
                strcat(tag_num, code_num_3);                
                code_num_4 = intToBinary(recv_packet[17]);
                strcat(tag_num, code_num_4);    

                code_num = binaryToInt(tag_num);
                ROS_INFO("code_number = %d", code_num); 
            }
            else
            {
                ROS_WARN("Invalid data value");
                return;
            }
        }

        char *intToBinary(int i) 
        {
            static char s[7 + 1] = {'0', };
            int count = 7;
            do 
            {
                s[--count] = '0' + (char)(i & 1);
                i = i >> 1;
            } while (count);

            return s;
        }

        int binaryToInt(char *s) 
        {
            int i = 0;
            int count = 0;

            while (s[count])
                i = (i << 1) | (s[count++] - '0');

            return i;
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

        void send_type_to_scan()
        {
            // Straight ahead       =   0xEC, 0x13
            // Follow Left          =   0xE8, 0x17
            // Follow Right         =   0xE4, 0x1B
            // No lane is selected  =   0xE0, 0x1F

            SerialPort::DataBuffer req_type(2);

            req_type[0] = 0xEC;
            req_type[1] = 0x13;

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