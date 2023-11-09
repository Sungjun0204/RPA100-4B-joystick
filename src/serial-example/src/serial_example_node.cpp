/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>


//  global variables
int switch_num = 0;


serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void switch_callback(const std_msgs::Int16::ConstPtr& msg) {
    switch_num = msg->data;
}





int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub1 = nh.subscribe("write", 1000, write_callback);
    ros::Subscriber write_sub2 = nh.subscribe("packet", 1000, switch_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        int packet_size;
        char packet_on[9] = {0,};
        char packet_off[9] = {0,};
        char acm_check = 0x06;

        packet_on[0] = 0x02;   // -STX-
        packet_on[1] = 0xFF;   // -Dummy-
        packet_on[2] = 0x44;   // D
        packet_on[3] = 0x42;   // B
        packet_on[4] = 0x30;
        packet_on[5] = 0x31;
        packet_on[6] = 0x03;   // -EXT-
        packet_on[7] = 0xF8;   // LGT
        packet_on[8] = acm_check;

        packet_off[0] = 0x02;   // -STX-
        packet_off[1] = 0xFF;   // -Dummy-
        packet_off[2] = 0x44;   // D
        packet_off[3] = 0x42;   // B
        packet_off[4] = 0x30;
        packet_off[5] = 0x30;
        packet_off[6] = 0x03;   // -EXT-
        packet_off[7] = 0xF9;   // LGT
        packet_off[8] = acm_check;

        if (switch_num == 1) ser.write(packet_off); 
        else if (switch_num == 2) ser.write(packet_on);
        else ser.write("");

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            
            //for(int i=0; i < sizeof(result.data); i++)
                //printf("%#x\n", result.data[i]);
            
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

