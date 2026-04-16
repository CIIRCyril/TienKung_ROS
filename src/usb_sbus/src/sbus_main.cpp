#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <thread>
#include <sensor_msgs/msg/joy.hpp>
#include <mutex>
#include <math.h>
#include "usbSbus.h"

#define SBUS_USB_SERIAL_DEV "/dev/ttyUSB0"

#define DF_HC_MAX_LINE_SPEED (2.0 )          // m/s
#define DF_HC_MAX_ANGLE_SPEED (0.5)          // rad/s

#define JOY_MIN_VAL     282
#define JOY_MAX_VAL     1722
#define JOY_NORAM_VAL   1002


const float EPSINON = 1e-6;
#define Equ(a,b) ((fabs((a) - (b)) < (EPSINON)))

usbSbus sbusHandle_;
rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher;
bool bRunning=true;

std::mutex data_locker;
float fAxes_0,fAxes_1;
double axes_time;
unsigned int joy_seq=0;

int joy_now_ = 0;

bool bJoyEnable_ = false;

static rclcpp::Logger get_logger() {
    return rclcpp::get_logger("usb_sbus_node");
}

float get_axes_val(uint16_t nVal)
{
    float fData=0.0;
    if(nVal < JOY_NORAM_VAL)
        fData = (JOY_NORAM_VAL - nVal) * 1.0 / (JOY_NORAM_VAL - JOY_MIN_VAL);
    else if(nVal > JOY_NORAM_VAL)
        fData = (JOY_NORAM_VAL - nVal) * 1.0 / (JOY_MAX_VAL - JOY_NORAM_VAL);

    return (fData * (-1.0));
}

int ch5_button_check(uint16_t nVal)
{
    int nResult = JOY_NORAM_VAL - nVal;
    if(nResult>280)
    {
        RCLCPP_INFO(get_logger(), "joy control switch on!!!");
        bJoyEnable_ = true;
    }
    else
    {
        bJoyEnable_ = false;
    }
    return 0;
}

void recvThreadFunc(rclcpp::Node::SharedPtr node)
{
    uint16_t channels_in[16];
    float fAxes;
    RCLCPP_INFO(get_logger(), "sbus read thread begin.");

    bool bEqu=false;

    uint16_t in_change[16];

    while (bRunning && rclcpp::ok())
    {
        int nRet = sbusHandle_.sbus_read(channels_in);
        if(nRet!=16)
        {
            usleep(10*1000);
            continue;
        }
        sensor_msgs::msg::Joy joy;
        bool bHasPress = false;
        joy.header.stamp = node->get_clock()->now();

        joy.axes.resize(12);
        joy.buttons.resize(0);
        for(int i=0;i < 12;i++)
        {
            joy.axes[i] = get_axes_val(channels_in[i]);
            if(in_change[i] != channels_in[i])
                bHasPress = true;
            in_change[i] = channels_in[i];
        }
        if(bHasPress == true)
        {
            RCLCPP_INFO(get_logger(), "get sbus packet:");
            for (int ch=0; ch < 16; ch++)
            {
                RCLCPP_INFO(get_logger(), "channel[%d] = %d    failsafe = %d", ch, channels_in[ch], sbusHandle_.sbus_failsafe());
            }
            RCLCPP_INFO(get_logger(), "---------------");
        }
        joy_publisher->publish(joy);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("usb_sbus_node");
    joy_publisher = node->create_publisher<sensor_msgs::msg::Joy>("/sbus_data", 10);

    bool bRet = sbusHandle_.init(SBUS_USB_SERIAL_DEV);
    if(bRet == false)
        return -1;

    std::thread sbus_thrd(recvThreadFunc, node);

    rclcpp::Rate r_10HZ(10);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        r_10HZ.sleep();
    }

    bRunning = false;

    sbus_thrd.join();

    rclcpp::shutdown();
    return 0;
}
