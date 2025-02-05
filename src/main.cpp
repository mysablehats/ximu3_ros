// This file was generated by Examples/Cpp/generate_main.py

#include "ximu3_ros/BluetoothConnection.h"
#include "ximu3_ros/Commands.h"
#include "ximu3_ros/DataLogger.h"
#include "ximu3_ros/FileConverter.h"
#include "ximu3_ros/GetAvailablePorts.h"
#include "ximu3_ros/NetworkDiscovery.h"
#include "ximu3_ros/OpenAndPing.h"
#include "ximu3_ros/SerialConnection.h"
#include "ximu3_ros/SerialDiscovery.h"
#include "ximu3_ros/TcpConnection.h"
#include "ximu3_ros/UdpConnection.h"
#include "ximu3_ros/UsbConnection.h"
#include "ximu3_ros/Helpers.hpp"
#include <iostream>

int main(int argc, const char* argv[])
{
    setbuf(stdout, NULL);
    std::cout << "Select example " << std::endl;
    std::cout << "A. BluetoothConnection.h" << std::endl;
    std::cout << "B. Commands.h" << std::endl;
    std::cout << "C. DataLogger.h" << std::endl;
    std::cout << "D. FileConverter.h" << std::endl;
    std::cout << "E. GetAvailablePorts.h" << std::endl;
    std::cout << "F. NetworkDiscovery.h" << std::endl;
    std::cout << "G. OpenAndPing.h" << std::endl;
    std::cout << "H. SerialConnection.h" << std::endl;
    std::cout << "I. SerialDiscovery.h" << std::endl;
    std::cout << "J. TcpConnection.h" << std::endl;
    std::cout << "K. UdpConnection.h" << std::endl;
    std::cout << "L. UsbConnection.h" << std::endl;
    switch (helpers::getKey())
    {
        case 'A':
            BluetoothConnection();
            break;
        case 'B':
            Commands();
            break;
        case 'C':
            DataLogger();
            break;
        case 'D':
            FileConverter();
            break;
        case 'E':
            GetAvailablePorts();
            break;
        case 'F':
            NetworkDiscovery();
            break;
        case 'G':
            OpenAndPing();
            break;
        case 'H':
            SerialConnection();
            break;
        case 'I':
            SerialDiscovery();
            break;
        case 'J':
            TcpConnection();
            break;
        case 'K':
            UdpConnection();
            break;
        case 'L':
            UsbConnection();
            break;
    }
}
