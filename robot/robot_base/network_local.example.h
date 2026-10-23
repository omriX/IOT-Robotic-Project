#pragma once

// Copy to network_local.h (ignored by Git). Use a 2.4 GHz Wi-Fi network.
constexpr char WIFI_SSID[] = "YOUR_WIFI_NAME";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
// The agent computer's Wi-Fi IPv4 address, not the robot's address.
constexpr char MICROROS_AGENT_IP[] = "YOUR_PC_WIFI_IP";

// Uncomment to talk to the micro-ROS agent over the USB cable instead of Wi-Fi
// (agent-serial, Linux only). Wi-Fi above is then used for WebSerial only.
// #define MICROROS_TRANSPORT_SERIAL 1
