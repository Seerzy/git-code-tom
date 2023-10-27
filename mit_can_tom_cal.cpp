#include <iostream>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <cmath>
#include <csignal> // Include for signal handling
#include <chrono>
#include <thread>
#include <cstdlib>  // Include the necessary header for system()
float torque_input = 0;

volatile sig_atomic_t interrupted = 0;
float delay_in_req = 0.02;
int s; // Declare s as a global variable 

float p = 0;
float v = 0;
float t = 0;

float P_MIN = -12.5;
float P_MAX = 12.5;
float V_MIN = -10.0;
float V_MAX = 10.0;
float T_MIN = -90.0;
float T_MAX = 90.0;
float Kp_MIN = 0.0;
float Kp_MAX = 500.0;
float Kd_MIN = 0.0;
float Kd_MAX = 5.0;

// Function to convert a float to an unsigned int based on a specified range and number of bits.
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)(1 << bits) / span));
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    float x = (float)x_int / ((float)(1 << bits) / span) + offset;
    return x;
}

// Function to pack command data into a CAN frame.
void pack_cmd(struct can_frame *frame, float p_des, float v_des, float kp, float kd, float t_ff) {
    // Define the minimum and maximum bounds for your parameters.

    // Limit the data to be within bounds.
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

    // Convert floats to unsigned ints using the specified bit lengths.
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    // Pack ints into the CAN frame.
    frame->can_id = 0x1; // Set your CAN ID here
    frame->can_dlc = 8;   // 8 bytes of data

    frame->data[0] = p_int >> 8; // Position 8 higher
    frame->data[1] = p_int & 0xFF; // Position 8 lower
    frame->data[2] = v_int >> 4; // Speed 8 higher
    frame->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed 4 bits lower, KP 4 bits higher
    frame->data[4] = kp_int & 0xFF;
    frame->data[5] = kd_int >> 4;
    frame->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KD 4 bits lower, T 8 bits higher
    frame->data[7] = t_int & 0xFF;
    // Additional packing logic...
}

void unpack_reply(const struct can_frame frame) {
    int id = frame.data[0];
    int position_uint = (frame.data[1] << 8) | frame.data[2];
    int velocity_uint = ((frame.data[3] << 8) | (frame.data[4] >> 4)) & 0xFFF;
    int torque_uint = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    p = uint_to_float(position_uint,P_MIN,P_MAX,16);
    v = uint_to_float(velocity_uint,V_MIN,V_MAX,12);
    t = uint_to_float(torque_uint,T_MIN,T_MAX,12);
}

// Function to initialize the CAN socket.
int canInit() {
    // set the orginal can0 down and step up can communication with a bitrate of 1000000
    //TODO: why we disconnecting first
    int result_down = system("sudo ip link set can0 down");
    int result_up = system("sudo ip link set can0 up type can bitrate 1000000");

    s = socket(AF_CAN, SOCK_RAW, CAN_RAW);

    if (s == -1) {
        perror("Socket creation error");
        return -1; // Return an error code
    }

    // Specify the CAN interface (e.g., "can0")
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
        perror("ioctl error");
        close(s);
        return -1; // Return an error code
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("Bind error");
        close(s);
        return -1; // Return an error code
    }

    return s; // Return the socket file descriptor
}

// Function to power on the device.
void powerOn(const int& s_in) {
    struct can_frame frame;

    // Create a CAN frame with the power-on message
    frame.can_id = 0x1; // Set the CAN ID for power-on message
    frame.can_dlc = 8;   // 9 bytes of data

    // Fill the frame with the power-on message (0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xfc)
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFC;

    // Send the power-on message
    write(s_in, &frame, sizeof(struct can_frame));
}

void idleMotor(const int& s_in){
    struct can_frame frame;
    pack_cmd(&frame, 0.0, 0.0, 0.0, 0.0, 0.0);
    write(s_in, &frame, sizeof(struct can_frame));
    // std::cout<<"the size of frame"<<sizeof(struct can_frame)<<std::endl;
    sleep(0.01);
}

void powerOff(const int& s_in){
    struct can_frame frame;
   
    frame.can_id = 0x1; // Set the CAN ID for power-on message
    frame.can_dlc = 8;   // 8 bytes of data

    
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFD;

    // Send the power-off message
    write(s_in, &frame, sizeof(struct can_frame));
    sleep(1);
    // std::cout<< "The final position of this motor is " << p << v  << t << std::endl;
}


void setOutputTorque(const int& s_in, float torque) {
    struct can_frame frame;

    //set torque to the pack message and do the mapping
    pack_cmd(&frame, 0, 0, 0, 0, torque);

    // Send the frame using the CAN socket.
    write(s_in, &frame, sizeof(struct can_frame));
}

void handleInterrupt(int signal) {
    torque_input = 0;
    interrupted = 1;
    std::cout<<"The motor is stopped" << std::endl;
    idleMotor(s);
    sleep(1);
    powerOff(s);
    sleep(1);
    std::cout<<"The end of this script" << std::endl; 
    close(s);
    sleep(1);
    exit(signal);
}

void setOutputAngleRadians(const int& s_in,float pos, float kp = 15, float kd = 0.5){
    struct can_frame frame;

    pack_cmd(&frame, pos, 0, kp, kd, 0); //the data is position, velocity, kp, kd and torque

    // Send the frame using the CAN socket.
    write(s_in, &frame, sizeof(struct can_frame));
}

void setZeroPosition(const int& s_in){
    struct can_frame frame;

    frame.can_id = 0x1; // Set the CAN ID 
    frame.can_dlc = 8;   // 8bytes of data

    // Fill the frame with the power-on message (0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xfc)
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFE;

    // Send the power-on message
    write(s_in, &frame, sizeof(struct can_frame));
}

float getOutputTorque(const int& s_in){
    struct can_frame frame;
    ssize_t bytesRead = read(s_in, &frame, sizeof(struct can_frame));
    unpack_reply(frame);
    return t;
}

float getOutputAngleRadian(const int& s_in){
    struct can_frame frame;
    ssize_t bytesRead = read(s_in, &frame, sizeof(struct can_frame));
    unpack_reply(frame);
    return p;
}

void canListenerThread(const int& s_in) {
    struct can_frame frame;
    while (!interrupted) {
        ssize_t bytesRead = read(s_in, &frame, sizeof(struct can_frame));
        if (bytesRead > 0) {
            unpack_reply(frame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void getOutput(const int& s_in){
    struct can_frame frame;
    ssize_t bytesRead = read(s_in, &frame, sizeof(struct can_frame));
    unpack_reply(frame);
}

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}


int main() {

    std::cout <<"CAN is initalized" <<std::endl;

    s = canInit(); // this function is to initalize the CAN

    if (s == -1) {
        std::cout << "The Can initialization failed" << std::endl;
        return 1; // Exit with an error code
    }
    std::cout <<"Signal Handler is initalized" <<std::endl;
    signal(SIGINT,handleInterrupt);


    // Power on the device (if needed)
    std::cout <<"Motor is powered on" <<std::endl;
    powerOn(s);
    idleMotor(s);
    // sleep(1);
    // Send a message to the motor with a specific motor ID

    getOutput(s);
    std::cout << "The position " << p << " The velocity" << v << " The Torque" << t << std::endl; 
    std::cout <<"Motor is set to zero position " <<std::endl;
    setZeroPosition(s);

    sleep(1);
    // std::thread listener(canListenerThread, s);

    // setOutputTorque(s,1);
    torque_input = -2.33;
    while(true){
        if (interrupted != 1){
            setOutputTorque(s,torque_input);
        }

        // setOutputTorque(s,-1.33);
        getOutput(s);
        // float position = getOutputAngleRadian(s);
        std::cout << "The motor torque now is " << t <<std::endl;
        std::cout << "The motor position now is " << radiansToDegrees(p) <<std::endl;
        sleep(0.1);
    }
    

    return 0;

}