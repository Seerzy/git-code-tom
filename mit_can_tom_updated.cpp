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
volatile sig_atomic_t interrupted = 0;
float delay_in_req = 0.02;
using namespace std;
int s_int; // Declare s as a global variable 

float p;
float v;
float t;

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
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
// Function to pack command data into a CAN frame.
void pack_cmd(struct can_frame &frame, float p_des, float v_des, float kp, float kd, float t_ff) {
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
    // cout<<t_int<<endl;

    // Pack ints into the CAN frame.
    frame->can_id = 0x1; // Set your CAN ID here
    frame->can_dlc = 8;   // 8 bytes of data

    frame->data[0] = p_int >> 8; // Position 8 higher
    frame->data[1] = p_int & 0x00FF; // Position 8 lower
    frame->data[2] = v_int >> 4; // Speed 8 higher
    frame->data[3] = ((v_int & 0x00F) << 4) | (kp_int >> 8); // Speed 4 bits lower, KP 4 bits higher
    frame->data[4] = kp_int & 0x0FF;
    frame->data[5] = kd_int >> 4;
    frame->data[6] = ((kd_int & 0x00F) << 4) | (t_int >> 8); // KD 4 bits lower, T 8 bits higher
    frame->data[7] = t_int & 0x0FF;
    // Additional packing logic...
}

void unpack_reply(const struct can_frame frame) {
    int id = frame.data[0];
    int position_uint = (frame.data[1] << 8) | frame.data[2];
    int velocity_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
    int torque_uint = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    cout << position_uint << endl;
    p = uint_to_float(position_uint,P_MIN,P_MAX,16);
    v = uint_to_float(velocity_uint,V_MIN,V_MAX,12);
    t = uint_to_float(torque_uint,T_MIN,T_MAX,12);
}

// Function to initialize the CAN socket.
int canInit() {
    // set the orginal can0 down and step up can communication with a bitrate of 1000000
    int result_down = system("sudo ip link set can0 down");
    sleep(0.01);
    int result_up = system("sudo ip link set can0 up type can bitrate 1000000");

    int s = socket(AF_CAN, SOCK_RAW, CAN_RAW);
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
void powerOn(int s) {
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
    write(s, &frame, sizeof(struct can_frame));
}

void idleMotor(int s){
    struct can_frame frame;
    pack_cmd(&frame, 0, 0, 0, 0, 0);
    write(s, &frame, sizeof(struct can_frame));
    // std::cout<<"the size of frame"<<sizeof(struct can_frame)<<std::endl;
    sleep(1);
}

void powerOff(int s){
    struct can_frame frame;
   
    frame.can_id = 0x1; // Set the CAN ID for power-on message
    frame.can_dlc = 8;   // 8 bytes of data

    
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFD;

    // Send the power-off message
    write(s, &frame, sizeof(struct can_frame));
    sleep(1);
    std::cout<< "The final position of this motor is " << p << v  << t << std::endl;
}


void setOutputTorque(int s, float torque) {
    struct can_frame frame;

    //set torque to the pack message and do the mapping
    pack_cmd(&frame, 0, 0, 0, 0, torque);

    // Send the frame using the CAN socket.
    write(s, &frame, sizeof(struct can_frame));
}

void signalHandler(int signal) {
    interrupted = 1;
    std::cout<<"The motor is stopped" << std::endl;
    idleMotor(s_int);
    powerOff(s_int);
    std::cout<<"The end of this script" << std::endl; 
    close(s_int);
    sleep(1);
    exit(signal);
}

void setOutputAngleRadians(int s,float pos, float kp = 15, float kd = 0.5){
    struct can_frame frame;

    pack_cmd(frame, pos, 0, kp, kd, 0); //the data is position, velocity, kp, kd and torque

    // Send the frame using the CAN socket.
    write(s, &frame, sizeof(struct can_frame));
}

void setZeroPosition(int s){
    struct can_frame frame;

    frame.can_id = 0x1; // Set the CAN ID 
    frame.can_dlc = 8;   // 8bytes of data

    // Fill the frame with the zero position message (0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xfc)
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFE;

    // Send the power-on message
    write(s, &frame, sizeof(struct can_frame));
}

float getOutputTorque(int s){
    struct can_frame frame;
    ssize_t bytesRead = read(s, &frame, sizeof(struct can_frame));
    unpack_reply(frame);
    return t;
}

float getOutputAngleRadian(int s){
    struct can_frame frame;
    ssize_t bytesRead = read(s, &frame, sizeof(struct can_frame));
    unpack_reply(frame);
    return p;
}

void canListenerThread(int s) {
    struct can_frame frame;
    while (!interrupted) {
        ssize_t bytesRead = read(s, &frame, sizeof(struct can_frame));
        if (bytesRead > 0) {
            unpack_reply(frame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void getOutput(int s){
    struct can_frame frame;
    ssize_t bytesRead = read(s, &frame, sizeof(struct can_frame));
    unpack_reply(frame);
}

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}


int main() {
    s_int = canInit(); // this function is to initalize the CAN

    if (s_int == -1) {
        std::cout << "The Can initialization failed" << std::endl;
        return 1; // Exit with an error code
    }

    std::signal(SIGINT, signalHandler); //define the signalHandler when interrrupt receive, trigger the signalHandler function

    // Power on the device (if needed)
    powerOn(s_int);
    // idleMotor(s);
    sleep(1);
    // Send a message to the motor with a specific motor ID
    int motor_id = 1; // Set the motor ID

    setZeroPosition(s_int);

    sleep(1);
    // std::thread listener(canListenerThread, s);

    // setOutputTorque(s,1);
    // for (int counter = 0; counter < 10000; counter++){
    //     setOutputTorque(s,-1.5);
    //     // setOutputTorque(s,-2.33);
    //     getOutput(s);
    //     cout << "The motor torque now is " << t << endl;
    //     cout << "The motor position now is " << radiansToDegrees(p) << endl;
    //     sleep(0.01);
    // }
    // idleMotor(s);
    // powerOff(s);
    // sleep(1);
    // cout << "The motor is stopped and exit the control mode";
    // return 0;

    while (true){
        // setOutputTorque(s,-2.33);
        setOutputTorque(s_int,-1.);
        getOutput(s_int);
        // float position = getOutputAngleRadian(s);
        std::cout << "The motor torque now is " << t <<std::endl;
        std::cout << "The motor position now is " << radiansToDegrees(p) <<std::endl;
        sleep(0.01);
    }
    close(s_int);
    return 0;

}