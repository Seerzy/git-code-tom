#include <chrono>
#include <cmath>
#include <csignal> // Include for signal handling
#include <cstdlib> // Include the necessary header for system()
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using namespace std; // Add this using directive

class CanMotorController {
public:
  volatile sig_atomic_t interrupted = 0;

  CanMotorController() : socket_can(canInit()) {
    if (socket_can == -1) {
      std::cout << "CAN initalization failed" << std::endl;
    }
    powerOn();
    setZeroPosition();
    sleep(1);
  }

  ~CanMotorController() {
    idleMotor();
    powerOff();
    close(socket_can);
    sleep(1);
  }

  void setTorque(float torque) {
    setOutputTorque(torque);
    updateMotorState();
  }

  float getTorque() { return output_torque; }

  float getPositionDegrees() { return this->output_position * (180.0 / M_PI); }

  float getPositionRad() { return this->output_position; }

private:
  const int socket_can;
  float output_position;
  float output_velocity;
  float output_torque;

  const float P_MIN = -12.5;
  const float P_MAX = 12.5;
  const float V_MIN = -10.0;
  const float V_MAX = 10.0;
  const float T_MIN = -90.0;
  const float T_MAX = 90.0;
  const float Kp_MIN = 0.0;
  const float Kp_MAX = 500.0;
  const float Kd_MIN = 0.0;
  const float Kd_MAX = 5.0;

  int canInit() {
    int result_down = system("sudo ip link set can0 down");
    int result_up = system("sudo ip link set can0 up type can bitrate 1000000");

    int socket_int = socket(AF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_int == -1) {
      perror("Socket creation error");
      return -1; // Return an error code
    }

    // Specify the CAN interface (e.g., "can0")
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(socket_int, SIOCGIFINDEX, &ifr) == -1) {
      perror("ioctl error");
      close(socket_int);
      return -1; // Return an error code
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_int, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      perror("Bind error");
      close(socket_int);
      return -1; // Return an error code
    }

    return socket_int; // Return the socket file descriptor
  }

  void powerOn() {
    struct can_frame frame;

    // Create a CAN frame with the power-on message
    frame.can_id = 0x1; // Set the CAN ID for power-on message
    frame.can_dlc = 8;  // 9 bytes of data

    // Fill the frame with the power-on message (0xff 0xff 0xff 0xff 0xff 0xff
    // 0xff 0xff 0xfc)
    for (int i = 0; i < 7; i++) {
      frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFC;

    // Send the power-on message
    write(socket_can, &frame, sizeof(struct can_frame));
  }

  void idleMotor() {
    struct can_frame frame;
    packCmd(&frame, 0.0, 0.0, 0.0, 0.0, 0.0);
    write(socket_can, &frame, sizeof(struct can_frame));
    // std::cout<<"the size of frame"<<sizeof(struct can_frame)<<std::endl;
    sleep(0.01);
  }

  // Function to pack command data into a CAN frame.
  void packCmd(struct can_frame *frame, float p_des, float v_des, float kp,
                float kd, float t_ff) {
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
    frame->can_dlc = 8;  // 8 bytes of data

    frame->data[0] = p_int >> 8;   // Position 8 higher
    frame->data[1] = p_int & 0xFF; // Position 8 lower
    frame->data[2] = v_int >> 4;   // Speed 8 higher
    frame->data[3] = ((v_int & 0xF) << 4) |
                     (kp_int >> 8); // Speed 4 bits lower, KP 4 bits higher
    frame->data[4] = kp_int & 0xFF;
    frame->data[5] = kd_int >> 4;
    frame->data[6] = ((kd_int & 0xF) << 4) |
                     (t_int >> 8); // KD 4 bits lower, T 8 bits higher
    frame->data[7] = t_int & 0xFF;
    // Additional packing logic...
  }

  int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    if (x < x_min)
      x = x_min;
    else if (x > x_max)
      x = x_max;
    return (int)((x - x_min) * ((float)(1 << bits) / span));
  }

  float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    float x = (float)x_int / ((float)(1 << bits) / span) + offset;
    return x;
  }

  void setZeroPosition() {
    struct can_frame frame;

    frame.can_id = 0x1; // Set the CAN ID
    frame.can_dlc = 8;  // 8bytes of data

    // Fill the frame with the power-on message (0xff 0xff 0xff 0xff 0xff 0xff
    // 0xff 0xff 0xfc)
    for (int i = 0; i < 7; i++) {
      frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFE;

    // Send the power-on message
    write(socket_can, &frame, sizeof(struct can_frame));
  }

  void updateMotorState() {
    struct can_frame frame;
    ssize_t bytesRead = read(socket_can, &frame, sizeof(struct can_frame));
    unpackReply(frame);
  }

  void unpackReply(const struct can_frame frame) {
    int id = frame.data[0];
    int position_uint = (frame.data[1] << 8) | frame.data[2];
    int velocity_uint = ((frame.data[3] << 8) | (frame.data[4] >> 4)) & 0xFFF;
    int torque_uint = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    output_position = uint_to_float(position_uint, P_MIN, P_MAX, 16);
    output_velocity = uint_to_float(velocity_uint, V_MIN, V_MAX, 12);
    output_torque = uint_to_float(torque_uint, T_MIN, T_MAX, 12);
  }

  void powerOff() {
    struct can_frame frame;

    frame.can_id = 0x1; // Set the CAN ID for power-on message
    frame.can_dlc = 8;  // 8 bytes of data

    for (int i = 0; i < 7; i++) {
      frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFD;

    // Send the power-off message
    write(socket_can, &frame, sizeof(struct can_frame));
    sleep(1);
    // std::cout<< "The final position of this motor is " << p << v  << t <<
    // std::endl;
  }

  void setOutputTorque(float torque) {
    struct can_frame frame;

    // set torque to the pack message and do the mapping
    packCmd(&frame, 0, 0, 0, 0, torque);

    // Send the frame using the CAN socket.
    write(socket_can, &frame, sizeof(struct can_frame));
  }
};

CanMotorController *mit_controller_ptr = nullptr;



void handleInterruptMember(int signal) {
  if (mit_controller_ptr) {
    delete mit_controller_ptr;
  }
  exit(signal);
}

int main() {
  // Create an instance of the canmotorcontroller class
  mit_controller_ptr = new CanMotorController;

  // Set the signal handler for SIGINT (Ctrl+C)
  signal(SIGINT, handleInterruptMember);

  while (true) {
    mit_controller_ptr->setTorque(-1.31);
    float torque = mit_controller_ptr->getTorque();
    float position = mit_controller_ptr->getPositionDegrees();
    cout << "The motor torque now is" << torque << endl;
    cout << "The motor position now is" << position << endl;
    sleep(0.01);
  }

  return 0;
} 