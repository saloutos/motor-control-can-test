/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"


/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 10.0f
#define T_MIN -72.0f
#define T_MAX 72.0f

// Master CAN ID ///
#define CAN_ID 0x0

// Blinking rate in milliseconds
#define BLINKING_RATE_MS                                                    500

// Initialize serial port
RawSerial pc(USBTX, USBRX, 115200);
// Initialise the digital pin LED1 as an output
DigitalOut led(LED1);

// CAN setup
PinName MOTOR_CAN_TX = PD_1; //PB_9;
PinName MOTOR_CAN_RX = PD_0; //PB_8;
CAN motor_can(MOTOR_CAN_RX, MOTOR_CAN_TX, 1000000);
CANMessage   rxMsg;
CANMessage   txMsg;

// controller setup
Ticker control_loop;
float p_des, v_des, kp, kd, t_ff;
float p, v, t;
int loop_count; 

// helper functions
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void pack_cmd(CANMessage * msg, float p_des_raw, float v_des_raw, float kp_raw, float kd_raw, float t_ff_raw){
     
     /// limit data to be within bounds ///
     float p_des = fminf(fmaxf(P_MIN, p_des_raw), P_MAX);                    
     float v_des = fminf(fmaxf(V_MIN, v_des_raw), V_MAX);
     float kp = fminf(fmaxf(KP_MIN, kp_raw), KP_MAX);
     float kd = fminf(fmaxf(KD_MIN, kd_raw), KD_MAX);
     float t_ff = fminf(fmaxf(T_MIN, t_ff_raw), T_MAX);
     /// convert floats to unsigned ints ///
     uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
     uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
     uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
     /// pack ints into the can buffer ///
     msg->data[0] = p_int>>8;                                       
     msg->data[1] = p_int&0xFF;
     msg->data[2] = v_int>>4;
     msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     msg->data[4] = kp_int&0xFF;
     msg->data[5] = kd_int>>4;
     msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     msg->data[7] = t_int&0xff;
     }
     
/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]

void unpack_reply(CANMessage msg){
    /// unpack ints from can buffer ///
    uint16_t id = msg.data[0];
    uint16_t p_int = (msg.data[1]<<8)|msg.data[2];
    uint16_t v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    uint16_t i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    /// convert uints to floats ///
    p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}

void Zero(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFE;
    //WriteAll();
    }

void EnterMotorMode(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFC;
    //WriteAll();
    }
    
void ExitMotorMode(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFD;
    //WriteAll();
    }

void WriteAll(){
    motor_can.write(txMsg);
    wait(.00002);
}

// controller ISR
int control_flag = 0;
void control(){
    control_flag = 1;
}

// main loop
int main()
{
    


    // serial setup
    pc.printf("Starting.\n\r");
    wait(2.0);
    

    // CAN setup
    // motor_can.filter(CAN_ID, 0x000, CANStandard, 0);
    txMsg.len = 8;
    txMsg.id = 0x01;
    rxMsg.len = 6;

    pc.printf("Starting motor control test.\n\r");
    // initial motor commands
    pc.printf("Entering motor mode.\n\r");
    EnterMotorMode(&txMsg);
    motor_can.write(txMsg);

    wait(0.1);
    
    // pc.printf("Exiting motor mode.\n\r");
    // ExitMotorMode(&txMsg);
    // motor_can.write(txMsg);

    // attach control interrupt
    control_loop.attach(&control, 0.002); // at 500Hz

    while (true) {
        if (control_flag == 1){
            // pc.printf("Here 2.");
            control_flag = 0;
            loop_count = loop_count + 1;
            if ((loop_count%250) == 0){
                led = !led;
            }
            if (loop_count>(500*10)){
                pc.printf("Exiting motor mode.\n\r");
                ExitMotorMode(&txMsg);
                motor_can.write(txMsg);
                wait_us(100);
                control_loop.detach();
            } else {
                pack_cmd(&txMsg, 0.0, 0.0, 10.0, 0.1, 0.0);
                motor_can.write(txMsg);
                wait_us(100);
            }
        }
    }


}
