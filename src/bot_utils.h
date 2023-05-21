#pragma once

#include <PiPCA9685/PCA9685.h>
#include "glm/glm.hpp"

#define SERVOMIN  60
#define SERVOMID ((SERVOMIN + SERVOMAX) / 2)
#define SERVOMAX  500

#define BR0 6
#define BL0 7  // Invert angle
#define BR1 4
#define BL1 5  // Invert angle
#define BR2 2
#define BL2 3  // Invert angle
#define BR3 0
#define BL3 1  // Invert angle

#define FR0 8
#define FL0 9  // Invert angle
#define FR1 10
#define FL1 11 // Invert angle
#define FR2 12
#define FL2 13 // Invert angle
#define FR3 14
#define FL3 15 // Invert angle

#define L01 50
#define L12 45
#define L23 45
#define L3E 78
#define LET 33

#define AET 117.0f / 360.0f * 2.0f * 3.14159265359f

#define BODY_LENGTH 42
#define BODY_WIDTH  50

//Cooardinate system:
//X = forward
//Y = left
//Z = up

static glm::vec3 calcLegPos(float a01, float a12, float a23, float a3e)
{
    glm::vec3 pos = glm::vec3(0);
    
    glm::vec3 pos_2e = glm::vec3(0);
    glm::vec3 pos_2e_0 = glm::vec3(L23 * cosf(a23), L23 * sinf(a23), 0);   //Thigh bone
    glm::vec3 pos_2e_1 = glm::vec3(-L3E * sinf(a3e + M_PI / 3.0f), -L3E * cosf(a3e + M_PI / 3.0f), 0);  //Lower leg bone
    glm::vec3 pos_2e_2 = glm::vec3(LET * cosf(-AET), LET * sinf(-AET), 0); //Foot

    pos_2e = pos_2e_0 + pos_2e_1 + pos_2e_2; //Leg position relative to hip
    //printf("pos_2e_0: %f %f %f\n", pos_2e_0.x, pos_2e_0.y, pos_2e_0.z);
    //printf("pos_2e_1: %f %f %f\n", pos_2e_1.x, pos_2e_1.y, pos_2e_1.z);
    //printf("pos_2e_2: %f %f %f\n", pos_2e_2.x, pos_2e_2.y, pos_2e_2.z);
    //printf("pos_2e: %f %f %f\n", pos_2e.x, pos_2e.y, pos_2e.z);

    //Rotate pos_2e around pos_ax by a12
    pos = glm::vec3(pos_2e.x * cosf(a12), pos_2e.y + L12 + L01, pos_2e.x * sinf(a12)); //Pelvis XZ rotation
    //printf("pos: %f %f %f\n", pos.x, pos.y, pos.z);

    //Rotate pos around origin by a01 along xy plane
    float pos_rellen = glm::length(glm::vec2(pos.x, pos.y));
    //printf ("pos_rellen: %f\n", pos_rellen);
    pos = glm::vec3(pos_rellen * cosf(a01 - M_PI_4), pos_rellen * sinf(a01 - M_PI_4), pos.z); //Pelvis XY rotation
    //printf("pos: %f %f %f\n", pos.x, pos.y, pos.z);

    return pos;
}

static glm::vec3 calcLegPosFromCenter(bool front, bool left, float a01, float a12, float a23, float a3e)
{
    glm::vec3 pos = glm::vec3(BODY_LENGTH, BODY_WIDTH, 0);
    pos += calcLegPos(a01, a12, a23, a3e);
    if (!front) pos.x = -pos.x;
    if (!left) pos.y = -pos.y;
    return pos;
}

class ServoController{
    private:
        PiPCA9685::PCA9685 pca;
    
    public:
        ServoController() {
            pca.set_pwm_freq(50);
        }

        void Move(int servo_index, float angle)
        {
            //printf("Servo %d: %f\n", servo_index, angle);
            //return;
            if (servo_index & 1) angle = M_PI - angle;
            int pwm = (SERVOMAX - SERVOMIN) * angle / M_PI + SERVOMIN;
            pca.set_pwm(servo_index, 0, pwm);
        }
};


