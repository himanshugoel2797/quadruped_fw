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
#define LET 22

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
    pos_2e += glm::vec3(0, L23 * sinf(a23), L23 * cosf(a23));   //Thigh bone
    pos_2e += glm::vec3(0, L3E * cosf(a3e), -L3E * sinf(a3e));  //Lower leg bone
    pos_2e += glm::vec3(0, -LET * sinf(AET), -LET * cosf(AET)); //Foot

    //Rotate pos_2e around pos_ax by a12
    float pos_2e_rellen = glm::vec2(pos_2e.x, pos_2e.z).length();
    pos = glm::vec3(pos_2e_rellen * sinf(a12), pos_2e.y + L12 + L01, pos_2e_rellen * cosf(a12)); //Pelvis XZ rotation

    //Rotate pos around origin by a01 along xy plane
    float pos_rellen = glm::vec2(pos.x, pos.y).length();
    pos = glm::vec3(pos_rellen * cosf(a01), pos_rellen * sinf(a01), pos.z); //Pelvis XY rotation

    return pos;
}

static glm::vec3 calcLegPosFromCenter(bool front, bool left, float a01, float a12, float a23, float a3e)
{
    glm::vec3 pos = glm::vec3(0);
    pos.x += front ? BODY_LENGTH : -BODY_LENGTH;
    pos.y += left ? BODY_WIDTH : -BODY_WIDTH;

    return pos + calcLegPos(a01, a12, a23, a3e);
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
            //printf("Servo %d: %d\n", servo_index, angle);
            //return;
            if (servo_index & 1) angle = 180 - angle;
            int pwm = (SERVOMAX - SERVOMIN) * angle / 180 + SERVOMIN;
            pca.set_pwm(servo_index, 0, pwm);
        }
};


