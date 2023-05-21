#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <thread>  
#include <math.h>
#include <errno.h>
#include <pigpio.h>
#include <PiPCA9685/PCA9685.h>

#include "bot_utils.h"

float ctr = 0.0f;

int calcS(float val)
{
  val = val / 90.0f - 1.0f;
  if (val >= 1.0f) val = 1.0f;
  if (val <= -1.0f) val = -1.0f;
  return ( val * (float)(SERVOMAX - SERVOMIN) / 2.) + SERVOMID;
}

int main(int argc, char** argv )
{
    // Check command line arguments
    bool calibration_mode = false;
    bool motor_move = false;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[1], "--calibrate") == 0)
        {
            calibration_mode = true;
        }
        else if (strcmp(argv[1], "--manual_move") == 0)
        {
            motor_move = true;
        }
        else if (strcmp(argv[1], "--help") == 0)
        {
            printf("Usage: %s [--calibrate|--manual_move|--help]\n", argv[0]);
            return 0;
        }
    }
    
    gpioInitialise();


    if (motor_move)
    {
        PiPCA9685::PCA9685 pca{};
        pca.set_pwm_freq(50);
        for (int i = 0; i < 16; i++)
        {
            pca.set_pwm(i, 0, calcS(90.0f));
        }
        while(true)
        {
            // Read motor index and position from stdin
            int index;
            float position;
            printf("Enter index and position: ");
            int ret = scanf("%d %f", &index, &position);
            if (ret != 2)
            {
                goto exit;
            }

            // Set motor position
            pca.set_pwm(index, 0, calcS(position));
        }
    }
    else if (calibration_mode)
    {
        const int clk_pin = 27;
        const int dt_pin = 22;
        const uint32_t dt_mask = 1 << dt_pin;
        const uint32_t clk_mask = 1 << clk_pin;
        const uint32_t clkdt_mask = clk_mask | dt_mask;

        PiPCA9685::PCA9685 pca{};
        pca.set_pwm_freq(50);

        //Setup the clk pin
        gpioSetMode(clk_pin, PI_INPUT);
        
        //Setup the dt pin
        gpioSetMode(dt_pin, PI_INPUT);

        //Start a separate thread for the encoder measurement
        volatile int rot_counter = 0;
        
        std::thread encoder_thread([&rot_counter](){
            uint32_t prev_clk = 0, cur_clk = 0, cur_dt = 0;
            uint32_t prev_clkdt = 0, cur_clkdt = 0;
            while(true)
            {
                cur_clkdt = gpioRead_Bits_0_31() & clkdt_mask;
                cur_clk = cur_clkdt & clk_mask;
                cur_dt = cur_clkdt & dt_mask;
                if (cur_clk != prev_clk && cur_clk)
                {
                    if (cur_dt)
                        rot_counter --;
                    else
                        rot_counter ++;

                    //printf("rot_counter: %d\n", rot_counter);
                }
                usleep(1000);
                prev_clk = cur_clk;
            }
        });


        while(true)
        {
            //Wait for the user to specify the servo index
            int servo_index = -1;
            do
            {
                printf("Enter servo index: ");
                int ret = scanf("%d", &servo_index);
                if (servo_index == -1) goto exit;
                if (servo_index < 0 || servo_index > 15)
                {
                    printf("Invalid servo index: %d\n", servo_index);
                }
            }
            while (servo_index < 0 || servo_index > 15);

            //Start calibration by setting the servo to a reasonable middle value (300)
            //Then gradually decrease the value and check how the counter value changes
            //When the counter value stops changing, we have found the minimum value
            //Then gradually increase the value and check how the counter value changes
            //When the counter value stops changing, we have found the maximum value
            //Save the minimum and maximum values and convert the counter values to angles
            int min_counter = INT32_MAX, max_counter = INT32_MIN, min_pwm = INT32_MAX, max_pwm = INT32_MIN;
            int prev_counter = -2, cur_counter = -1, same_counter = 0;
            
            //Set the servo to a reasonable middle value
            /*pca.set_pwm(servo_index, 0, SERVOMID);

            //Wait for the counter value to settle
            while (prev_counter != cur_counter)
            {
                prev_counter = cur_counter;
                usleep(1000);
                cur_counter = rot_counter;
            }*/
            
            pca.set_pwm(servo_index, 0, SERVOMID + 50);

            //Decrease the servo value until the counter value stops changing
            for (int pwm = SERVOMID; pwm >= 0; pwm --)
            {
                pca.set_pwm(servo_index, 0, pwm);
                usleep(20000);
                prev_counter = cur_counter;
                cur_counter = rot_counter;
                if (prev_counter == cur_counter)
                {
                    min_counter = cur_counter;
                    if (same_counter == 0) min_pwm = pwm;
                    same_counter++;
                }
                else
                    same_counter = 0;
                
                if(same_counter > 25)
                {
                    min_pwm = pwm;
                    printf("min_counter: %d, min_pwm: %d\n", min_counter, min_pwm);
                    break;
                }
            }

            prev_counter = -2, cur_counter = -1, same_counter = 0;
            pca.set_pwm(servo_index, 0, SERVOMID - 50);

            //Increase the servo value until the counter value stops changing
            for (int pwm = SERVOMID; pwm <= 2048; pwm ++)
            {
                pca.set_pwm(servo_index, 0, pwm);
                usleep(20000);
                prev_counter = cur_counter;
                cur_counter = rot_counter;
                if (prev_counter == cur_counter)
                {
                    max_counter = cur_counter;
                    if (same_counter == 0) max_pwm = pwm;
                    same_counter++;
                }
                else
                    same_counter = 0;
                
                if (same_counter > 25)
                {
                    printf("max_counter: %d, max_pwm: %d\n", max_counter, max_pwm);
                    break;
                }
            }

            //Calculate the angle range
            float angle_range = (max_counter - min_counter) * 360.0f / 90.0f;

            //Write the calibration data to a file
            char filename[256];
            sprintf(filename, "servo_cals/servo%02d.cal", servo_index);
            FILE* f = fopen(filename, "w");
            if (f == NULL)
            {
                printf("Error opening file %s: %d\n", filename, errno);
                continue;
            }
            fprintf(f, "%d %d %f\n", min_pwm, max_pwm, angle_range);
            fclose(f);

            printf("Calibration data written to %s\n", filename);

            //Reset the servo to the middle value
            pca.set_pwm(servo_index, 0, (max_pwm - min_pwm) / 2 + min_pwm);
        }
    }
    else
    {
        ServoController servo_controller{};

        float a01 = M_PI_2;
        float a12 = M_PI_2;
        float a23 = M_PI_2;
        float a3e = M_PI_2;
        while(true)
        {
            glm::vec3 pos_fl = calcLegPosFromCenter(true, true, a01, a12, a23, a3e);
            glm::vec3 pos_fr = calcLegPosFromCenter(true, false, a01, a12, a23, a3e);
            glm::vec3 pos_bl = calcLegPosFromCenter(false, true, a01, a12, a23, a3e);
            glm::vec3 pos_br = calcLegPosFromCenter(false, false, a01, a12, a23, a3e);

            printf("FL: %f %f %f\n", pos_fl.x, pos_fl.y, pos_fl.z);
            printf("FR: %f %f %f\n", pos_fr.x, pos_fr.y, pos_fr.z);
            printf("BL: %f %f %f\n", pos_bl.x, pos_bl.y, pos_bl.z);
            printf("BR: %f %f %f\n", pos_br.x, pos_br.y, pos_br.z);

            servo_controller.Move(FL0, a01);
            servo_controller.Move(FR0, a01);
            servo_controller.Move(BL0, a01);
            servo_controller.Move(BR0, a01);

            servo_controller.Move(FL1, a12);
            servo_controller.Move(FR1, a12);
            servo_controller.Move(BL1, a12);
            servo_controller.Move(BR1, a12);

            servo_controller.Move(FL2, a23);
            servo_controller.Move(FR2, a23);
            servo_controller.Move(BL2, a23);
            servo_controller.Move(BR2, a23);

            servo_controller.Move(FL3, a3e);
            servo_controller.Move(FR3, a3e);
            servo_controller.Move(BL3, a3e);
            servo_controller.Move(BR3, a3e);

            /*pca.set_pwm(FR0, 0, calcS( 90.0f));
            pca.set_pwm(FL0, 0, calcS( 90.0f));
            pca.set_pwm(BR0, 0, calcS( 90.0f));
            pca.set_pwm(BL0, 0, calcS( 90.0f));

            pca.set_pwm(FR1, 0, calcS( 99));
            pca.set_pwm(FL1, 0, calcS( 90));
            pca.set_pwm(BR1, 0, calcS( 99));
            pca.set_pwm(BL1, 0, calcS( 99));

            float ctr0 = asinf(sinf(ctr)) * (90 * 1.8f) / 3.1415f;
            ctr += 0.1f;
            if (ctr > 2*3.1415f)ctr -= 2*3.1415f;

            pca.set_pwm(FR2, 0, calcS(ctr0));
            pca.set_pwm(FL2, 0, calcS(-ctr0));
            pca.set_pwm(BR2, 0, calcS(ctr0));
            pca.set_pwm(BL2, 0, calcS(-ctr0));

            float off = -0.0f;
            pca.set_pwm(FL3, 0, calcS(-(ctr0 + off)));
            pca.set_pwm(FR3, 0, calcS(ctr0 + off));
            pca.set_pwm(BL3, 0, calcS(-(ctr0 + off)));
            pca.set_pwm(BR3, 0, calcS(ctr0 + off));*/
            


            usleep(5'000'000);
        }
    }

exit:
    gpioTerminate();
    return 0;
}