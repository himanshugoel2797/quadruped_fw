#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <thread>  
#include <math.h>
#include <errno.h>
#include <pigpio.h>
#include <PiPCA9685/PCA9685.h>

#define SERVOMIN  100
#define SERVOMID 300
#define SERVOMAX  500

#define FR0 6
#define FL0 7
#define FR1 4
#define FL1 5
#define FR2 2
#define FL2 3
#define FR3 0
#define FL3 1
#define BR0 8
#define BL0 9
#define BR1 10
#define BL1 11
#define BR2 12
#define BL2 13
#define BR3 14
#define BL3 15


float ctr = 0.0f;

int calcS(float val)
{
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

    PiPCA9685::PCA9685 pca{};
    pca.set_pwm_freq(50);

    if (motor_move)
    {
        for (int i = 0; i < 16; i++)
        {
            pca.set_pwm(i, 0, calcS(0.0f));
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
                printf("Error reading from stdin: %d\n", errno);
                continue;
            }

            // Set motor position
            pca.set_pwm(index, 0, (int)position);
        }
    }
    else if (calibration_mode)
    {
        const int clk_pin = 27;
        const int dt_pin = 22;
        const uint32_t dt_mask = 1 << dt_pin;
        const uint32_t clk_mask = 1 << clk_pin;
        const uint32_t clkdt_mask = clk_mask | dt_mask;

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
            pca.set_pwm(servo_index, 0, SERVOMID);
        }
    }
/*
        pca.set_pwm(FR0, 0, calcS( 0.0f));
        pca.set_pwm(FL0, 0, calcS( 0.0f));
        pca.set_pwm(BR0, 0, calcS( 0.0f));
        pca.set_pwm(BL0, 0, calcS( 0.0f));

        pca.set_pwm(FR1, 0, calcS(-0.1f));
        pca.set_pwm(FL1, 0, calcS(-0.1f));
        pca.set_pwm(BR1, 0, calcS( 0.0f));
        pca.set_pwm(BL1, 0, calcS(-0.1f));

        float ctr0 = asinf(sinf(ctr)) * 0.8f / 3.1415f;
        ctr += 0.8f;
        if (ctr > 2*3.1415f)ctr -= 2*3.1415f;

        pca.set_pwm(FR2, 0, calcS(ctr0));
        pca.set_pwm(FL2, 0, calcS(ctr0));
        pca.set_pwm(BR2, 0, calcS(ctr0));
        pca.set_pwm(BL2, 0, calcS(ctr0));

        float off = -0.1f;
        pca.set_pwm(FR3, 0, calcS(ctr0 + off));
        pca.set_pwm(FL3, 0, calcS(ctr0 + off));
        pca.set_pwm(BR3, 0, calcS(ctr0 + off));
        pca.set_pwm(BL3, 0, calcS(ctr0 + off));
        
        usleep(50'000);
*/

exit:
    gpioTerminate();
    return 0;
}