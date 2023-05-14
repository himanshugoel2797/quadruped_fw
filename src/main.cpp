#include <stdio.h>
#include <unistd.h>
#include <PiPCA9685/PCA9685.h>
#include <errno.h>

int main()
{
   PiPCA9685::PCA9685 pca{};

    pca.set_pwm_freq(50);

    for (int i = 0; i < 16; i++)
    {
        pca.set_pwm(i, 0, 300);
    }
}