#include <stdio.h>
#include <unistd.h>
#include <PiPCA9685/PCA9685.h>
#include <errno.h>

int main()
{
   PiPCA9685::PCA9685 pca{};

    pca.setPWMFreq(50);

    for (int i = 0; i < 16; i++)
    {
        pca.setPWM(i, 0, 300);
    }
}