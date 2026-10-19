#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "i2c.h"
#include "max9867.h"

#define I2C_BUS MXC_I2C1

int main(void)
{
    int result;

    printf("\nCodec start test\n");

    result = MXC_I2C_Init(I2C_BUS, 1, 0);
    printf("Bus start result: %d\n", result);
    MXC_I2C_SetFrequency(I2C_BUS, 100000);

    result = max9867_init(I2C_BUS, 12288000, 1);
    printf("Codec init result: %d (0 means success)\n", result);

    while (1) {
        MXC_Delay(MXC_DELAY_SEC(2));
        printf("alive\n");
    }
}