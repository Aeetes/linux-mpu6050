#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>

struct mpu6050_data {
	struct {
		signed short x;
		signed short y;
		signed short z;
	} accel;
	struct {
		signed short x;
		signed short y;
		signed short z;
	} gyro;
	signed short temp;
};

int main(int argc, char *argv[])
{
	int fd;
	struct mpu6050_data data;
	signed int gyro_x_adc, gyro_y_adc, gyro_z_adc;
	signed int accel_x_adc, accel_y_adc, accel_z_adc;
	signed int temp_adc;

	float gyro_x_act, gyro_y_act, gyro_z_act;
	float accel_x_act, accel_y_act, accel_z_act;
	float temp_act;

	int ret = 0;
	char* filename = "/dev/mpu6050";
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}

	while (1) {
		ret = read(fd, &data, sizeof(data));
		if(ret == 0) {
			gyro_x_adc = data.gyro.x;
			gyro_y_adc = data.gyro.y;
			gyro_z_adc = data.gyro.z;
			accel_x_adc = data.accel.x;
			accel_y_adc = data.accel.y;
			accel_z_adc = data.accel.z;
			temp_adc = data.temp;

			gyro_x_act = (float)(gyro_x_adc)  / 16.4;
			gyro_y_act = (float)(gyro_y_adc)  / 16.4;
			gyro_z_act = (float)(gyro_z_adc)  / 16.4;
			accel_x_act = (float)(accel_x_adc) / 2048;
			accel_y_act = (float)(accel_y_adc) / 2048;
			accel_z_act = (float)(accel_z_adc) / 2048;
			temp_act = ((float)(temp_adc) - 25 ) / 326.8 + 25;

			printf("\r\nADC value:\r\n");
			printf("gx = %d, gy = %d, gz = %d\r\n", gyro_x_adc, gyro_y_adc, gyro_z_adc);
			printf("ax = %d, ay = %d, az = %d\r\n", accel_x_adc, accel_y_adc, accel_z_adc);
			printf("temp = %d\r\n", temp_adc);
			printf("TRUE value:");
			printf("act gx = %.2f°/S, act gy = %.2f°/S, act gz = %.2f°/S\r\n", gyro_x_act, gyro_y_act, gyro_z_act);
			printf("act ax = %.2fg, act ay = %.2fg, act az = %.2fg\r\n", accel_x_act, accel_y_act, accel_z_act);
			printf("act temp = %.2f°C\r\n", temp_act);
		}
		usleep(100000); 
	}
	close(fd);
	return 0;
}


