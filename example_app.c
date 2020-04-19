#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <serial_io.h>


void sig_handler(int signo)
{
    printf("Received SIGINT\n");
    close_serial_port();
    usleep(10000);
    printf("Exiting\n");
    exit(0);
    
}

int main(int argc, char *argv[])
{
	// set up signal handler
	if (signal(SIGINT, sig_handler) == SIG_ERR)
		printf("\ncan't catch SIGINT\n");
	else
		printf("Successfully set up SIGINT handler\n");	
	
	configure_serial_port();

    int bytes_read = 0;
    int i = 0;
    int timestamp = 0;
    float imu_data[64];
    
	while(1)
	{
	    bytes_read = read_serial_data(&timestamp, imu_data);
	    printf("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", timestamp, imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5],imu_data[6],imu_data[7],imu_data[8]);
//		usleep(1000);
	}
}	






                 
