    /*====================================================================================================*/
    /* Serial Port Programming in C (Serial Port Read)                                                    */
	/* Non Cannonical mode                                                                                */
	/*----------------------------------------------------------------------------------------------------*/
    /* Program reads a string from the serial port at 115200 bps 8N1 format                               */
	/* Baudrate - 115200                                                                                  */
	/* Stop bits -1                                                                                       */
	/* No Parity                                                                                          */
    /*----------------------------------------------------------------------------------------------------*/
	/* Compiler/IDE  : gcc 4.6.3                                                                          */
	/* Library       :                                                                                    */
	/* Commands      : gcc -o serialport_read serialport_read.c                                           */
	/* OS            : Linux(x86) (Linux Mint 13 Maya)(Linux Kernel 3.x.x)                                */                              
	/* Programmer    : Rahul.S                                                                            */
	/* Date	         : 21-December-2014                                                                   */
	/*====================================================================================================*/

	/*====================================================================================================*/
	/* www.xanthium.in										      */
	/* Copyright (C) 2014 Rahul.S                                                                         */
	/*====================================================================================================*/

	
	/*-------------------------------------------------------------*/
    /* termios structure -  /usr/include/asm-generic/termbits.h    */ 
	/* use "man termios" to get more info about  termios structure */
	/*-------------------------------------------------------------*/

    	#include <stdio.h>
    	#include <fcntl.h>   /* File Control Definitions           */
    	#include <termios.h> /* POSIX Terminal Control Definitions */
    	#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
    	#include <errno.h>   /* ERROR Number Definitions           */
    	#include <string.h>
    	#include <stdlib.h>
        
        int fd;/*File Descriptor*/
	
	    void configure_serial_port(void)
    	{
		    /*------------------------------- Opening the Serial Port -------------------------------*/

		    /* Open /dev/ttyACM0, as this is the port for USB to Serial data on the BeagleBone       */

        	fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);
			   					/* O_RDWR   - Read/Write access to serial port       */
								/* O_NOCTTY - No terminal will control the process   */
								/* Open in blocking mode,read will wait              */
									
									                                        
									
        	if(fd == -1)						/* Error Checking */
            	   printf("\nError! in Opening ttyACM0  ");
        	else
            	   printf("\nttyACM0 Opened Successfully ");

	
		    /*---------- Setting the Attributes of the serial port using termios structure --------- */
		
		    struct termios SerialPortSettings;	/* Create the structure                          */

		    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		    /* Setting the Baud rate */
		    cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 115200                   */
		    cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 115200                   */

		    /* 8N1 Mode */
		    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
		
		    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		
		
		    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

		    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
		
		    /* Setting Time outs */
		    SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
		    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


		    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
		        printf("\nSetup Faile: ERROR ! in Setting attributes\n\n");
		    else
                printf("\nSetup Complete: BaudRate = 115200  StopBits = 1  Parity = none\n");
			
    	}
    	
    	
    	int read_serial_data(int *timestamp, float *imu_data)
    	{
	        /*------------------------------- Read data from serial port -----------------------------*/

		    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */

		    int  bytes_read = 0;    /* Number of bytes read by the read() system call */
 		    int i = 0;
            char data_str[20] = "0.2345";
            int firstItem = 1;
            int prev_comma_location = 0;
            int imu_data_cnt = 0;
            char read_buffer[256];
        
                bytes_read = read(fd,read_buffer,128); /* Read the data                   */
		        for(i=0;i<bytes_read;i++)	 /*printing only the received characters*/
		        {
		            if(read_buffer[i] == ',') 
		            {
		                if(firstItem)
		                {
		                    strncpy(data_str, read_buffer, i);
		                    *timestamp = atoi(data_str);
		                    firstItem = 0;
		                    prev_comma_location = i;
		                }
		                else
		                {
		                    memset(data_str, '\0', 20);
		                    strncpy(data_str, &read_buffer[prev_comma_location+2], i-(prev_comma_location+2));
		                    data_str[i-(prev_comma_location+2)] = '\0';
		                    imu_data[imu_data_cnt] = atof(data_str);
//		                    printf("Data %d = %f\n", imu_data_cnt, imu_data[imu_data_cnt]);
                            imu_data_cnt++;
    	                    prev_comma_location = i;		                    
		                }
		            }
		            else if(i == (bytes_read-1))
		            {
		                memset(data_str, '\0', 20);
		                strncpy(data_str, &read_buffer[prev_comma_location+2], i-(prev_comma_location+2));
		                data_str[i-(prev_comma_location+2)] = '\0';
		                imu_data[imu_data_cnt] = atof(data_str);
//		                printf("Data %d = %f\n", imu_data_cnt, imu_data[imu_data_cnt]);
                        imu_data_cnt++;
		            }
		            
		        }
//		            printf("%c",read_buffer[i]);
	
        		printf("\n");
//            }
            return bytes_read;
    	}
    	
    	void close_serial_port(void)
    	{
		    close(fd); /* Close the serial port */
    	    printf("Serial Port Closed\n");
    	}
