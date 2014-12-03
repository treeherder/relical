/**************************************************
* Code for the DX.com 10DOF sensor board - GY-80
*
* With some code borrowed from that by James Henderson
**************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

/* I2C device addresses */
int c_address = 0x1E;
int a_address = 0x53;
int g_address = 0x69;
int p_address = 0x77;

/* Storage for calibration data */
struct p_calib_data {
    short ac1,ac2,ac3;
    unsigned short ac4,ac5,ac6;
    short b1,b2,mb,mc,md;
} p_calib;


/*************************************************/
static void i2c_seek(int fd, unsigned char offset) {
    if ((write(fd, &offset, 1)) != 1) {
        fprintf(stderr, "Error writing to i2c slave %2x",offset);
        exit(1);
    }
}

/*************************************************/
static int i2c_write_reg(int fd, unsigned char reg, unsigned char val) {
    char buf[2];
    buf[0] = reg;
    buf[1] = val;
    if ((write(fd, buf, 2)) != 2) {
        fprintf(stderr, "Error writing to i2c slave %2x %2x\n",reg,val);
        return -1;
    }
    return 0;
}

/*************************************************/
static int i2c_read(int fd, unsigned char offset, unsigned char *buf, unsigned char len)
{
    i2c_seek(fd, offset);
    if (read(fd, buf, len) != len) {
        printf("Unable to read at offset 0x%2X\n",offset);
        return -1;
    }
    return 0;
}

/*************************************************
* Compass setup                                  *
*************************************************/
static int compass_setup(int fd)
{
    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, c_address) < 0) {
        printf("Unable to get bus access to talk to compass sensor\n");
        return -1;
    }

    /* Set the compass into continious read mode */
    if(i2c_write_reg(fd, 2, 0) < 0)        return -1;
    return 0;
}

/*************************************************
* Compass read                                   *
*************************************************/
static int compass_read(int fd)
{
    char buf[6];
    short int x,y,z;
    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, c_address) < 0) {
        printf("Unable to get bus access to compass sensor\n");
        return -1;
    }

    i2c_seek(fd,3);

    /* Wait for conversion.
     * Should reaaly be monitoring status register*/
    usleep( 10000);

    /* Read the compass registers */
    if (read(fd, buf, 6) != 6){
        printf("Unable to read from compass sensor\n");
        return -1;
    }

    /* Display the register values */
    x = (buf[0] <<8) | buf[1];
    y = (buf[2] <<8) | buf[3];
    z = (buf[4] <<8) | buf[5];
    printf("Mag = (%5i, %5i, %5i)", x,y,z);
    return 0;
}

/*************************************************
* Pressure setup   BMP085                        *
*************************************************/
static int pressure_setup(int fd)
{
    char data[22];
    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, p_address) < 0)  {
        printf("Unable to get bus access to talk to pressure sensor\n");
        return -1;
    }
    i2c_seek(fd, 0xAA);
    if (read(fd, data, 22) != 22){
            printf("Unable to read calibrationd data from presure sensor\n");
        return -1;
    }
    p_calib.ac1 = (data[ 0]<<8) | data[ 1];
    p_calib.ac2 = (data[ 2]<<8) | data[ 3];
    p_calib.ac3 = (data[ 4]<<8) | data[ 5];
    p_calib.ac4 = (data[ 6]<<8) | data[ 7];
    p_calib.ac5 = (data[ 8]<<8) | data[ 9];
    p_calib.ac6 = (data[10]<<8) | data[11];
    p_calib.b1  = (data[12]<<8) | data[13];
    p_calib.b2  = (data[14]<<8) | data[15];
    p_calib.mb  = (data[16]<<8) | data[17];
    p_calib.mc  = (data[18]<<8) | data[19];
    p_calib.md  = (data[20]<<8) | data[21];
    return 0;
}

/************************************************
* Pressure read BMP085                          *
*************************************************/
static int read_pressure(int fd)
{
    unsigned char oss = 3;
    unsigned char buf[3];
    /* chech the size of all of these! */
    long ut,up;
    long x1,x2,x3;
    long b3,b5,b6;
    unsigned long b4,b7;
    long t,p;
    float alt;

    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, p_address) < 0) {
       printf("Unable to get bus access to pressure sensor\n");
       return -1;
    }
   
    /* Start conversion to get temperature */
    if(i2c_write_reg(fd, 0xF4, 0x2E) < 0)
        return -1;

    usleep(4500);

    i2c_seek(fd, 0xF6);
    if (read(fd, buf, 2) != 2){
        printf("Unable to read from pressure sensor\n");
        return -1;
    }

    ut = (buf[0] <<8) | buf[1];

    if(i2c_write_reg(fd, 0xF4, 0x34 + (oss <<6)) < 0)
        return -1;

    
    usleep(25500); /* Changes depending on value of oss */

    i2c_seek(fd, 0xF6);
    if (read(fd, buf, 3) != 3){
            printf("Unable to read from pressure sensor\n");
            return -1;
    }

    /* Display the register values */
    up = ((buf[0] <<16) | (buf[1]<<8) | buf[2]) >> (8-oss);

    /* Calculate the true temp */
    x1 = (ut - p_calib.ac6) * p_calib.ac5 / (1<<15);
    x2 = (p_calib.mc *(1<<11)) / (x1 + p_calib.md);
    b5 = x1+x2;
    t = (b5+8)/(1<<4);

    /* Calc pressure */
    b6 = b5 - 4000;
    x1 = (p_calib.b2*((b6*b6)>>12))>>11;
    x2 = (p_calib.ac2*b6)>>11;
    x3 = x1+x2;
    b3 = (((p_calib.ac1*4+x3)<<oss)+2)/4;
    x1 = (p_calib.ac3*b6)>>13;
    x2 = (p_calib.b1*(b6*b6)>>12)>>16;
    x3 = ((x1+x2)+2)>>2;
    b4 = (p_calib.ac4*((unsigned long)x3+32768))>>15;
    b7 = ((unsigned long)up-b3)*(50000>>oss);

    if(b7 < 0x80000000)
      p = (b7*2)/b4;
    else
      p = (b7/b4)*2;

    x1 = (p>>8)*(p>>8);
    x1 = (x1*3038)>>16;
    x2 = (-7357*p)>>16;
    p = p + ((x1+x2+3781)>>4);
    alt = 44330.0 * (1-pow(p/101325.0,1.0/5.255));

    printf("T = %li.%li C,   P = %li Pa  alt = %7.2f m", t/10, t%10, p, alt);

    return 0;
}

/*************************************************
* Accel setup                                    *
*************************************************/
static int accel_setup(int fd)
{
    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, a_address) < 0) {
        printf("Unable to get bus access to talk to accel sensor\n");
        return -1;
    }

    /* Set the acceleration sensor into continious read mode */
    /* approx 3Hz */
    if(i2c_write_reg(fd, 0x2C, 0x06) < 0) return -1;

    /* Data mode */
    if(i2c_write_reg(fd, 0x31, 0x02) < 0) return -1;

    /* Power up */
    if(i2c_write_reg(fd, 0x31, 0x07) < 0) return -1;

    /* Calibration values - I made these up for my sensor! */
    if(i2c_write_reg(fd, 0x1E,   88) < 0) return -1;
    if(i2c_write_reg(fd, 0x1F,   60) < 0) return -1;
    if(i2c_write_reg(fd, 0x20, -127) < 0) return -1;

    return 0;
}

/*************************************************
* Acceleration                                   *
*************************************************/
static int accel_read(int fd)
{
    short int x,y,z;
    unsigned char buf[6];

    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, a_address) < 0) {
        printf("Unable to get bus access to compass\n");
        return -1;
    }

    /* Read the registers */
    if (i2c_read(fd, 0x32 | 0x80, buf, 6) < 0) {
        printf("Unable to read from acceleration sensor\n");
        return -1;
    }
    x = (buf[1] <<8) | buf[0];
    y = (buf[3] <<8) | buf[2];
    z = (buf[5] <<8) | buf[4];

    printf("Accel = (%6i, %6i, %6i)", x,y,z);
    return 0;
}

/************************************************
* Gyro setup                                    *
*************************************************/
static int gyro_setup(int fd)
{
    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, g_address) < 0) {
        printf("Unable to get bus access to talk to gyro sensor\n");
        return -1;
    }

    /* Normal mode, 100 Hz, all axis active */
    if(i2c_write_reg(fd, 0x20, 0x0F) < 0) return -1;
    if(i2c_write_reg(fd, 0x21, 0x00) < 0) return -1;
    if(i2c_write_reg(fd, 0x22, 0x08) < 0) return -1;
    if(i2c_write_reg(fd, 0x23, 0x00) < 0) return -1; 
    if(i2c_write_reg(fd, 0x24, 0x00) < 0) return -1; 

    return 0;
}


/*************************************************
* Gyroscope read                                 *
*************************************************/
static int gyro_read(int fd)
{
    short int x,y,z;
    unsigned char buf[6];

    /* Set address of the device we wish to speak to */
    if (ioctl(fd, I2C_SLAVE, g_address) < 0) {
        printf("Unable to get bus access to gyro\n");
        return -1;
    }

    if (i2c_read(fd, 0x28 | 0x80, buf, 6) < 0) {
        printf("Unable to read from gyro\n");
        return -1;
    }
    x = (buf[1] <<8) | buf[0];
    y = (buf[3] <<8) | buf[2];
    z = (buf[5] <<8) | buf[4];
    printf("Gyro = (%6i, %6i, %6i)", (int)x, (int)y, (int)z);
    return 0;
}

/***********************************************
* The main() program                           *
***********************************************/
int main(int argc, char **argv)
{

    int fd;
    char *fileName = "/dev/i2c-0";

    printf("**** 10DOF example program ****\n");

    if ((fd = open(fileName, O_RDWR)) < 0) {
        printf("Failed to open i2c port\n");
        exit(1);
    }

    /* Call the setup functions */
    compass_setup(fd);
    pressure_setup(fd);
    accel_setup(fd);
    gyro_setup(fd);

    while(1) {
        /* Read all the sensor data */
        compass_read(fd);  printf(",   ");
        read_pressure(fd); printf(",   ");
        accel_read(fd);    printf(",   ");
        gyro_read(fd);     putchar('\n');

        /* Wait a quarter of a second before next reading */
        usleep(40000);
    }

    /* We will never get here!*/
    return 0;
}
