/**
  ******************************************************************************
  * @file    stm32-motor-ctrl.c
  * @author  michtw@gmail.com
  * @version 1.0
  * @date    1-Nov-2017
  * @brief   STM32 motor control utility.
  *******************************************************************************
 **/

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "MC.h"

struct termios orig_tty;

static int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }
    orig_tty = tty;

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    cfmakeraw(&tty);

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

static void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}

static uint8_t CalcCRC (PFrameData_t pFrame)
{
    uint8_t nCRC = 0;
    uint16_t nSum = 0;
    uint8_t idx;

    nSum += pFrame->Code;
    nSum += pFrame->Size;

    for(idx = 0; idx < (pFrame->Size); idx++)
    {   
        nSum += pFrame->Buffer[idx];
    }   
    nCRC = (uint8_t)(nSum & 0xFF); // Low Byte of nSum
    nCRC += (uint8_t) (nSum >> 8); // High Byte of nSum

    return nCRC;
}

static uint8_t IsFrameValid (PFrameData_t pFrame)
{
    if (pFrame)
        return CalcCRC(pFrame) == pFrame->Buffer[pFrame->Size];
    else
        return 0;
}

static int init_uart (const char *uart)
{
    int fd = open (uart, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf ("error %d opening %s: %s", errno, uart, strerror (errno));
        return fd;
    }

    // B115200  380400 B230400
    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                    // set no blocking

    return fd;
}

static uint16_t receivedFrame (int fd, uint8_t *buff)
{
    int i, ret;
    uint8_t header[2];

    // Get frame header.
    for (i = 0; i < FRAME_HEADER_SIZE; i++) {
        read(fd, &header[i], 1);
    }

    uint16_t buffSize = header[1];
    buffSize++;  // Last character is CRC checksum.
    for (i = 0; i < buffSize; i++) {
        read(fd, &buff[i], 1);
    }
    // uint8_t crc = buff[buffSize-1];

    FrameData_t f;
    f.Code = header[0];
    f.Size = header[1];
    for (i = 0; i < buffSize; i++) {
        f.Buffer[i] = buff[i];
    }
    ret = IsFrameValid(&f);
    if (!ret) {
        printf("Receive frame failed.\n");
    }
/*
    for (i = 0; i < buffSize; i++) {
        printf("%02x ", buff[i]);
    }
    printf("\n");
 */
    return (buffSize - 1);
}

static ssize_t sentFrame (int fd, PFrameData_t pFrame_t)
{
    int i;
    uint8_t buff[FRAME_MAX_BUFFER_LENGTH];
    buff[0] = pFrame_t->Code;
    buff[1] = pFrame_t->Size;

    for (i = 0; i < pFrame_t->Size; i++) {
        buff[i+2] = pFrame_t->Buffer[i];
    }
    buff[FRAME_HEADER_SIZE + pFrame_t->Size] = pFrame_t->nCRC;

    return write(fd, buff, FRAME_HEADER_SIZE + pFrame_t->Size + FRAME_CRC_SIZE);
}

static int getAck (int fd)
{
    int ret = ACK_ERROR;
    uint8_t buff[128];
    receivedFrame(fd, buff);
    if (buff[0] == ACK_NOERROR) {
        ret = ACK_NOERROR;
    } else if (buff[0] == ACK_ERROR) {
        ret = ACK_ERROR;
        printf("ACK_ERROR\n");
    } else {
        ret = ACK_ERROR;
        printf("Wrong ACK format.\n");
    }
    return ret;
}

static void get_board_info (int fd)
{
    uint8_t buff[FRAME_MAX_BUFFER_LENGTH];
    FrameData_t f;
    f.Code = MC_PROTOCOL_CODE_GET_BOARD_INFO;
    f.Size = 0;
    f.nCRC = CalcCRC(&f);
 
    sentFrame(fd, &f);
    receivedFrame(fd, buff);
    printf("Board info: %s\n", buff);
} 

static void get_reg (const int fd, int reg, uint8_t *buff)
{
    // Pattern: 22 01 22 45
    uint16_t i, size;
    FrameData_t f;

    /* Protocol version >3.3 motor selection inside Frame ID */
    //  uint8_t bMotorSelection = (Code & 0xE0) >> 5; /* Mask: 1110|0000 */
    f.Code = MC_PROTOCOL_CODE_GET_REG | MOTOR_1;
    f.Size = 1;
    f.Buffer[0] = reg;
    f.nCRC = CalcCRC(&f);

    sentFrame(fd, &f);
    size = receivedFrame(fd, buff);

    printf("Reg: 0x%02x value: ", reg);
    for (i = 0; i < size; i++) {
        printf("0x%02x ", buff[i]);
    }
    printf("\n");
}

static void set_reg (const int fd, uint8_t reg, int32_t val, TYPE_t type)
{
/*  [Pattern]
 
    [01/11/2017 10:42:36] Written data (COM1) 
        21 02 1d 25 65                                    !..%e            
    [01/11/2017 10:42:36] Read data (COM1) 
        f0 00 f0                                          ?.?             

    // Set value to 1600 (0x0640)

    [01/11/2017 11:09:12] Written data (COM1) 
        21 05 5b 40 06 00 00 c7                           !.[@...?         
    [01/11/2017 11:09:12] Read data (COM1) 
        f0 00 f0                                          ?.?              
 */
    uint16_t i, size, len;
    uint8_t buff[FRAME_MAX_BUFFER_LENGTH];
    FrameData_t f;

    switch (type) {
        case U8:
        case S8:
           len = 1;
           break;
        case U16:
        case S16:
           len = 2;
           break;
        case U32:
        case S32:
           len = 4;
           break;
        default:
           return;
    }
    /* Protocol version >3.3 motor selection inside Frame ID */
    //  uint8_t bMotorSelection = (Code & 0xE0) >> 5; /* Mask: 1110|0000 */
    f.Code = MC_PROTOCOL_CODE_SET_REG | MOTOR_1;
    f.Buffer[0] = reg;

    for (i = 0; i < len; i++) {
        f.Buffer[i+1] = (val >> (i * 8)) & 0xff;
    }
    f.Size = len + 1;
    f.nCRC = CalcCRC(&f);

    sentFrame(fd, &f);
    size = receivedFrame(fd, buff);

    printf("Set reg: 0x%02x Ack value: ", reg);
    for (i = 0; i < size; i++) {
        printf("0x%02x ", buff[i]);
    }
    printf("\n");
}

static void motor_ctrl (int fd, int op)
{
    int i;
    int size = 1;
    uint8_t buff[size];
    FrameData_t f;
    f.Code = MC_PROTOCOL_CODE_EXECUTE_CMD | MOTOR_1;

    switch (op) {
        case START_MOTOR:
            buff[0] = MC_PROTOCOL_CMD_START_MOTOR;
            break;
        case STOP_MOTOR:
            buff[0] = MC_PROTOCOL_CMD_STOP_MOTOR;
            break;
        default:
            return;
    }
    for (i = 0; i < size; i++) {
        f.Buffer[i] = buff[i];
    }
    f.Size = 1;
    f.nCRC = CalcCRC(&f);

    sentFrame(fd, &f);
    getAck(fd);
}

int main (int argc, char *argv[])
{
    uint8_t buff[FRAME_MAX_BUFFER_LENGTH];
    int fd = init_uart(USART);
    if (fd < 0) {
        printf ("Open %s failed. %s", USART, strerror(errno));
        return fd;
    } 
    get_board_info(fd);
    get_reg(fd, MC_PROTOCOL_REG_RUC_STAGE_NBR, buff);
    // Ramp final speed.
    set_reg(fd, 0x5b, 1600, S32);

    motor_ctrl(fd, START_MOTOR);
    sleep(5);
    get_reg(fd, MC_PROTOCOL_REG_SPEED_MEAS, buff);
    int32_t rmp = buff[0] | (buff[1] << 8) | (buff[2] << 16) | (buff[3] << 24);
    printf("Motor speed measured: %d RPM\n", rmp);
    sleep(3);
    motor_ctrl(fd, STOP_MOTOR);

    tcsetattr(fd, TCSAFLUSH, &orig_tty);
    close(fd);
    return 0;
}

