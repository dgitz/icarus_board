/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-10-24 06:49:44.094030***/
/***Target: Arduino ***/
#ifndef SPIMESSAGE_H
#define SPIMESSAGE_H
#define BYTE2_OFFSET 32768
#define SPI_Diagnostic_ID 0x12
#define SPI_TestMessageCounter_ID 0x14
#define SPI_Get_DIO_Port1_ID 0x19
#define SPI_Get_ANA_Port1_ID 0x20
#define SPI_Get_IMUAcc_ID 0x27
#define SPI_Get_IMUGyro_ID 0x28
#define SPI_Get_IMUMag_ID 0x29
#define SPI_LEDStripControl_ID 0x42

int encode_DiagnosticSPI(unsigned char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message);

int encode_TestMessageCounterSPI(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12);

int encode_Get_DIO_Port1SPI(unsigned char* outbuffer,int* length,unsigned int u1,unsigned int u2);

int encode_Get_ANA_Port1SPI(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value);

int encode_Get_IMUAccSPI(unsigned char* outbuffer,int* length,unsigned int acc1_x,unsigned int acc1_y,unsigned int acc1_z,unsigned int acc2_x,unsigned int acc2_y,unsigned int acc2_z);

int encode_Get_IMUGyroSPI(unsigned char* outbuffer,int* length,unsigned int gyro1_x,unsigned int gyro1_y,unsigned int gyro1_z,unsigned int gyro2_x,unsigned int gyro2_y,unsigned int gyro2_z);

int encode_Get_IMUMagSPI(unsigned char* outbuffer,int* length,unsigned int mag1_x,unsigned int mag1_y,unsigned int mag1_z,unsigned int mag2_x,unsigned int mag2_y,unsigned int mag2_z);

int decode_LEDStripControlSPI(unsigned char* inbuffer,int* length,unsigned char checksum,unsigned char * LEDPixelMode,unsigned char * Param1,unsigned char * Param2);
#endif