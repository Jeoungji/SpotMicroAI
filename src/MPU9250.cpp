/**
 * Invensense MPU-9250 library using the SPI interface
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT License. See LICENSE.txt.
 */

#include <SPI.h>
#include <MPU9250.h>
#include <main.h>

bool MPU9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
    select();
    spi->transfer(WriteAddr);
    spi->transfer(WriteData);
    deselect();

    delayMicroseconds(1000);

    uint8_t read = ReadReg(WriteAddr);
    if (WriteData != read) {
        #if MPU9250DEBUG==1
            Serial.print("FAIL MPU9250 Write : Addr ");
            Serial.print(WriteAddr,HEX);
            Serial.print(" setting ");
            Serial.print(WriteData,HEX);
            Serial.print(" setted ");
            Serial.println(read,HEX);
        #endif
        return false;
    }
    delayMicroseconds(1000);
    return true;
}
uint8_t  MPU9250::ReadReg( uint8_t ReadAddr)
{
    unsigned int temp_val = 0;

    select();
    spi->transfer(ReadAddr| READ_FLAG);
    temp_val=spi->transfer(0x00);
    deselect();

    //delayMicroseconds(50);
    return temp_val;
}
void MPU9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;

    select();
    spi->transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = spi->transfer(0x00);
    deselect();

    //delayMicroseconds(50);
}


/*                                     INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ 
 * BITS_DLPF_CFG_5HZ 
 * BITS_DLPF_CFG_2100HZ_NOLPF
 * returns 1 if an error occurred
 */

#define MPU_InitRegNum 15

bool MPU9250::init(bool calib_gyro, bool calib_acc){
    pinMode(my_cs, OUTPUT);
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, HIGH);
#else
    digitalWrite(my_cs, HIGH);
#endif
    float temp[3];

    if(calib_gyro && calib_acc){
        calibrate(g_bias, a_bias);
    }
    else if(calib_gyro){
        calibrate(g_bias, temp);
    }
    else if(calib_acc){
        calibrate(temp, a_bias);
    }
    
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},               // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
        {my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        // {BITS_FS_500DPS, MPUREG_GYRO_CONFIG},    // +-250dps
        // {BITS_FS_4G, MPUREG_ACCEL_CONFIG},       // +-2G
        {my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x12, MPUREG_INT_PIN_CFG},      //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x30, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
        {0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
        {0x16, MPUREG_I2C_SLV0_DO},   // Register value to 100Hz continuous measurement in 16bit
#else
        {0x12, MPUREG_I2C_SLV0_DO},   // Register value to 8Hz continuous measurement in 16bit
#endif
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        
    };

    for(i = 0; i < MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        //delayMicroseconds(1000);  // I2C must slow down the write speed, otherwise it won't work
    }

    // set_acc_scale(BITS_FS_4G);
    // set_gyro_scale(BITS_FS_500DPS);
    
    delayMicroseconds(1000);

    return 0;
}

/*                          auto_init
    return value:
    0  : Clear Initialization
    1   : whoami Error
    2   : accel calibrate Error
    3   : gyro calibrate Error
*/

int MPU9250::auto_init(uint8_t AccelScale, uint8_t GyroScale) {
    spi->begin();
    delayMicroseconds(1000);

    init(false, false);

	uint8_t wai = whoami();
	if (wai == 0x71){
        #if MPU9250DEBUG==1 && DEBUGINFO==1
        Serial.println("SUCC MPU9250 connection");
        #endif
	}
	else{
        #if MPU9250DEBUG==1
		Serial.print("FAIL MPU9250 connection: ");
		Serial.println(wai, HEX);
        #endif
        return 1;
	}
    set_acc_scale(AccelScale);
    set_gyro_scale(GyroScale);

    if (!accel_calibrate(100))  {
        #if MPU9250DEBUG==1
		Serial.println("FAIL MPU9250 accel_calibrate: ");
        #endif
        return 2;
    }
    //delay(10);
    calib_acc();
    if (!gyro_calibrate(100)) {
        #if MPU9250DEBUG==1
		Serial.println("FAIL MPU9250 gyro_calibrate: ");
        #endif
        return 3;
    }
    delay(10);
    return 0;
}

/*                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */

uint16_t MPU9250::set_acc_scale(uint8_t scale){
    uint8_t temp_scale;
    uint16_t return_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale);

    switch (scale){
        case BITS_FS_2G:    acc_divider=16384;  break;
        case BITS_FS_4G:    acc_divider=8192;   break;
        case BITS_FS_8G:    acc_divider=4096;   break;
        case BITS_FS_16G:   acc_divider=2048;   break;   
    }

    temp_scale = ReadReg(MPUREG_ACCEL_CONFIG);
    switch (temp_scale){
        case BITS_FS_2G:    return_scale=2;     break;
        case BITS_FS_4G:    return_scale=4;     break;
        case BITS_FS_8G:    return_scale=8;     break;
        case BITS_FS_16G:   return_scale=16;    break;   
        default:            return_scale=0xFFFF;break;
    }

#if MPU9250DEBUG==1 && DEBUGDATA==1
    Serial.print("INFO MPU9250 accel scale : set ");
    Serial.print(scale, HEX);
    Serial.print("  temp_scale : ");
    Serial.println(temp_scale, HEX);
#endif
    return return_scale;
}



/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */

uint16_t MPU9250::set_gyro_scale(uint8_t scale){
    uint8_t temp_scale;
    uint16_t return_scale;
    WriteReg(MPUREG_GYRO_CONFIG, scale);

    switch (scale){
        case BITS_FS_250DPS:   gyro_divider = 131;  break;
        case BITS_FS_500DPS:   gyro_divider = 65.5; break;
        case BITS_FS_1000DPS:  gyro_divider = 32.8; break;
        case BITS_FS_2000DPS:  gyro_divider = 16.4; break;   
    }

    temp_scale = ReadReg(MPUREG_GYRO_CONFIG);

    switch (temp_scale){
        case BITS_FS_250DPS:   return_scale = 250;    break;
        case BITS_FS_500DPS:   return_scale = 500;    break;
        case BITS_FS_1000DPS:  return_scale = 1000;   break;
        case BITS_FS_2000DPS:  return_scale = 2000;   break;   
        default:               return_scale = 0xFFFF; break;
    }

    #if MPU9250DEBUG==1 && DEBUGDATA==1
    Serial.print("INFO MPU9250 gyro scale : set ");
    Serial.print(scale, HEX);
    Serial.print("  temp_scale : ");
    Serial.println(temp_scale, HEX);
    #endif

    return return_scale;
}



/*                                 WHO AM I?
 * usage: call this function to know if SPI is working correctly. It checks the I2C address of the
 * mpu9250 which should be 0x71
 */

unsigned int MPU9250::whoami(){
    unsigned int response;
    response = ReadReg(MPUREG_WHOAMI);
    return response;
}



/*                                 READ ACCELEROMETER
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250::read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    int i;
    
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8)|response[i*2+1];
        accel_lowdata[i] = (float)bit_data;
        accel_data[i] = (accel_lowdata[i]/acc_divider) - a_bias[i];
    }
}

/*                                 READ GYROSCOPE
 * usage: call this function to read gyroscope data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250::read_gyro()
{
    uint8_t response[6];
    int16_t bit_data;
    int i;
    ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        gyro_lowdata[i] = (float)bit_data;
        gyro_data[i] = gyro_lowdata[i]/gyro_divider - g_bias[i];
    }

}


/*                                 READ temperature
 * usage: call this function to read temperature data. 
 * returns the value in °C
 */

void MPU9250::read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    ReadRegs(MPUREG_TEMP_OUT_H,response,2);

    bit_data = ((int16_t)response[0]<<8)|response[1];
    data = (float)bit_data;
    temperature = (data/333.87)+21.0;
    deselect();
}

/*                                 READ ACCELEROMETER CALIBRATION
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 * returns Factory Trim value
 */

void MPU9250::calib_acc()
{
    uint8_t response[4];
    //READ CURRENT ACC SCALE
    uint8_t temp_scale=ReadReg(MPUREG_ACCEL_CONFIG);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0] = ((response[0]&11100000)>>3) | ((response[3]&00110000)>>4);
    calib_data[1] = ((response[1]&11100000)>>3) | ((response[3]&00001100)>>2);
    calib_data[2] = ((response[2]&11100000)>>3) | ((response[3]&00000011));

#if MPU9250DEBUG==1 && DEBUGDATA==1
    Serial.print("INFO MPU9250 Accel_config : ");
    Serial.print(temp_scale, HEX);
    Serial.print("  calib_acc : ");
    Serial.print(calib_data[0]);    Serial.print(" ");
    Serial.print(calib_data[1]);    Serial.print(" ");
    Serial.print(calib_data[2]);    Serial.println(" ");
#endif
}

void MPU9250::read_all(){
    read_acc();
    read_gyro();
    read_temp();
}

void MPU9250::calibrate(float *dest1, float *dest2){  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
    // reset device
    WriteReg(MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);
   
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    WriteReg(MPUREG_PWR_MGMT_1, 0x01);  
    WriteReg(MPUREG_PWR_MGMT_2, 0x00);
    delay(200);                                    

    // Configure device for bias calculation
    WriteReg(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    WriteReg(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    WriteReg(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteReg(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    WriteReg(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);
  
    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteReg(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    WriteReg(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    WriteReg(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    WriteReg(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
    
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteReg(MPUREG_USER_CTRL, 0x40);   // Enable FIFO  
    WriteReg(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    WriteReg(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
    
    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadRegs(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
            
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
    // Push gyro biases to hardware registers
    WriteReg(MPUREG_XG_OFFS_USRH, data[0]);
    WriteReg(MPUREG_XG_OFFS_USRL, data[1]);
    WriteReg(MPUREG_YG_OFFS_USRH, data[2]);
    WriteReg(MPUREG_YG_OFFS_USRL, data[3]);
    WriteReg(MPUREG_ZG_OFFS_USRH, data[4]);
    WriteReg(MPUREG_ZG_OFFS_USRL, data[5]);
  
    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    ReadRegs(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadRegs(MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadRegs(MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    
    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
    
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
    WriteReg(MPUREG_XA_OFFSET_H, data[0]);
    WriteReg(MPUREG_XA_OFFSET_L, data[1]);
    WriteReg(MPUREG_YA_OFFSET_H, data[2]);
    WriteReg(MPUREG_YA_OFFSET_L, data[3]);
    WriteReg(MPUREG_ZA_OFFSET_H, data[4]);
    WriteReg(MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

bool MPU9250::accel_calibrate(int sample) {
#if MPU9250DEBUG==1 && DEBUGINFO==1
    Serial.println ("INFO MPU9250 Start Accel Calibration");
#endif

    WriteReg(MPUREG_XA_OFFSET_H, 0x00);
    WriteReg(MPUREG_XA_OFFSET_L, 0x00);
    WriteReg(MPUREG_YA_OFFSET_H, 0x00);
    WriteReg(MPUREG_YA_OFFSET_L, 0x00);
    WriteReg(MPUREG_ZA_OFFSET_H, 0x00);
    WriteReg(MPUREG_ZA_OFFSET_L, 0x00);

    a_bias[0] = 0;
    a_bias[1] = 0;
    a_bias[2] = 0;
    float local_a_bias[3] = {0, 0, 0};

    for (int i = 0; i < sample; i++) {
        read_acc();
        for (int n = 0; n < 3; n++)
            local_a_bias[n] += accel_data[n];
        delayMicroseconds(1000);
    }

    for (int n = 0; n < 3; n++)
        a_bias[n] = local_a_bias[n] / (float)sample;

#if MPU9250DEBUG==1 && DEBUGINFO==1
    Serial.println ("INFO MPU9250 Reset Accel Offset Parameters");
#endif
#if MPU9250DEBUG==1 && DEBUGDATA==1
    Serial.print("INFO MPU9250 accel bias : ");
    Serial.print(a_bias[0]);    Serial.print(" ");
    Serial.print(a_bias[1]);    Serial.print(" ");
    Serial.print(a_bias[2]);    Serial.println(" ");
#endif
    
    read_acc();
    //a_bias[0] -= 0.45;
    if (accel_data[0] > 0.05 || accel_data[0] < -0.05) return false;
    if (accel_data[1] > 0.05 || accel_data[1] < -0.05) return false;
    if (accel_data[2] > 0.05 || accel_data[2] < -0.05) return false;

    a_bias[2] -= 1;
    
    return true;
}

bool MPU9250::gyro_calibrate(int sample) {

    WriteReg(MPUREG_XG_OFFS_USRH, 0x00);
    WriteReg(MPUREG_XG_OFFS_USRL, 0x00);
    WriteReg(MPUREG_YG_OFFS_USRH, 0x00);
    WriteReg(MPUREG_YG_OFFS_USRL, 0x00);
    WriteReg(MPUREG_ZG_OFFS_USRH, 0x00);
    WriteReg(MPUREG_ZG_OFFS_USRL, 0x00);

    g_bias[0] = 0;
    g_bias[1] = 0;
    g_bias[2] = 0;
    float local_g_bias[3] = {0, 0, 0};
    delayMicroseconds(1000);

    for (int i = 0; i < sample; i++) {
        read_gyro();
        for (int n = 0; n < 3; n++)
            local_g_bias[n] += gyro_data[n];
        delayMicroseconds(1000);
    }

    for (int n = 0; n < 3; n++)
        g_bias[n] = local_g_bias[n] / (float)sample;

#if MPU9250DEBUG==1 && DEBUGINFO==1
    Serial.println ("INFO MPU9250 Reset Gyro Offset Parameters");
#endif
#if MPU9250DEBUG==1 && DEBUGDATA==1

    Serial.print("INFO MPU9250 gyro bias : ");
    Serial.print(g_bias[0]);    Serial.print(" ");
    Serial.print(g_bias[1]);    Serial.print(" ");
    Serial.print(g_bias[2]);    Serial.println(" ");
    delay(500);
#endif
    local_g_bias[0] = 0;
    local_g_bias[1] = 0;
    local_g_bias[2] = 0;
    for (int i = 0; i < sample; i++) {
        read_gyro();
        for (int n = 0; n < 3; n++)
            local_g_bias[n] += gyro_data[n];
        delayMicroseconds(1000);
    }
    if (local_g_bias[0]/sample > 1 || local_g_bias[0]/sample < -1) return false;
    if (local_g_bias[1]/sample > 1 || local_g_bias[1]/sample < -1) return false;
    if (local_g_bias[2]/sample > 1 || local_g_bias[2]/sample < -1) return false;

    return true;
}

void MPU9250::select() {
    //Set CS low to start transmission (interrupts conversion)
    spi->beginTransaction(SPISettings(my_clock, MSBFIRST, SPI_MODE3));
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, LOW);
#else
    digitalWrite(my_cs, LOW);
#endif
}
void MPU9250::deselect() {
    //Set CS high to stop transmission (restarts conversion)
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, HIGH);
#else
    digitalWrite(my_cs, HIGH);
#endif
    spi->endTransaction();
}


void MPU9250::init_Kalman() {
    KalmanAngle.Roll = 0;
    KalmanUncertaintyAngle.Roll = 2*2;
    KalmanAngle.Pitch = 0;
    KalmanUncertaintyAngle.Pitch = 2*2;
    Kalman1DOutput[0] = 0;
    Kalman1DOutput[1] = 0;

    kalmantimes = 0;
}

void MPU9250::CarculateAngle(float accel[3]){
    AngleRoll = atan(accel[1]/sqrt(accel[0]*accel[0] + accel[2]*accel[2]))*1/ (PI/180);
    AnglePitch = -atan(accel[0]/sqrt(accel[1]*accel[1] + accel[2]*accel[2]))*1/ (PI/180);
}

void MPU9250::kalman_1d(float KalmanState, float KalmanUncertainty,
    float KalmanInput, float KalmanMeasurement, float deltatime)
{
    KalmanState = KalmanState + deltatime * KalmanInput; // carculate gyro to predict angle
    KalmanUncertainty = KalmanUncertainty + deltatime * deltatime * 4 * 4;

    float KalmanGain = KalmanUncertainty * 1/ (1*KalmanUncertainty + 3*3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

ANGLE MPU9250::CalKalmanAngle(uint32_t time) {
    #if MPU9250DEBUG==1 && DEBUGTIME==1
    uint32_t mtime = micros(); // calculate time test, microsecons
    #endif
    read_acc();
    read_gyro();

    CarculateAngle(accel_data);

    if (time < kalmantimes) time = time + (0xFFFFFFFF-kalmantimes); // time overflow
    float deltatime = ((float)(time- kalmantimes) / 1000);
    
    kalmantimes = time;
#if MPU9250DEBUG==1 && DEBUGTIME==1
    Serial.print("INFO NPU9250 kalman delta time : "); Serial.println(deltatime*1000);
#endif

    // Roll
    kalman_1d(KalmanAngle.Roll, KalmanUncertaintyAngle.Roll, gyro_data[0], AngleRoll, deltatime);
    KalmanAngle.Roll= Kalman1DOutput[0];
    KalmanUncertaintyAngle.Roll = Kalman1DOutput[1];
    // Pitch
    kalman_1d(KalmanAngle.Pitch, KalmanUncertaintyAngle.Pitch, gyro_data[1], AnglePitch, deltatime);
    KalmanAngle.Pitch = Kalman1DOutput[0];
    KalmanUncertaintyAngle.Pitch = Kalman1DOutput[1];

#if MPU9250DEBUG==1 && DEBUGTIME==1
    uint32_t d_mtime = micros()-mtime;
    Serial.print("INFO NPU9250 MPU9250 calculate time : "); Serial.print(d_mtime); Serial.println("us");
#endif
	
    return KalmanAngle;
}