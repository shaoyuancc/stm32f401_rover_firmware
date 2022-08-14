/********************************************************************
* This is a library for the 9-axis gyroscope, accelerometer and magnetometer MPU9250.
*
* You'll find an example which should enable you to use the library.
*
* You are free to use it, change it or build on it. In case you like
* it, it would be cool if you give it a star.
*
* If you find bugs, please inform me!
*
* Written by Wolfgang (Wolle) Ewald
*
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
*
* Modified by Shao Yuan to work with STM32F401 Black Pill Board
*
*********************************************************************/

#include "MPU9250_WE.h"


/* Registers AK8963 */
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_WIA         ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_INFO        ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_STATUS_1    ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_HXL         ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_HYL         ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_HZL         ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_STATUS_2    ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_CNTL_1      ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_CNTL_2      ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_ASTC        ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_I2CDIS      ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_ASAX        ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_ASAY        ;
uint8_t constexpr MPU9250_WE::REGISTER_AK8963_ASAZ        ;

/* Register Values */
uint8_t constexpr MPU9250_WE::REGISTER_VALUE_AK8963_16_BIT;
uint8_t constexpr MPU9250_WE::REGISTER_VALUE_AK8963_OVF   ;
uint8_t constexpr MPU9250_WE::REGISTER_VALUE_AK8963_READ  ;

/* Others */
uint8_t constexpr MPU9250_WE::WHO_AM_I_CODE               ;
uint8_t constexpr MPU9250_WE::MAGNETOMETER_I2C_ADDRESS    ;
uint8_t constexpr MPU9250_WE::MAGNETOMETER_WHO_AM_I_CODE  ;


/************  Constructors ************/

MPU9250_WE::MPU9250_WE(I2C_HandleTypeDef * i2cHandle, uint8_t addr)
    : MPU6500_WE(i2cHandle, addr)
{
    // intentionally empty
}

//MPU9250_WE::MPU9250_WE()
//    : MPU6500_WE()
//{
//    // intentionally empty
//}

//MPU9250_WE::MPU9250_WE(SPIClass *s, int cs, bool spi)
//    : MPU6500_WE(s, cs, spi)
//{
//    // intentionally empty
//}

/************ Basic Settings ************/

bool MPU9250_WE::init(){
    return MPU6500_WE::init(WHO_AM_I_CODE);
}

/************* x,y,z results *************/

xyzFloat MPU9250_WE::getMagValues(){
    xyzFloat magVal;

    uint64_t const xyzDataReg = readAK8963Data();
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);

    float constexpr scaleFactor = 4912.0 / 32760.0;

    magVal.x = xRaw * scaleFactor * magCorrFactor.x;
    magVal.y = yRaw * scaleFactor * magCorrFactor.y;
    magVal.z = zRaw * scaleFactor * magCorrFactor.z;

    return magVal;
}

/************** Magnetometer **************/

bool MPU9250_WE::initMagnetometer(){
    enableI2CMaster();
    resetMagnetometer();

    uint8_t wai = whoAmIMag();
    if(!(whoAmIMag() == MAGNETOMETER_WHO_AM_I_CODE)){
        return false;
    }
    setMagOpMode(AK8963_FUSE_ROM_ACC_MODE);
    HAL_Delay(10);
    getAsaVals();
    HAL_Delay(10);
    setMagnetometer16Bit();
    HAL_Delay(10);
    setMagOpMode(AK8963_CONT_MODE_8HZ);
    HAL_Delay(10);

    return true;
}

uint8_t MPU9250_WE::whoAmIMag(){
    return readAK8963Register8(REGISTER_AK8963_WIA);
}

void MPU9250_WE::setMagOpMode(AK8963_opMode opMode){
    uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
    regVal &= 0xF0;
    regVal |= opMode;
    writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
    HAL_Delay(10);
    if(opMode!=AK8963_PWR_DOWN){
        enableMagDataRead(REGISTER_AK8963_HXL, 0x08);
    }
}

void MPU9250_WE::startMagMeasurement(){
    setMagOpMode(AK8963_TRIGGER_MODE);
    HAL_Delay(200);
}

/************************************************
     Private Functions
*************************************************/

void MPU9250_WE::enableMagDataRead(uint8_t reg, uint8_t bytes){
    writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS | REGISTER_VALUE_AK8963_READ); // read AK8963
    writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be read
    writeMPU9250Register(REGISTER_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
    HAL_Delay(10);
}

void MPU9250_WE::resetMagnetometer(){
    writeAK8963Register(REGISTER_AK8963_CNTL_2, 0x01);
    HAL_Delay(100);
}

void MPU9250_WE::getAsaVals(){
    uint8_t rawCorr = 0;
    rawCorr = readAK8963Register8(REGISTER_AK8963_ASAX);
    magCorrFactor.x = (0.5 * (rawCorr-128)/128.0) + 1.0;
    rawCorr = readAK8963Register8(REGISTER_AK8963_ASAY);
    magCorrFactor.y = (0.5 * (rawCorr-128)/128.0) + 1.0;
    rawCorr = readAK8963Register8(REGISTER_AK8963_ASAZ);
    magCorrFactor.z = (0.5 * (rawCorr-128)/128.0) + 1.0;
}

void MPU9250_WE::writeAK8963Register(uint8_t reg, uint8_t val){
    writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS); // write AK8963
    writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be written to
    writeMPU9250Register(REGISTER_I2C_SLV0_DO, val);
}

uint8_t MPU9250_WE::readAK8963Register8(uint8_t reg){
    enableMagDataRead(reg, 0x01);
    uint8_t const regVal = readMPU9250Register8(REGISTER_EXT_SLV_SENS_DATA_00);
    enableMagDataRead(REGISTER_AK8963_HXL, 0x08);

    return regVal;
}

uint64_t MPU9250_WE::readAK8963Data(){
    uint8_t magByte[6];
    uint64_t regValue = 0;

    if (HAL_I2C_Mem_Read(_i2cHandle, i2cAddress, MPU9250_EXT_SLV_SENS_DATA_00, 1, magByte, 6, I2C_TIMEOUT) == HAL_OK){
        regValue = ((uint64_t) magByte[1]<<40) + ((uint64_t) magByte[0]<<32) +((uint64_t) magByte[3]<<24) +
             + ((uint64_t) magByte[2]<<16) + ((uint64_t) magByte[5]<<8) +  (uint64_t) magByte[4];

    }

    return regValue;

//    if(!useSPI){
//    }
//    else{
//        uint8_t reg = MPU9250_EXT_SLV_SENS_DATA_00 | 0x80;
//        _spi->beginTransaction(mySPISettings);
//        digitalWrite(csPin, LOW);
//        _spi->transfer(reg);
//        for(int i=0; i<6; i++){
//                magByte[i] = _spi->transfer(0x00);
//        }
//        digitalWrite(csPin, HIGH);
//        _spi->endTransaction();
//    }
}

void MPU9250_WE::setMagnetometer16Bit(){
    uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
    regVal |= REGISTER_VALUE_AK8963_16_BIT;
    writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
}

uint8_t MPU9250_WE::getStatus2Register(){
    return readAK8963Register8(REGISTER_AK8963_STATUS_2);
}


