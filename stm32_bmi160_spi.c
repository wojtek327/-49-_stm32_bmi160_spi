#include "stm32_bmi160_spi.h"

#define _BMI160_CS_ENABLE()			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
#define _BMI160_CS_DISABLE()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

#define BMI160_SIGN_EXTEND(val, from) \
    (((val) & (1 << ((from) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (from))) - 1) << (from))) : val)

extern SPI_HandleTypeDef hspi1;

static HAL_StatusTypeDef BMI160_SPI_WriteRegister(uint8_t address, uint8_t cmd);
static HAL_StatusTypeDef BMI160_SPI_ReadRegister(uint8_t address, uint8_t dataSize, uint8_t *rec);
static HAL_StatusTypeDef BMI160_WriteSPI(const uint8_t registerAddr, uint8_t *bufferPtr, const uint8_t bufferSize);
static HAL_StatusTypeDef BMI160_ReadSPI(const uint8_t registerAddr, uint8_t *bufferPtr, const uint8_t bufferSize);
static HAL_StatusTypeDef BMI160_SPI_WriteRegisterBits(uint8_t reg, uint8_t data, unsigned pos, unsigned len);
static uint8_t BMI160_SPI_ReadRegBits(uint8_t reg, unsigned pos, unsigned len);
static HAL_StatusTypeDef BMI160_ReadWriteSPI(const uint8_t registerAddr, uint8_t *bufferPtr, const uint8_t bufferSize);

HAL_StatusTypeDef (*BMI160_WriteToRegister)(uint8_t, uint8_t);
HAL_StatusTypeDef (*BMI160_ReadFromRegister)(uint8_t, uint8_t, uint8_t*);

typedef struct BMI160_CommuniFunct bmi160_funct;

HAL_StatusTypeDef BMI160_SPI_Initialize(void)
{
	HAL_StatusTypeDef opStatus = HAL_ERROR;

	BMI160_WriteToRegister = BMI160_SPI_WriteRegister;
	BMI160_ReadFromRegister = BMI160_SPI_ReadRegister;

	opStatus = BMI160_SPI_PerformSoftReset();
	if(opStatus != HAL_OK) { return opStatus; }

	HAL_Delay(300);

	if(BMI160_SPI_CheckSensorID() == 0) { return HAL_ERROR; }

	opStatus = BMI160_SPI_PowerUpAccelerometer();
	if(opStatus != HAL_OK) { return opStatus; }

	opStatus = BMI160_SPI_PowerUpGyroscope();
	if(opStatus != HAL_OK) { return opStatus; }

	opStatus = BMI160_SPI_SetFullScaleGyroRange(BMI160_GYRO_RANGE_250);
	if(opStatus != HAL_OK) { return opStatus; }

	opStatus = BMI160_SPI_SetFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
	if(opStatus != HAL_OK) { return opStatus; }

    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = 0xFF;

    opStatus = BMI160_WriteSPI(BMI160_RA_INT_MAP_0, &dataToSend[0], 1);
	if(opStatus != HAL_OK) { return opStatus; }

    dataToSend[0] = 0xF0;
    opStatus = BMI160_WriteSPI(BMI160_RA_INT_MAP_1, &dataToSend[0], 1);
	if(opStatus != HAL_OK) { return opStatus; }

    dataToSend[0] = 0x00;
    opStatus = BMI160_WriteSPI(BMI160_RA_INT_MAP_2, &dataToSend[0], 1);
	if(opStatus != HAL_OK) { return opStatus; }

	return HAL_OK;
}

HAL_StatusTypeDef BMI160_SPI_PerformSoftReset(void)
{
	//uint8_t buffer[1] = {BMI160_CMD_SOFT_RESET};
	//HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_CMD, &buffer[0], 1);
	//HAL_StatusTypeDef opStatus = BMI160_SPI_WriteRegister(BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);

	HAL_StatusTypeDef opStatus = BMI160_WriteToRegister(BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);
	return opStatus;
}

uint8_t BMI160_SPI_ReadChipID(void)
{
	uint8_t array[1] = {0x00};
	//HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_CHIP_ID, &array[0], 1);
	//HAL_StatusTypeDef opStatus = BMI160_SPI_ReadRegister(BMI160_RA_CHIP_ID, 1, &array[0]);
	HAL_StatusTypeDef opStatus = BMI160_ReadFromRegister(BMI160_RA_CHIP_ID, 1, &array[0]);
	if(opStatus != HAL_OK) { return (uint8_t)opStatus; }

	return array[0];
}

HAL_StatusTypeDef BMI160_SPI_PowerUpAccelerometer(void)
{
	uint8_t buffer[1] = {BMI160_CMD_ACC_MODE_NORMAL};
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_CMD, &buffer[0], 1);

	if(opStatus != HAL_OK) { return opStatus; }

	HAL_Delay(2);

	while(0x1 != BMI160_SPI_ReadRegBits(BMI160_RA_PMU_STATUS, BMI160_ACC_PMU_STATUS_BIT, BMI160_ACC_PMU_STATUS_LEN)) { HAL_Delay(1); }

	return HAL_OK;
}

HAL_StatusTypeDef BMI160_SPI_PowerUpGyroscope(void)
{
	uint8_t buffer[1] = {BMI160_CMD_GYR_MODE_NORMAL};
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_CMD, &buffer[0], 1);

	if(opStatus != HAL_OK) { return opStatus; }

	HAL_Delay(2);

	while(0x1 != BMI160_SPI_ReadRegBits(BMI160_RA_PMU_STATUS, BMI160_GYR_PMU_STATUS_BIT, BMI160_GYR_PMU_STATUS_LEN)) { HAL_Delay(1); }

	return HAL_OK;
}

uint8_t BMI160_SPI_CheckSensorID(void) {
	uint8_t readedChipId = BMI160_SPI_ReadChipID();

	if(readedChipId == SENSOR_CHIP_ID_BMI160) { return 1; }
	else if(readedChipId == SENSOR_CHIP_ID_BMI160_C2) { return 2; }
	else if(readedChipId == SENSOR_CHIP_ID_BMI160_C3) { return 3; }

	return 0;
}

HAL_StatusTypeDef BMI160_SPI_ReadGyro(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t buffer[6] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_GYRO_X_L, &buffer[0], 6);
    int16_t xa = 0;
	int16_t ya = 0;
	int16_t za = 0;

    //*(x+0) = (((int16_t)buffer[1]) << 8) | buffer[0];
    //*(y+0) = (((int16_t)buffer[3]) << 8) | buffer[2];
    //*(z+0) = (((int16_t)buffer[5]) << 8) | buffer[4];

	//xa = (((int16_t)buffer[1]) << 8) | buffer[0];
	//ya = (((int16_t)buffer[3]) << 8) | buffer[2];
	//za = (((int16_t)buffer[5]) << 8) | buffer[4];

    return opStat;
}

float BMI160_SPI_ScaledData(const int16_t gRaw, const BMI160GyroRange gyroRange) {
  float g = 0.0F;

  if(gyroRange == BMI160_GYRO_RANGE_2000) { g = (gRaw / SENS_2000_DPS_LSB_PER_DPS); }
  else if(gyroRange == BMI160_GYRO_RANGE_1000) { g = (gRaw / SENS_1000_DPS_LSB_PER_DPS); }
  else if(gyroRange == BMI160_GYRO_RANGE_500) { g = (gRaw / SENS_500_DPS_LSB_PER_DPS); }
  else if(gyroRange == BMI160_GYRO_RANGE_250) { g = (gRaw / SENS_250_DPS_LSB_PER_DPS); }
  else if(gyroRange == BMI160_GYRO_RANGE_125) { g = (gRaw / SENS_125_DPS_LSB_PER_DPS); }

  return g;
}

int16_t BMI160_SPI_GetTemperature(void) {
    uint8_t buffer[2] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_TEMP_L, &buffer[0], 2);

    if(opStat != HAL_OK) { return HAL_ERROR; }
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

float BMI160_SPI_ConvertRawTemp(int16_t tempRaw)
{
	float convertTemp = 0;

    if(tempRaw & 0x8000) { convertTemp = (23.0F - ((0x10000 - tempRaw)/512.0F)); }
    else { convertTemp = ((tempRaw/512.0F) + 23.0F); }

    return convertTemp;
}

int16_t BMI160_SPI_GetAccelerationX(void) {
    uint8_t buffer[2] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_ACCEL_X_L, &buffer[0], 2);

    if(opStat != HAL_OK) { return HAL_ERROR; }
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

int16_t BMI160_SPI_GetAccelerationY(void) {
    uint8_t buffer[2] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_ACCEL_Y_L, &buffer[0], 2);

    if(opStat != HAL_OK) { return HAL_ERROR; }

    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

int16_t BMI160_SPI_GetAccelerationZ(void) {
    uint8_t buffer[2] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_ACCEL_Z_L, &buffer[0], 2);

    if(opStat != HAL_OK) { return HAL_ERROR; }

    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

HAL_StatusTypeDef BMI160_SPI_GetAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t buffer[6] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_ACCEL_X_L, &buffer[0], 6);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];

    return opStat;
}

HAL_StatusTypeDef BMI160_SPI_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[12] = {0x00};

    HAL_StatusTypeDef opStat = BMI160_ReadSPI(BMI160_RA_GYRO_X_L, &buffer[0], 12);

    *gx = (((int16_t)buffer[1])  << 8) | buffer[0];
    *gy = (((int16_t)buffer[3])  << 8) | buffer[2];
    *gz = (((int16_t)buffer[5])  << 8) | buffer[4];
    *ax = (((int16_t)buffer[7])  << 8) | buffer[6];
    *ay = (((int16_t)buffer[9])  << 8) | buffer[8];
    *az = (((int16_t)buffer[11]) << 8) | buffer[10];

    return opStat;
}

BMI160GyroRate BMI160_SPI_GetGyroRate(void) {
	return (BMI160GyroRate)BMI160_SPI_ReadRegBits(BMI160_RA_GYRO_CONF, BMI160_GYRO_RATE_SEL_BIT, BMI160_GYRO_RATE_SEL_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetGyroRate(const BMI160GyroRate rate) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_GYRO_CONF, (uint8_t)rate, BMI160_GYRO_RATE_SEL_BIT, BMI160_GYRO_RATE_SEL_LEN);
}

BMI160AccelRate BMI160_SPI_GetAccelRate(void) {
	return (BMI160AccelRate)BMI160_SPI_ReadRegBits(BMI160_RA_ACCEL_CONF, BMI160_ACCEL_RATE_SEL_BIT, BMI160_ACCEL_RATE_SEL_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetAccelRate(const BMI160AccelRate rate) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_ACCEL_CONF, (uint8_t)rate, BMI160_ACCEL_RATE_SEL_BIT, BMI160_ACCEL_RATE_SEL_LEN);
}

BMI160DLPFMode BMI160_SPI_GetGyroDLPFMode(void) {
    return (BMI160DLPFMode)BMI160_SPI_ReadRegBits(BMI160_RA_GYRO_CONF, BMI160_GYRO_DLPF_SEL_BIT, BMI160_GYRO_DLPF_SEL_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetGyroDLPFMode(BMI160DLPFMode mode) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_GYRO_CONF, mode, BMI160_GYRO_DLPF_SEL_BIT, BMI160_GYRO_DLPF_SEL_LEN);
}

BMI160DLPFMode BMI160_SPI_GetAccelDLPFMode(void) {
    return (BMI160DLPFMode)BMI160_SPI_ReadRegBits(BMI160_RA_ACCEL_CONF, BMI160_ACCEL_DLPF_SEL_BIT, BMI160_ACCEL_DLPF_SEL_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetAccelDLPFMode(BMI160DLPFMode mode) {
    return BMI160_SPI_WriteRegisterBits(BMI160_RA_ACCEL_CONF, mode, BMI160_ACCEL_DLPF_SEL_BIT, BMI160_ACCEL_DLPF_SEL_LEN);
}

BMI160GyroRange BMI160_SPI_GetFullScaleGyroRange(void) {
	return (BMI160GyroRange)BMI160_SPI_ReadRegBits(BMI160_RA_GYRO_RANGE, BMI160_GYRO_RANGE_SEL_BIT, BMI160_GYRO_RANGE_SEL_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetFullScaleGyroRange(const BMI160GyroRange range) {
    return BMI160_SPI_WriteRegisterBits(BMI160_RA_GYRO_RANGE, (uint8_t)range, BMI160_GYRO_RANGE_SEL_BIT, BMI160_GYRO_RANGE_SEL_LEN);
}

BMI160AccelRange BMI160_SPI_GetFullScaleAccelRange(void) {
	return (BMI160AccelRange)BMI160_SPI_ReadRegBits(BMI160_RA_ACCEL_RANGE, BMI160_ACCEL_RANGE_SEL_BIT, BMI160_ACCEL_RANGE_SEL_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetFullScaleAccelRange(const BMI160AccelRange range) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_ACCEL_RANGE, (uint8_t)range, BMI160_ACCEL_RANGE_SEL_BIT, BMI160_ACCEL_RANGE_SEL_LEN);
}

uint8_t BMI160_SPI_GetAccelOffsetEnabled(void) {
    return !!(BMI160_SPI_ReadRegBits(BMI160_RA_OFFSET_6, BMI160_ACC_OFFSET_EN, 1));
}

HAL_StatusTypeDef BMI160_SPI_SetAccelOffsetEnabled(const uint8_t enabled) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_OFFSET_6, enabled ? 0x1 : 0, BMI160_ACC_OFFSET_EN, 1);
}

HAL_StatusTypeDef BMI160_SPI_AutoCalibrateXAccelOffset(int target)
{
    uint8_t foc_conf = 0;
    if (target == 1) { foc_conf = (0x1 << BMI160_FOC_ACC_X_BIT); }
    else if (target == -1) { foc_conf = (0x2 << BMI160_FOC_ACC_X_BIT); }
    else if (target == 0) { foc_conf = (0x3 << BMI160_FOC_ACC_X_BIT); }
    else { return HAL_ERROR; }

    uint8_t commandToSend[1] = {0x00};
    commandToSend[0] = foc_conf;

    BMI160_WriteSPI(BMI160_RA_FOC_CONF, &commandToSend[0], 1);
    commandToSend[0] = BMI160_CMD_START_FOC;

    BMI160_WriteSPI(BMI160_RA_CMD, &commandToSend[0], 1);

    while (!(BMI160_SPI_ReadRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) { HAL_Delay(1); }

    return HAL_OK;
}

HAL_StatusTypeDef BMI160_SPI_AutoCalibrateYAccelOffset(int target) {
    uint8_t foc_conf = 0;
    if (target == 1) { foc_conf = (0x1 << BMI160_FOC_ACC_Y_BIT); }
    else if (target == -1) { foc_conf = (0x2 << BMI160_FOC_ACC_Y_BIT); }
    else if (target == 0) { foc_conf = (0x3 << BMI160_FOC_ACC_Y_BIT); }
    else { return HAL_ERROR; }

    uint8_t commandToSend[1] = {0x00};
    commandToSend[0] = foc_conf;
    BMI160_WriteSPI(BMI160_RA_FOC_CONF, &commandToSend[0], 1);

    commandToSend[0] = BMI160_CMD_START_FOC;
    BMI160_WriteSPI(BMI160_RA_CMD, &commandToSend[0], 1);

    while (!(BMI160_SPI_ReadRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) { HAL_Delay(1); }

    return HAL_OK;
}

HAL_StatusTypeDef BMI160_SPI_AutoCalibrateZAccelOffset(int target) {
    uint8_t foc_conf = 0;

    if (target == 1) { foc_conf = (0x1 << BMI160_FOC_ACC_Z_BIT); }
    else if (target == -1) { foc_conf = (0x2 << BMI160_FOC_ACC_Z_BIT); }
    else if (target == 0) { foc_conf = (0x3 << BMI160_FOC_ACC_Z_BIT); }
    else { return HAL_ERROR; }

    uint8_t commandToSend[1] = {0x00};
    commandToSend[0] = foc_conf;
    BMI160_WriteSPI(BMI160_RA_FOC_CONF, &commandToSend[0], 1);
    commandToSend[0] = BMI160_CMD_START_FOC;
    BMI160_WriteSPI(BMI160_RA_CMD, &commandToSend[0], 1);

    while (!(BMI160_SPI_ReadRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) { HAL_Delay(1); }

    return HAL_OK;
}

int8_t BMI160_SPI_GetXAccelOffset(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_OFFSET_0, &readData, 1);

	if(opStatus != HAL_OK) {
		return opStatus;
	}
    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetXAccelOffset(int8_t offset) {
	uint8_t buffer[1] = { offset };
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_OFFSET_0, &buffer[0], 1);

    return opStatus;
}

int8_t BMI160_SPI_GetYAccelOffset(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_OFFSET_1, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }
    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetYAccelOffset(int8_t offset) {
	uint8_t buffer[1] = { offset };
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_OFFSET_1, &buffer[0], 1);
    return opStatus;
}

int8_t BMI160_SPI_GetZAccelOffset(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_OFFSET_2, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }

    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetZAccelOffset(int8_t offset) {
	uint8_t buffer[1] = { offset };
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_OFFSET_2, &buffer[0], 1);

    return opStatus;
}

uint8_t BMI160_SPI_GetGyroOffsetEnabled(void) {
    return !!(BMI160_SPI_ReadRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_EN, 1));
}

HAL_StatusTypeDef BMI160_SPI_SetGyroOffsetEnabled(uint8_t enabled) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_OFFSET_6, enabled ? 0x1 : 0, BMI160_GYR_OFFSET_EN, 1);
}

HAL_StatusTypeDef BMI160_SPI_AutoCalibrateGyroOffset(void) {
    uint8_t foc_conf = (1 << BMI160_FOC_GYR_EN);

    uint8_t commandToSend[1] = {0x00};
    commandToSend[0] = foc_conf;
    BMI160_WriteSPI(BMI160_RA_FOC_CONF, &commandToSend[0], 1);
    commandToSend[0] = BMI160_CMD_START_FOC;
    BMI160_WriteSPI(BMI160_RA_CMD, &commandToSend[0], 1);

    while (!(BMI160_SPI_ReadRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) { HAL_Delay(1); }

    return HAL_OK;
}

int16_t BMI160_SPI_GetXGyroOffset(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_OFFSET_3, &readData, 1);

	if(opStatus != HAL_OK){ return opStatus; }

	int16_t offset = readData;

	offset |= (int16_t)(BMI160_SPI_ReadRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_X_MSB_BIT, BMI160_GYR_OFFSET_X_MSB_LEN)) << 8;
    return (int16_t)BMI160_SIGN_EXTEND(offset, 10);
}

HAL_StatusTypeDef BMI160_SPI_SetXGyroOffset(int16_t offset) {
    uint8_t commandToSend[1] = {0x00};
    commandToSend[0] = offset;

    HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_OFFSET_3, &commandToSend[0], 1);
	if(opStatus != HAL_OK) { return opStatus; }

	opStatus = BMI160_SPI_WriteRegisterBits(BMI160_RA_OFFSET_6, offset >> 8, BMI160_GYR_OFFSET_X_MSB_BIT, BMI160_GYR_OFFSET_X_MSB_LEN);

	if(opStatus != HAL_OK) { return opStatus; }
	return HAL_OK;
}

int16_t BMI160_SPI_GetYGyroOffset(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_OFFSET_4, &readData, 1);

	if(opStatus != HAL_OK){ return opStatus; }

	int16_t offset = readData;
    offset |= (int16_t)(BMI160_SPI_ReadRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_Y_MSB_BIT, BMI160_GYR_OFFSET_Y_MSB_LEN)) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

int16_t BMI160_SPI_GetZGyroOffset(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_OFFSET_5, &readData, 1);

	if(opStatus != HAL_OK){ return opStatus; }

	int16_t offset = readData;
    offset |= (int16_t)(BMI160_SPI_ReadRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_Z_MSB_BIT, BMI160_GYR_OFFSET_Z_MSB_LEN)) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

HAL_StatusTypeDef BMI160_SPI_SetZGyroOffset(int16_t offset) {
    uint8_t commandToSend[1] = {0x00};
    commandToSend[0] = offset;
    HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_OFFSET_5, &commandToSend[0], 1);

	if(opStatus != HAL_OK) { return opStatus; }

	opStatus = BMI160_SPI_WriteRegisterBits(BMI160_RA_OFFSET_6, offset >> 8, BMI160_GYR_OFFSET_Z_MSB_BIT, BMI160_GYR_OFFSET_Z_MSB_LEN);

	if(opStatus != HAL_OK) { return opStatus; }
	return HAL_OK;
}

uint8_t BMI160_SPI_GetFreefallDetectionThreshold(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_INT_LOWHIGH_1, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }
    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetFreefallDetectionThreshold(uint8_t threshold) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = threshold;

	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_INT_LOWHIGH_1, &dataToSend[0], 1);
    return opStatus;
}

uint8_t BMI160_SPI_GetFreefallDetectionDuration(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_INT_LOWHIGH_0, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }

    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetFreefallDetectionDuration(uint8_t duration) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = duration;
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_INT_LOWHIGH_0, &dataToSend[0], 1);
    return opStatus;
}

uint8_t BMI160_SPI_GetShockDetectionThreshold(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_INT_LOWHIGH_4, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }

    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetShockDetectionThreshold(uint8_t threshold) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = threshold;
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_INT_LOWHIGH_4, &dataToSend[0], 1);
    return opStatus;
}

uint8_t BMI160_SPI_GetShockDetectionDuration(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_INT_LOWHIGH_3, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }

    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetShockDetectionDuration(uint8_t duration) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = duration;
	HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_INT_LOWHIGH_3, &dataToSend[0], 1);
    return opStatus;
}

uint8_t BMI160_SPI_GetStepDetectionMode(void) {
    uint8_t ret_step_conf0 = 0;
    uint8_t ret_min_step_buf = 0;

	BMI160_ReadSPI(BMI160_RA_STEP_CONF_0, &ret_step_conf0, 1);
	BMI160_ReadSPI(BMI160_RA_STEP_CONF_0, &ret_min_step_buf, 1);

    if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_NOR) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_NOR)) {
    	return BMI160_STEP_MODE_NORMAL;
    }
    else if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_SEN) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_SEN)) {
    	return BMI160_STEP_MODE_SENSITIVE;
	}
    else if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_ROB) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_ROB)) {
    	return BMI160_STEP_MODE_ROBUST;
    }
    else {
    	return BMI160_STEP_MODE_UNKNOWN;
    }
}

HAL_StatusTypeDef BMI160_SPI_SetStepDetectionMode(BMI160StepMode mode) {
    uint8_t step_conf0 = 0;
    uint8_t min_step_buf = 0;

    switch (mode)
    {
		case BMI160_STEP_MODE_NORMAL:
			step_conf0 = 0x15;
			min_step_buf = 0x3;
			break;
		case BMI160_STEP_MODE_SENSITIVE:
			step_conf0 = 0x2D;
			min_step_buf = 0x0;
			break;
		case BMI160_STEP_MODE_ROBUST:
			step_conf0 = 0x1D;
			min_step_buf = 0x7;
			break;
		default:
			return HAL_ERROR;
    };

    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = step_conf0;
	BMI160_WriteSPI(BMI160_RA_STEP_CONF_0, &dataToSend[0], 1);
    BMI160_SPI_WriteRegisterBits(BMI160_RA_STEP_CONF_1, min_step_buf, BMI160_STEP_BUF_MIN_BIT, BMI160_STEP_BUF_MIN_LEN);

    return HAL_OK;
}

uint8_t BMI160_SPI_GetStepCountEnabled(void) {
    return !!(BMI160_SPI_ReadRegBits(BMI160_RA_STEP_CONF_1, BMI160_STEP_CNT_EN_BIT, 1));
}

HAL_StatusTypeDef BMI160_SPI_SetStepCountEnabled(uint8_t enabled) {
    return BMI160_SPI_WriteRegisterBits(BMI160_RA_STEP_CONF_1, enabled ? 0x1 : 0, BMI160_STEP_CNT_EN_BIT, 1);
}

uint16_t BMI160_SPI_GetStepCount(void) {
    uint8_t buffer[2];
    buffer[0] = BMI160_RA_STEP_CNT_L;

    BMI160_ReadWriteSPI(BMI160_RA_STEP_CNT_L, &buffer[0], 2);

    return (((uint16_t)buffer[1]) << 8) | buffer[0];
}

HAL_StatusTypeDef BMI160_SPI_ResetStepCount(void) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = BMI160_CMD_STEP_CNT_CLR;
    HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_CMD, &dataToSend[0], 1);

    return opStatus;
}

HAL_StatusTypeDef BMI160_SPI_GetMotionDetectionThreshold(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_INT_MOTION_1, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }
    return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetMotionDetectionThreshold(uint8_t threshold) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = threshold;
    HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_INT_MOTION_1, &dataToSend[0], 1);
    return opStatus;
}

uint8_t BMI160_SPI_GetMotionDetectionDuration(void) {
    return (1 + BMI160_SPI_ReadRegBits(BMI160_RA_INT_MOTION_0, BMI160_ANYMOTION_DUR_BIT, BMI160_ANYMOTION_DUR_LEN));
}

HAL_StatusTypeDef BMI160_SPI_SetMotionDetectionDuration(uint8_t samples) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_INT_MOTION_0, samples - 1, BMI160_ANYMOTION_DUR_BIT, BMI160_ANYMOTION_DUR_LEN);
}

uint8_t BMI160_SPI_GetZeroMotionDetectionThreshold(void) {
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(BMI160_RA_INT_MOTION_2, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }

	return readData;
}

HAL_StatusTypeDef BMI160_SPI_SetZeroMotionDetectionThreshold(const uint8_t threshold) {
    uint8_t dataToSend[1] = {0x00};
    dataToSend[0] = threshold;

    HAL_StatusTypeDef opStatus = BMI160_WriteSPI(BMI160_RA_INT_MOTION_2, &dataToSend[0], 1);
    return opStatus;
}

uint8_t BMI160_SPI_GetZeroMotionDetectionDuration(void) {
    return BMI160_SPI_ReadRegBits(BMI160_RA_INT_MOTION_0, BMI160_NOMOTION_DUR_BIT, BMI160_NOMOTION_DUR_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetZeroMotionDetectionDuration(const uint8_t duration) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_INT_MOTION_0, duration, BMI160_NOMOTION_DUR_BIT, BMI160_NOMOTION_DUR_LEN);
}

uint8_t BMI160_SPI_GetTapDetectionThreshold(void) {
    return BMI160_SPI_ReadRegBits(BMI160_RA_INT_TAP_1, BMI160_TAP_THRESH_BIT, BMI160_TAP_THRESH_LEN);
}

HAL_StatusTypeDef BMI160_SPI_SetTapDetectionThreshold(const uint8_t threshold) {
	return BMI160_SPI_WriteRegisterBits(BMI160_RA_INT_TAP_1, threshold, BMI160_TAP_THRESH_BIT, BMI160_TAP_THRESH_LEN);
}

uint8_t BMI160_SPI_GetTapShockDuration(void) {
    return !!(BMI160_SPI_ReadRegBits(BMI160_RA_INT_TAP_0, BMI160_TAP_SHOCK_BIT, 1));
}

static HAL_StatusTypeDef BMI160_SPI_WriteRegister(uint8_t address, uint8_t cmd)
{
	uint8_t buff[1] ={cmd};
	return BMI160_WriteSPI(address, &buff[0], 1);
}

static HAL_StatusTypeDef BMI160_SPI_ReadRegister(uint8_t address, uint8_t dataSize, uint8_t *rec)
{
	uint8_t array[20] = {0xA5};

	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(address, &array[0], dataSize);

	if(opStatus != HAL_OK) {return HAL_ERROR;}

	for(uint8_t i=0; i<dataSize; i++)
	{
		*(rec + i) = array[i];
	}

	return opStatus;
}

static HAL_StatusTypeDef BMI160_WriteSPI(const uint8_t registerAddr, uint8_t *bufferPtr, const uint8_t bufferSize)
{
	uint8_t spiReg = registerAddr;
	_BMI160_CS_ENABLE();

	HAL_StatusTypeDef opStatus = HAL_SPI_Transmit(&hspi1, &spiReg, 1, 10);

	if(opStatus != HAL_OK) {
		_BMI160_CS_DISABLE();
		return opStatus;
	}
    opStatus = HAL_SPI_Transmit(&hspi1, bufferPtr, bufferSize, 10);

    _BMI160_CS_DISABLE();

	if(opStatus != HAL_OK) {
		return opStatus;
	}
	return 0;
}

static HAL_StatusTypeDef BMI160_ReadSPI(const uint8_t registerAddr, uint8_t *bufferPtr, const uint8_t bufferSize)
{
	uint8_t spiBuf[15] = {0x00};
	spiBuf[0] = registerAddr | 0x80;

	_BMI160_CS_ENABLE();

	HAL_StatusTypeDef opStatus = HAL_SPI_Transmit(&hspi1, spiBuf, 1, 20);

	if(opStatus != HAL_OK) {
		_BMI160_CS_DISABLE();
		return opStatus;
	}

	opStatus = HAL_SPI_Receive(&hspi1, spiBuf, bufferSize, 30);

	_BMI160_CS_DISABLE();

	if(opStatus != HAL_OK) {
		return opStatus;
	}

	for(uint8_t i=0; i<(bufferSize); i++){
		*(bufferPtr + i) = spiBuf[i];
	}

	return 0;
}

static uint8_t BMI160_SPI_ReadRegBits(uint8_t reg, unsigned pos, unsigned len)
{
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(reg, &readData, 1);

	if(opStatus != HAL_OK) { return 0x00; }

    uint8_t mask = (1 << len) - 1;
    readData >>= pos;
    readData &= mask;

    return readData;
}

static HAL_StatusTypeDef BMI160_SPI_WriteRegisterBits(uint8_t reg, uint8_t data, unsigned pos, unsigned len)
{
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = BMI160_ReadSPI(reg, &readData, 1);

	if(opStatus != HAL_OK) { return opStatus; }

    uint8_t mask = ((1 << len) - 1) << pos;

    data <<= pos;
    data &= mask;
    readData &= ~(mask);
    readData |= data;

    return BMI160_WriteSPI(reg, &readData, 1);
}

static HAL_StatusTypeDef BMI160_ReadWriteSPI(const uint8_t registerAddr, uint8_t *bufferPtr, const uint8_t bufferSize)
{
	uint8_t spiBuf[15] = {0x00};
	uint8_t recBuffer[15] = {0x00};
	spiBuf[0] = registerAddr | 0x80;

	_BMI160_CS_ENABLE();

	HAL_StatusTypeDef opstatus = HAL_SPI_TransmitReceive(&hspi1, spiBuf, recBuffer, bufferSize, 50);

	_BMI160_CS_DISABLE();

	for(uint8_t i=0; i<(bufferSize); i++){
		*(bufferPtr + i) = recBuffer[i+1];
	}

	return opstatus;
}

