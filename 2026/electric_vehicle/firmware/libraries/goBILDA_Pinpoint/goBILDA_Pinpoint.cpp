#include "CRC8.h"
#include <Arduino.h>
#include "goBILDA_Pinpoint.h"

void goBILDA::Pinpoint::begin(TwoWire &wire)
{
    i2c = wire;
    i2c.begin();

    if(getDeviceID() == PINPOINT_DEVICE_ID)
    {
        uint32_t version = getDeviceVersion();
        if(version >= 3)
            hasUpdatedFirmware = true;
    }
}

goBILDA::PinpointError goBILDA::Pinpoint::getLastError() const { return _lastError; }

uint32_t goBILDA::Pinpoint::getDeviceID(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::DEVICE_ID, 4, buf);
        _lastDeviceId = convertBufferToUint(buf);
    }
    return _lastDeviceId;
}

uint32_t goBILDA::Pinpoint::getDeviceVersion(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::DEVICE_VERSION, 4, buf);
        _lastDeviceVersion = convertBufferToUint(buf);
    }
    return _lastDeviceVersion;
}

float goBILDA::Pinpoint::getYawScalar(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::YAW_SCALAR, 4, buf);
        _lastYawScalar = convertBufferToFloat(buf);
    }
    return _lastYawScalar;
}

goBILDA::PinpointStatus goBILDA::Pinpoint::getDeviceStatus(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::DEVICE_STATUS, 4, buf);
        _lastDeviceStatus = convertBufferToUint(buf);
    }
    return PinpointStatus::GetStatus(_lastDeviceStatus);
}

uint32_t goBILDA::Pinpoint::getLoopTime(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::LOOP_TIME, 4, buf);
        _lastLoopTime = convertBufferToUint(buf);
    }
    return _lastLoopTime;
}

float goBILDA::Pinpoint::getFrequency(bool useBulkReadValue)
{
    uint32_t loopTime = getLoopTime(useBulkReadValue);
    if(loopTime == 0) return 0;
    return 1000000.0 / loopTime;
}

int32_t goBILDA::Pinpoint::getEncoderX(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::X_ENCODER_VALUE, 4, buf);
        _lastEncoderX = convertBufferToint(buf);
    }
    return _lastEncoderX;
}

int32_t goBILDA::Pinpoint::getEncoderY(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::Y_ENCODER_VALUE, 4, buf);
        _lastEncoderY = convertBufferToint(buf);
    }
    return _lastEncoderY;
}

float goBILDA::Pinpoint::getPositionXInMM(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::X_POSITION, 4, buf);
        _lastPositionX = convertBufferToFloat(buf);
    }
    return _lastPositionX;
}

float goBILDA::Pinpoint::getPositionYInMM(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::Y_POSITION, 4, buf);
        _lastPositionY = convertBufferToFloat(buf);
    }
    return _lastPositionY;
}

float goBILDA::Pinpoint::getNormalizedHeading(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::H_ORIENTATION, 4, buf);
        _lastHeading = convertBufferToFloat(buf);
    }
    return fmod(fabs(_lastHeading * RAD_TO_DEG), 360.0f) - 180.0;
}

float goBILDA::Pinpoint::getUnNormalizedHeading(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::H_ORIENTATION, 4, buf);
        _lastHeading = convertBufferToFloat(buf);
    }
    return _lastHeading * RAD_TO_DEG;
}

float goBILDA::Pinpoint::getVelocityX(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::X_VELOCITY, 4, buf);
        _lastVelocityX = convertBufferToFloat(buf);
    }
    return _lastVelocityX;
}

float goBILDA::Pinpoint::getVelocityY(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::Y_VELOCITY, 4, buf);
        _lastVelocityY = convertBufferToFloat(buf);
    }
    return _lastVelocityY;
}

float goBILDA::Pinpoint::getVelocityHeading(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::H_VELOCITY, 4, buf);
        _lastVelocityH = convertBufferToFloat(buf);
    }
    return _lastVelocityH;
}

float goBILDA::Pinpoint::getMmPerTick(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::MM_PER_TICK, 4, buf);
        _lastMmPerTick = convertBufferToFloat(buf);
    }
    return _lastMmPerTick;
}

float goBILDA::Pinpoint::getOffsetX(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::X_POD_OFFSET, 4, buf);
        _lastOffsetX = convertBufferToFloat(buf);
    }
    return _lastOffsetX;
}

float goBILDA::Pinpoint::getOffsetY(bool useBulkReadValue)
{
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::Y_POD_OFFSET, 4, buf);
        _lastOffsetY = convertBufferToFloat(buf);
    }
    return _lastOffsetY;
}

float goBILDA::Pinpoint::getPitch(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::PITCH)) return 0.0;
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::PITCH, 4, buf);
        _lastPitch = convertBufferToFloat(buf);
    }
    return _lastPitch;
}

float goBILDA::Pinpoint::getRoll(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::ROLL)) return 0.0;
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::ROLL, 4, buf);
        _lastRoll = convertBufferToFloat(buf);
    }
    return _lastRoll;
}

float goBILDA::Pinpoint::getQuaternionW(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_W)) return 0.0;
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::QUATERNION_W, 4, buf);
        _lastQuaternionW = convertBufferToFloat(buf);
    }
    return _lastQuaternionW;
}

float goBILDA::Pinpoint::getQuaternionX(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_X)) return 0.0;
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::QUATERNION_X, 4, buf);
        _lastQuaternionX = convertBufferToFloat(buf);
    }
    return _lastQuaternionX;
}

float goBILDA::Pinpoint::getQuaternionY(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_Y)) return 0.0;
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::QUATERNION_Y, 4, buf);
        _lastQuaternionY = convertBufferToFloat(buf);
    }
    return _lastQuaternionY;
}

float goBILDA::Pinpoint::getQuaternionZ(bool useBulkReadValue)
{
    if(!firmwareIsAbleToRead(Register::QUATERNION_Z)) return 0.0;
    if(!useBulkReadValue){
        uint8_t buf[4];
        getData(Register::QUATERNION_Z, 4, buf);
        _lastQuaternionZ = convertBufferToFloat(buf);
    }
    return _lastQuaternionZ;
}

goBILDA::Pose2D goBILDA::Pinpoint::getPosition(bool useBulkReadValue)
{
    float posX = getPositionXInMM(useBulkReadValue);
    float posY = getPositionYInMM(useBulkReadValue);
    float head = getUnNormalizedHeading(useBulkReadValue);
    return goBILDA::Pose2D{posX, posY, head};
}

goBILDA::Quaternion goBILDA::Pinpoint::getQuaternion(bool useBulkReadValue)
{
    float w = getQuaternionW(useBulkReadValue);
    float x = getQuaternionX(useBulkReadValue);
    float y = getQuaternionY(useBulkReadValue);
    float z = getQuaternionZ(useBulkReadValue);
    return goBILDA::Quaternion{w, x, y, z};
}

goBILDA::BulkReadData goBILDA::Pinpoint::bulkRead(void)
{
    uint8_t data[96]; // Max 24 registers * 4 bytes = 96
    getData(Register::BULK_READ, sizeof(uint32_t) * bulkReadScopeCount, data);
    
    BulkReadData brData;
    brData.Error = getLastError();

    if(brData.Error != PinpointError::None)
        return brData;

    for(int i = 0; i < bulkReadScopeCount; i++){
        saveData(bulkReadScope[i], &data[i * 4], brData);
    }

    return brData;
}

void goBILDA::Pinpoint::resetBulkRead(void)
{
    uint8_t newBulkReadScope[1] = {static_cast<uint8_t>(Register::BULK_READ)};
    writeData(Register::SET_BULK_READ, newBulkReadScope, 1);
    
    bulkReadScope[0] = PinpointRegisters::DeviceStatus;
    bulkReadScope[1] = PinpointRegisters::LoopTime;
    bulkReadScope[2] = PinpointRegisters::EncoderValueX;
    bulkReadScope[3] = PinpointRegisters::EncoderValueY;
    bulkReadScope[4] = PinpointRegisters::PositionX;
    bulkReadScope[5] = PinpointRegisters::PositionY;
    bulkReadScope[6] = PinpointRegisters::Heading;
    bulkReadScope[7] = PinpointRegisters::VelocityX;
    bulkReadScope[8] = PinpointRegisters::VelocityY;
    bulkReadScope[9] = PinpointRegisters::VelocityH;
    bulkReadScopeCount = 10;
}

void goBILDA::Pinpoint::setBulkReadScope(const PinpointRegisters* registers, uint8_t count)
{
    if(!hasUpdatedFirmware) return;
    
    uint32_t reg_bm = 0;
    bulkReadScopeCount = 0;
    for(int i = 0; i < count; i++){
        uint32_t curr_reg = 1 << static_cast<uint32_t>(registers[i]);
        if(!(reg_bm & curr_reg)){
            reg_bm |= curr_reg;
            bulkReadScope[bulkReadScopeCount++] = registers[i];
        }
    }

    uint8_t buf[24];
    for(uint8_t i = 0; i < bulkReadScopeCount; i++)
        buf[i] = static_cast<uint8_t>(bulkReadScope[i]);
        
    writeData(Register::SET_BULK_READ, buf, bulkReadScopeCount);
}

void goBILDA::Pinpoint::recalibrateIMU(void) const
{
    uint32_t bit_mask = 1 << 0;
    uint8_t buf[4];
    loadBufferWithUint(buf, bit_mask);
    writeData(Register::DEVICE_CONTROL, buf, 4);
}

void goBILDA::Pinpoint::resetPositionAndIMU(void) const
{
    uint32_t bit_mask = 1 << 1;
    uint8_t buf[4];
    loadBufferWithUint(buf, bit_mask);
    writeData(Register::DEVICE_CONTROL, buf, 4);
}

void goBILDA::Pinpoint::setOffsets(float x, float y)
{
    uint8_t buf[4];
    loadBufferWithFloat(buf, x);
    writeData(Register::X_POD_OFFSET, buf, 4);
    loadBufferWithFloat(buf, y);
    writeData(Register::Y_POD_OFFSET, buf, 4);
}

void goBILDA::Pinpoint::setEncoderDirections(EncoderDirection x, EncoderDirection y)
{
    constexpr uint32_t X_FORWARD_BM  = 1 << 5;
    constexpr uint32_t X_BACKWARD_BM = 1 << 4;
    constexpr uint32_t Y_FORWARD_BM  = 1 << 3;
    constexpr uint32_t Y_BACKWARD_BM = 1 << 2;

    uint32_t control_reg_val = 0;
    control_reg_val |= (x == EncoderDirection::Forward ? X_FORWARD_BM : X_BACKWARD_BM);
    control_reg_val |= (y == EncoderDirection::Forward ? Y_FORWARD_BM : Y_BACKWARD_BM);

    uint8_t buf[4];
    loadBufferWithUint(buf, control_reg_val);
    writeData(Register::DEVICE_CONTROL, buf, 4);
}

void goBILDA::Pinpoint::setEncoderResolution(EncoderResolution resolution)
{
    uint8_t buf[4];
    if(resolution == EncoderResolution::goBILDA_4_BAR_POD)
        loadBufferWithFloat(buf, 19.89436789f);
    else if(resolution == EncoderResolution::goBILDA_SWINGARM_POD)
        loadBufferWithFloat(buf, 13.26291192f);
    
    writeData(Register::MM_PER_TICK, buf, 4);
}

void goBILDA::Pinpoint::setEncoderResolution(float ticks_per_mm)
{
    uint8_t buf[4];
    loadBufferWithFloat(buf, ticks_per_mm);
    writeData(Register::MM_PER_TICK, buf, 4);
}

void goBILDA::Pinpoint::setYawScalar(float yawScalar)
{
    uint8_t buf[4];
    loadBufferWithFloat(buf, yawScalar);
    writeData(Register::YAW_SCALAR, buf, 4);
}

void goBILDA::Pinpoint::setPosition(Pose2D pos)
{
    setPosX(pos.x);
    setPosY(pos.y);
    setHeading(pos.heading);
}

void goBILDA::Pinpoint::setPosX(float positionInMM)
{
    uint8_t buf[4];
    loadBufferWithFloat(buf, positionInMM);
    writeData(Register::X_POSITION, buf, 4);
}

void goBILDA::Pinpoint::setPosY(float positionInMM)
{
    uint8_t buf[4];
    loadBufferWithFloat(buf, positionInMM);
    writeData(Register::Y_POSITION, buf, 4);
}

void goBILDA::Pinpoint::setHeading(float angleInRadians)
{
    uint8_t buf[4];
    loadBufferWithFloat(buf, angleInRadians);
    writeData(Register::H_ORIENTATION, buf, 4);
}

// #region Private Member Functions

void goBILDA::Pinpoint::getData(Register reg, uint8_t count, uint8_t* outBuffer)
{
    _lastError = PinpointError::None;

    i2c.beginTransmission(i2c_address);
    i2c.write(static_cast<uint8_t>(reg));
    i2c.endTransmission(true);

    uint8_t bytes_to_request = count + (hasUpdatedFirmware ? 1 : 0);
    uint8_t bytes_to_read = i2c.requestFrom(i2c_address, bytes_to_request, static_cast<uint8_t>(true));
    
    if(bytes_to_read == 0){
        _lastError = PinpointError::I2C_Error;
        for(int i = 0; i < count; i++) outBuffer[i] = 0;
        return;
    }

    uint8_t actual_read = (count < bytes_to_read) ? count : bytes_to_read;
    for(uint8_t i = 0; i < actual_read; i++) outBuffer[i] = i2c.read();
    
    if(hasUpdatedFirmware && i2c.available()){
        uint8_t crc = i2c.read();
        if(CRC8_ComputeFast(outBuffer, count) != crc){
            _lastError = PinpointError::CRC_Failed;
            for(uint8_t i = 0; i < count; i++) outBuffer[i] = 0;
        }
    }
}

void goBILDA::Pinpoint::writeData(Register reg, const uint8_t* data, uint8_t length) const
{
    i2c.beginTransmission(i2c_address);
    i2c.write(static_cast<uint8_t>(reg));
    i2c.write(data, length);
    i2c.endTransmission(true);
}

void goBILDA::Pinpoint::saveData(PinpointRegisters reg, const uint8_t* data, BulkReadData &bulk_data)
{
    switch(reg){
    case PinpointRegisters::EncoderValueX:
        _lastEncoderX = bulk_data.EncoderX = convertBufferToint(data);
    break;
    case PinpointRegisters::EncoderValueY:
        _lastEncoderY = bulk_data.EncoderY = convertBufferToint(data);
    break;
    case PinpointRegisters::DeviceVersion:
        _lastDeviceVersion = bulk_data.DeviceVersion = convertBufferToUint(data);
    break;
    case PinpointRegisters::DeviceStatus:
        _lastDeviceStatus = convertBufferToUint(data);
        bulk_data.DeviceStatus = PinpointStatus::GetStatus(_lastDeviceStatus);
    break;
    case PinpointRegisters::Heading:
        _lastHeading = bulk_data.Position.heading = convertBufferToFloat(data);
    break;
    case PinpointRegisters::QuaternionW:
        _lastQuaternionW = bulk_data.quaternion.w = convertBufferToFloat(data);
    break;
    case PinpointRegisters::QuaternionX:
        _lastQuaternionX = bulk_data.quaternion.x = convertBufferToFloat(data);
    break;
    case PinpointRegisters::QuaternionY:
        _lastQuaternionY = bulk_data.quaternion.y = convertBufferToFloat(data);
    break;
    case PinpointRegisters::QuaternionZ:
        _lastQuaternionZ = bulk_data.quaternion.z = convertBufferToFloat(data);
    break;
    case PinpointRegisters::PodOffsetX:
        _lastOffsetX = bulk_data.OffsetX = convertBufferToFloat(data);
    break;
    case PinpointRegisters::PodOffsetY:
        _lastOffsetY = bulk_data.OffsetY = convertBufferToFloat(data);
    break;
    case PinpointRegisters::MmPerTick:
        _lastMmPerTick = bulk_data.MmPerTick = convertBufferToFloat(data);
    break;
    case PinpointRegisters::YawScalar:
        _lastYawScalar = bulk_data.YawScalar = convertBufferToFloat(data);
    break;
    case PinpointRegisters::PositionX:
        _lastPositionX = bulk_data.Position.x = convertBufferToFloat(data);
    break;
    case PinpointRegisters::PositionY:
        _lastPositionY = bulk_data.Position.y = convertBufferToFloat(data);
    break;
    case PinpointRegisters::VelocityX:
        _lastVelocityX = bulk_data.VelocityX = convertBufferToFloat(data);
    break;
    case PinpointRegisters::VelocityY:
        _lastVelocityY = bulk_data.VelocityY = convertBufferToFloat(data);
    break;
    case PinpointRegisters::VelocityH:
        _lastVelocityH = bulk_data.VelocityH = convertBufferToFloat(data);
    break;
    case PinpointRegisters::DeviceID:
        _lastDeviceId = bulk_data.DeviceId = convertBufferToUint(data);
    break;
    case PinpointRegisters::LoopTime:
        _lastLoopTime = bulk_data.LoopTime = convertBufferToUint(data);
    break;
    case PinpointRegisters::Pitch:
        _lastPitch = bulk_data.Pitch = convertBufferToFloat(data);
    break;
    case PinpointRegisters::Roll:
        _lastRoll = bulk_data.Roll = convertBufferToFloat(data);
    break;
    }
}

void goBILDA::Pinpoint::loadBufferWithFloat(uint8_t* buffer, float value) const
{
    union { float f; uint8_t b[4]; } u;
    u.f = value;
    buffer[0] = u.b[0];
    buffer[1] = u.b[1];
    buffer[2] = u.b[2];
    buffer[3] = u.b[3];
}

void goBILDA::Pinpoint::loadBufferWithUint(uint8_t* buffer, uint32_t value) const
{
    buffer[0] = (value >>  0) & 0xFF;
    buffer[1] = (value >>  8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
}

int32_t goBILDA::Pinpoint::convertBufferToint(const uint8_t* buffer) const
{
    int32_t returnVal = 0;
    returnVal |= (uint32_t)buffer[3] << 24;
    returnVal |= (uint32_t)buffer[2] << 16;
    returnVal |= (uint32_t)buffer[1] <<  8;
    returnVal |= (uint32_t)buffer[0] <<  0;
    return returnVal;
}

uint32_t goBILDA::Pinpoint::convertBufferToUint(const uint8_t* buffer) const
{
    uint32_t returnVal = 0;
    returnVal |= (uint32_t)buffer[3] << 24;
    returnVal |= (uint32_t)buffer[2] << 16;
    returnVal |= (uint32_t)buffer[1] <<  8;
    returnVal |= (uint32_t)buffer[0] <<  0;
    return returnVal;
}

float goBILDA::Pinpoint::convertBufferToFloat(const uint8_t* buffer) const
{
    union { float f; uint8_t b[4]; } u;
    u.b[3] = buffer[3];
    u.b[2] = buffer[2];
    u.b[1] = buffer[1];
    u.b[0] = buffer[0];
    return u.f;
}

bool goBILDA::Pinpoint::firmwareIsAbleToRead(Register reg) const
{
    switch(reg){
    case Register::DEVICE_ID:
    case Register::DEVICE_VERSION:
    case Register::DEVICE_STATUS:
    case Register::LOOP_TIME:
    case Register::X_ENCODER_VALUE:
    case Register::Y_ENCODER_VALUE:
    case Register::X_POSITION:
    case Register::Y_POSITION:
    case Register::H_ORIENTATION:
    case Register::X_VELOCITY:
    case Register::Y_VELOCITY:
    case Register::H_VELOCITY:
    case Register::MM_PER_TICK:
    case Register::X_POD_OFFSET:
    case Register::Y_POD_OFFSET:
    case Register::YAW_SCALAR:
        return true;

    default:
    case Register::QUATERNION_W:
    case Register::QUATERNION_X:
    case Register::QUATERNION_Y:
    case Register::QUATERNION_Z:
    case Register::PITCH:
    case Register::ROLL:
        return hasUpdatedFirmware;
    }
}
// #endregion Private Member Functions