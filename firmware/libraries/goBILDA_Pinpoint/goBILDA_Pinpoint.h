#include <Wire.h>
#include <stdint.h>

#ifndef GOBILDA_PINPOINT_H
#define GOBILDA_PINPOINT_H

namespace goBILDA{
    enum class PinpointError {
        None,
        CRC_Failed,
        I2C_Error,
    };

    enum class EncoderResolution{
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD
    };

    enum class EncoderDirection{
        Forward,
        Backward
    };

    enum class PinpointRegisters : uint8_t{
        DeviceID      = 1,
        DeviceVersion = 2,
        DeviceStatus  = 3,
        LoopTime      = 5,
        EncoderValueX = 6,
        EncoderValueY = 7,
        PositionX     = 8,
        PositionY     = 9,
        Heading       = 10,
        VelocityX     = 11,
        VelocityY     = 12,
        VelocityH     = 13,
        MmPerTick     = 14,
        PodOffsetX    = 15,
        PodOffsetY    = 16,
        YawScalar     = 17,
        QuaternionW   = 19,
        QuaternionX   = 20,
        QuaternionY   = 21,
        QuaternionZ   = 22,
        Pitch         = 23,
        Roll          = 24
    };

    struct PinpointStatus{
        bool ready;
        bool calibrating;
        bool xPodDetected;
        bool yPodDetected;
        bool FAULT_ImuRunaway;
        bool FAULT_badWriteCRC;
        bool FAULT_badRead;

        static PinpointStatus GetStatus(uint32_t device_status)
        {
            PinpointStatus status;
            if(device_status & 0b000001) status.ready = false;
            if(device_status & 0b000010) status.ready = false;
            if(device_status & 0b000100) status.calibrating = true;
            if(device_status & 0b001000) status.xPodDetected = true;
            if(device_status & 0b010000) status.yPodDetected = true;

            return status;
        }
    };

    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    };

    struct Pose2D{
        float x;
        float y;
        float heading;
    };

    struct BulkReadData{
        PinpointError Error = PinpointError::None;
        uint32_t DeviceId = 0;
        uint32_t DeviceVersion = 0;
        uint32_t LoopTime = 0;
        int32_t EncoderX  = 0;
        int32_t EncoderY  = 0;
        float YawScalar   = 0.0;
        float VelocityX   = 0.0;
        float VelocityY   = 0.0;
        float VelocityH   = 0.0;
        float OffsetX     = 0.0;
        float OffsetY     = 0.0;
        float Pitch       = 0.0;
        float Roll        = 0.0;
        float MmPerTick   = 0.0;
        Pose2D Position   = {0};
        Quaternion quaternion = {0};
        PinpointStatus DeviceStatus = {0};
    };

    class Pinpoint{
        public:
        void begin(TwoWire &wire = Wire);
        PinpointError getLastError() const;

        PinpointStatus getDeviceStatus(bool useBulkReadValue = false);
        uint32_t getDeviceID(bool useBulkReadValue = false);
        uint32_t getDeviceVersion(bool useBulkReadValue = false);
        uint32_t getLoopTime(bool useBulkReadValue = false);

        int32_t getEncoderX(bool useBulkReadValue = false);
        int32_t getEncoderY(bool useBulkReadValue = false);
        
        float getYawScalar(bool useBulkReadValue = false);
        float getFrequency(bool useBulkReadValue = false);
        float getPositionXInMM(bool useBulkReadValue = false);
        float getPositionYInMM(bool useBulkReadValue = false);
        float getNormalizedHeading(bool useBulkReadValue = false);
        float getUnNormalizedHeading(bool useBulkReadValue = false);
        float getVelocityX(bool useBulkReadValue = false);
        float getVelocityY(bool useBulkReadValue = false);
        float getVelocityHeading(bool useBulkReadValue = false);
        float getMmPerTick(bool useBulkReadValue = false);
        float getOffsetX(bool useBulkReadValue = false);
        float getOffsetY(bool useBulkReadValue = false);
        float getPitch(bool useBulkReadValue = false);
        float getRoll(bool useBulkReadValue = false);

        float getQuaternionW(bool useBulkReadValue = false);
        float getQuaternionX(bool useBulkReadValue = false);
        float getQuaternionY(bool useBulkReadValue = false);
        float getQuaternionZ(bool useBulkReadValue = false);

        goBILDA::Pose2D getPosition(bool useBulkReadValue = false);
        goBILDA::Quaternion getQuaternion(bool useBulkReadValue = false);

        BulkReadData bulkRead(void);
        void resetBulkRead(void);
        void setBulkReadScope(const PinpointRegisters* registers, uint8_t count);

        void recalibrateIMU(void) const;
        void resetPositionAndIMU(void) const;
        void setOffsets(float x, float y);
        void setEncoderDirections(EncoderDirection xForward, EncoderDirection yForward);
        void setEncoderResolution(EncoderResolution resolution);
        void setEncoderResolution(float ticks_per_mm);
        void setYawScalar(float yawScalar);
        void setPosition(Pose2D pos);
        void setPosX(float positionInMM);
        void setPosY(float positionInMM);
        void setHeading(float angleInRadians);

    private:
        static constexpr uint8_t PINPOINT_DEVICE_ID = 2;
        static constexpr uint8_t i2c_address = 0x31;

        bool hasUpdatedFirmware = false;
        TwoWire &i2c = Wire;

        enum class Register : uint8_t {
            DEVICE_ID       = 1,
            DEVICE_VERSION  = 2,
            DEVICE_STATUS   = 3,
            DEVICE_CONTROL  = 4,
            LOOP_TIME       = 5,
            X_ENCODER_VALUE = 6,
            Y_ENCODER_VALUE = 7,
            X_POSITION      = 8,
            Y_POSITION      = 9,
            H_ORIENTATION   = 10,
            X_VELOCITY      = 11,
            Y_VELOCITY      = 12,
            H_VELOCITY      = 13,
            MM_PER_TICK     = 14,
            X_POD_OFFSET    = 15,
            Y_POD_OFFSET    = 16,
            YAW_SCALAR      = 17,
            BULK_READ       = 18,
            QUATERNION_W    = 19,
            QUATERNION_X    = 20,
            QUATERNION_Y    = 21,
            QUATERNION_Z    = 22,
            PITCH           = 23,
            ROLL            = 24,
            SET_BULK_READ   = 25
        };

        PinpointRegisters bulkReadScope[24] = {
            PinpointRegisters::DeviceStatus ,
            PinpointRegisters::LoopTime     ,
            PinpointRegisters::EncoderValueX,
            PinpointRegisters::EncoderValueY,
            PinpointRegisters::PositionX    ,
            PinpointRegisters::PositionY    ,
            PinpointRegisters::Heading      ,
            PinpointRegisters::VelocityX    ,
            PinpointRegisters::VelocityY    ,
            PinpointRegisters::VelocityH
        };
        uint8_t bulkReadScopeCount = 10;

        PinpointError _lastError = PinpointError::None;
        uint32_t _lastDeviceId   = 0;
        uint32_t _lastDeviceStatus = 0;
        uint32_t _lastDeviceVersion = 0;
        uint32_t _lastLoopTime = 0;
        int32_t _lastEncoderX  = 0;
        int32_t _lastEncoderY  = 0;
        float _lastYawScalar   = 0.0;
        float _lastPositionX   = 0.0;
        float _lastPositionY   = 0.0;
        float _lastHeading     = 0.0;
        float _lastVelocityX   = 0.0;
        float _lastVelocityY   = 0.0;
        float _lastVelocityH   = 0.0;
        float _lastOffsetX     = 0.0;
        float _lastOffsetY     = 0.0;
        float _lastPitch       = 0.0;
        float _lastRoll        = 0.0;
        float _lastMmPerTick   = 0.0;
        float _lastQuaternionW = 0.0;
        float _lastQuaternionX = 0.0;
        float _lastQuaternionY = 0.0;
        float _lastQuaternionZ = 0.0;

        void getData(Register reg, uint8_t count, uint8_t* outBuffer);
        void writeData(Register reg, const uint8_t* data, uint8_t length) const;
        void saveData(PinpointRegisters reg, const uint8_t* data, BulkReadData &bulk_data);

        void loadBufferWithFloat(uint8_t* buffer, float value) const;
        void loadBufferWithUint(uint8_t* buffer, uint32_t value) const;

        int32_t convertBufferToint(const uint8_t* buffer) const;
        uint32_t convertBufferToUint(const uint8_t* buffer) const;
        float convertBufferToFloat(const uint8_t* buffer) const;

        bool firmwareIsAbleToRead(Register reg) const;
  };
}
#endif