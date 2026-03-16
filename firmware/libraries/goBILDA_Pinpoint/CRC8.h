// cppcheck-suppress-file unusedFunction
/*
 * CRC8.h
 *
 * Created: 5/2/2025
 * Author: Devin Marx
 * Version: 1.0.0
 * Version History: 
 *      1.0.0 - Created the CRC8 Library 
 */ 


#include <stdint.h>

#ifndef CRC8_H_
#define CRC8_H_
#ifdef __cplusplus
extern "C" {
#endif

const uint8_t CRC_INITIAL_VALUE    = 0x90;
const uint8_t CRC_POLYNOMIAL_VALUE = 0x31;
const uint8_t CRC_FINAL_XOR_VALUE  = 0x00;

typedef struct{
    uint8_t CRC;
    uint8_t numberOfBytes;
}CRC8_t;

/// @brief Initializes a CRC8_t struct with default values.
/// @return A CRC8_t struct with the CRC set to CRC_INITIAL_VALUE and numberOfBytes set to 0.
CRC8_t  CRC8_CreateCRC8(void);

/// @brief Retrieves the final CRC8 value from a CRC8_t struct, applying the final XOR value.
/// @param crc A CRC8_t struct containing the intermediate CRC value.
/// @return The finalized CRC8 value after applying the final XOR.
/// @note If CRC_FINAL_XOR_VALUE is 0x00, this function returns the raw CRC value.
inline uint8_t CRC8_GetFinalCRC(CRC8_t crc)
{
    return crc.CRC ^ CRC_FINAL_XOR_VALUE;
}

/// @brief Updates the CRC8 value using a precomputed lookup table for fast computation.
/// @param crc Pointer to an existing CRC8_t struct.
/// @param nextByte The next byte of data to include in the CRC computation.
/// @return The updated CRC value after including the next byte.
/// @note This method uses a 256-byte lookup table, which increases RAM usage.
uint8_t CRC8_ComputeNextByteFast(CRC8_t *crc, uint8_t nextByte);

/// @brief Updates the CRC8 value using bitwise operations for memory-constrained environments.
/// @param crc Pointer to an existing CRC8_t struct.
/// @param nextByte The next byte of data to include in the CRC computation.
/// @return The updated CRC value after including the next byte.
uint8_t CRC8_ComputeNextByteSlow(CRC8_t *crc, uint8_t nextByte);

/// @brief Computes the CRC8 of a data buffer using a fast lookup table method.
/// @param data Pointer to the data buffer.
/// @param len Length of the data buffer in bytes.
/// @return The computed CRC8 value.
/// @note This method uses a 256-byte lookup table, which increases RAM usage.
uint8_t CRC8_ComputeFast(const uint8_t *data, uint8_t len);

/// @brief Computes the CRC8 of a data buffer using a slow bitwise method for reduced memory usage.
/// @param data Pointer to the data buffer.
/// @param len Length of the data buffer in bytes.
/// @return The computed CRC8 value.
uint8_t CRC8_ComputeSlow(const uint8_t *data, uint8_t len);
#ifdef __cplusplus
}
#endif
#endif /* CRC8_H_ */