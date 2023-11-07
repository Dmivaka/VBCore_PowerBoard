#include "my_helpers.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

/*
  * @brief Decodes FDCAN_data_length_code into the decimal length of FDCAN message
  * @param[in]          length           FDCAN_data_length_code
  * @retval             uint8_t         Decimal message length (bytes)
*/
uint8_t LengthDecoder( uint32_t length )
{
  if( length < FDCAN_DLC_BYTES_12 )
  {
    return (uint8_t)(length >> 16);
  }
  else if( length < FDCAN_DLC_BYTES_32 )
  {
    return (uint8_t)((length >> 14) - 24);
  }

  switch( length )
  {
    case FDCAN_DLC_BYTES_32:    return 32;
    case FDCAN_DLC_BYTES_48:    return 48; 
    case FDCAN_DLC_BYTES_64:    return 64;
      
    default:
      while(1); //error
  }
}

/*
  * @brief Codes decimal length of FDCAN message into the FDCAN_data_length_code
  * @param[in]          length              Decimal message length (bytes)
  * @retval             FDCAN_data_length_code        Code of required message length
*/
uint32_t LengthCoder( uint8_t length )
{
  if( length < 12 )
  {
    return (uint32_t)length << 16;
  }
  if( length < 32 )
  {
    return (uint32_t)(length + 24 ) << 14;
  }

  switch( length )
  {
    case 32:    return FDCAN_DLC_BYTES_32;
    case 48:    return FDCAN_DLC_BYTES_48;
    case 64:    return FDCAN_DLC_BYTES_64;
      
    default:
      while(1); //error
  }
}