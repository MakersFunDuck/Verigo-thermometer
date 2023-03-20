/**************************************************************************************************'
  Filename:       hSi705.c
  Revised:        $Date: 2014-09-19 07:58:08 -0700 (Mon, 19 Mar 2023) $
  Revision:       $Revision: 000 $

  Description:    Driver for the Si705 sensor

**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/
#include "Si705.h"
//#include "hal_sensor.h"
#include "hal_i2c.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

// Sensor I2C address
#define HAL_TEMP112_I2C_ADDRESS     0x40       //rd/wr not included



/* ------------------------------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
bool HalTemp112ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
bool HalTemp112WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 buf[6];                      // Data buffer
static bool  success;
static uint8 buffer[24];

/**************************************************************************************************
* @fn          HalHumiInit
*
* @brief       Initialise the humidity sensor driver
*
* @return      none
**************************************************************************************************/
void Si705Init(void)
{

  
  HalI2CInit(HAL_TEMP112_I2C_ADDRESS,i2cClock_267KHZ);
 
}


uint16 read_si705_temperature(void){  
  
  
    uint8 buf[2];
    uint16 temp;

    // Initialize I2C interface
    HalI2CInit(HAL_TEMP112_I2C_ADDRESS, i2cClock_267KHZ);

    // Send temperature measurement command
    buf[0] = 0xE3;
    buf[1] = 0xFF;
    HalI2CWrite(1, buf);

    
   //halSleep(10 ); // 10 ms delay;
    HalI2CRead(2, buf);
 
     while (buf[0] == 0xE3 & buf[1] == 0xFF ) {
 // Do nothing
    }
    

    // Convert temperature to degrees Celsius, multiplied by 100 to get a lazy conversation
    temp = (uint16)(100*(((buf[0] << 8) | buf[1]) * 175.72 / 65536 - 46.85));
    
  

    // Disable I2C interface
    HalI2CDisable();

    
    
    
    return temp;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
}  
/**************************************************************************************************
 * @fn          HalSensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor. The sensor must
 *              be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - numbver of bytes to read
 *
 * @return      TRUE if the required number of bytes are reveived
 **************************************************************************************************/
bool HalTemp112ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes){
  uint8 i = 0;

  /* Send address we're reading from */
  if (HalI2CWrite(1,&addr) == 1)
  {
    /* Now read data */
    i = HalI2CRead(nBytes,pBuf);
  }

  return i == nBytes;
}
/**************************************************************************************************
* @fn          HalSensorWriteReg
* @brief       This function implements the I2C protocol to write to a sensor. he sensor must
*              be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool HalTemp112WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes){
  uint8 i;
  uint8 *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send address and data */
  i = HalI2CWrite(nBytes, buffer);
  if ( i!= nBytes){
    i = i;
  }
    //write error indicate

  return (i == nBytes);
}  
/**************************************************************************************************
* @fn          HalHumiExecMeasurementStep
*
* @brief       Execute measurement step
*
* @return      none
*/
bool HalHumiExecMeasurementStep(uint8 state)
{
 /* HalHumiSelect();

  switch (state)
  {
    case 0:
      // Turn on DC-DC control
      HalDcDcControl(ST_HUMID,true);

      // Start temperature read
      success = HalHumiWriteCmd(SHT21_CMD_TEMP_T_NH);
      break;

    case 1:
      // Read and store temperature value
      if (success)
      {
        success = HalHumiReadData(buf, DATA_LEN);

        // Start for humidity read
        if (success)
        {
          success = HalHumiWriteCmd(SHT21_CMD_HUMI_T_NH);
        }
      }
      break;

    case 2:
      // Read and store humidity value
      if (success)
      {
        success = HalHumiReadData(buf+DATA_LEN, DATA_LEN);
      }

      // Turn of DC-DC control
      HalDcDcControl(ST_HUMID,false);
      break;
  }
*/
  return success;
}


/**************************************************************************************************
* @fn          HalHumiReadMeasurement
*
* @brief       Get humidity sensor data
*
* @return      none
*/
bool HalHumiReadMeasurement(uint8 *pBuf)
{
  // Store temperature
  pBuf[0] = buf[1];
  pBuf[1] = buf[0];

  // Store humidity
  pBuf[2] = buf[4];
  pBuf[3] = buf[3];

  return success;
}



/**************************************************************************************************
* @fn          HalHumiTest
*
* @brief       Humidity sensor self test
*
* @return      none
**************************************************************************************************/
bool HalHumiTest(void)
{
/*  uint8 val;

  HalHumiSelect();

  // Verify default value
  ST_ASSERT(HalSensorReadReg(SHT21_CMD_READ_U_R,&val,1));
  ST_ASSERT(val==usr);
*/
  return TRUE;
}


/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/


/*********************************************************************
*********************************************************************/

