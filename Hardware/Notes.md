#### SDK
Examples seem to be written for higher level applications. For example the "running_sensor" example will detect if a user is still, running or walking.

So far I think we should approach this using a more stripped back example such as *simple_peripheral* and add sections as we need them.

### src/profiles/accelerometer/accelerometer.c - fuction to access accelerometer data

``` C
/*********************************************************************
 * @fn      Accel_GetParameter
 *
 * @brief   Get an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Accel_GetParameter( uint8 param, void *value )

/*********************************************************************
 * @fn          accel_ReadAttr
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t accel_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                   uint8 *pValue, uint16 *pLen, uint16 offset,
                                   uint16 maxLen, uint8 method )
```
