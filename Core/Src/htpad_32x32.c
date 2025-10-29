/*
 * htpad_32x32.c
 *
 *  Created on: Oct 12, 2025
 *      Author: Loc
 */
/*** PROGRAMM INFO***************************************************************************************
  source code for ESP32 and HTPAd Application Shield
  name:           ESP32_HTPAd_32x32.ino
  version/date:   2.2 / 20 Dec 2022
  programmer:     Heimann Sensor GmbH / written by Dennis Pauer (pauer@heimannsensor.com)
*********************************************************************************************************/



/*** MODES **********************************************************************************************
  The source code includes three ways to interact with the sensor:
  - via WIFI you can stream thermal images in our GUI (#define WIFIMODE)
  - via the SERIAL monitor you can observe the sensor data as text output (#define SERIALMODE)
  - via ACCESSPOINT the ESP32 creates the wifi network. You have to connect your computer to this network
    to stream thermal images in the GUI (#define ACCESSPOINT)
  Both modes are contain in the same code and you can activate one or both by activate the matching define.
*********************************************************************************************************/
// #define WIFIMODE
#define SERIALMODE
//#define ACCESSPOINT // automatically diablead if WIFIMODE is active

/*** NETWORK INFORMATION ********************************************************************************
  If you want to use the WIFI function, you have to change ssid and pass.
*********************************************************************************************************/
#include "main.h"
#include <stdlib.h>
#include <math.h>
#include "common.h"
#include "htpad_32x32.h"
#include <stdio.h>
#include <stdbool.h>
#include "uart_printf.h"


characteristics DevConst = DevConstDEF;


/********************************************************************
 ********************************************************************
    - - - PART 1: ARDUINO FUNCTIONS - - -
    TimerISR()...    interrupt service routine, called by timer
    setup()...  this function is only called once before jumping into
                the main function loop()
    loop()...   main function of an arduino code
    set_LED(uint8_t red, uint8_t green, uint8_t blue)
 ********************************************************************
 *********************************************************************/


/********************************************************************
   Function:        setup()
   Description:
 *******************************************************************/
void setup(htpad_t* hd)
{

  //*******************************************************************
  // use a heap allocated memory to store the pixc instead of a global nxm array
  //*******************************************************************
  //this is special for the ESP32, because it isn't enough space in the stack
  // you don't have to do that in this way if you have enough space to store pixc as global variable
  hd->eep.pixc2_0 = (uint32_t*)calloc(NUMBER_OF_PIXEL * 4, 1);
  if (hd->eep.pixc2_0 == NULL)
  {
	printf("Allocate memory failed\n");
  }else
  {
    printf("Allocate memory succeeded\n");
    hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap
  }


  //*******************************************************************
  // searching for sensor; if connected: read the whole EEPROM
  //*******************************************************************
  uint8_t error = HAL_ERROR;
  while (error != HAL_OK)
  {
    HAL_Delay(200);
    // Reinitialize the I2C peripheral
    HAL_I2C_DeInit(hd->hi2c);
    HAL_I2C_Init(hd->hi2c);

    // Check device readiness
     error = HAL_I2C_IsDeviceReady(hd->hi2c, SENSOR_ADDRESS << 1, 3, 100);
  }
  read_eeprom(hd);



  //*******************************************************************
  // wake up and start the sensor
  //*******************************************************************
  // to wake up sensor set configuration register to 0x01
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(hd, SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);

  // write the calibration settings into the trim registers
  write_calibration_settings_to_sensor(hd);

  // to start sensor set configuration register to 0x09
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(hd, SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);
  //printf("HTPAd is ready\n");

  //*******************************************************************
  // do bigger calculation here before you jump into the loop() function
  //*******************************************************************
  hd->var_ctrl.gradscale_div = pow(2, hd->eep.gradscale);
  hd->var_ctrl.vddscgrad_div = pow(2, hd->eep.vddscgrad);
  hd->var_ctrl.vddscoff_div = pow(2, hd->eep.vddscoff);
  calcPixC(hd); // calculate the pixel constants

  //*******************************************************************
  // timer initialization
  //*******************************************************************
  hd->var_ctrl.timert = calc_timert(hd->eep.clk_calib, hd->eep.mbit_calib);
  __HAL_TIM_SET_AUTORELOAD(hd->htim, hd->var_ctrl.timert);
  if (HAL_TIM_Base_Start_IT(hd->htim) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  //*******************************************************************
  // print the menu for the first time
  //*******************************************************************


}





/********************************************************************
  Function:        printWrongLUT()
  Description:
*******************************************************************/
void printWrongLUT(htpad_t* hd)
{
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < 8; n++) {
      if (LUTshape[m][n] == 1)
        hd->ss_dat.data_pixel[m][n] = 2732;
    }
  }
}


/********************************************************************
  Function:        loop()
  Description:
*******************************************************************/
void loop(htpad_t* hd)
{

  /* check if the timer interrupt set the hd->var_ctrl.NewDataAvailable flag. If so
     read the new raw pixel data via I2C */
  if(hd->var_ctrl.NewDataAvailable)
  {
    readblockinterrupt(hd);
    hd->var_ctrl.NewDataAvailable = 0;
  }


//#ifdef SERIALMODE
//  checkSerial(hd);
//#endif
  if (hd->var_ctrl.state)
  { // hd->var_ctrl.state is 1 when all raw sensor voltages are read for this picture

    sort_data(hd); // sort the data if the reading routing is done
    hd->var_ctrl.state = 0; // reset the hd->var_ctrl.state to sample new raw pixel data
    //*******************************************************************
    // SERIAL OUTPUT
    //*******************************************************************
      // here the print functions are called for serial monitor output
      // (there was activate in checkSerial() function before)

      // print final pixel temperatures in dK
      if (hd->var_ctrl.print_state == 1 || hd->var_ctrl.print_state == 4)
      {
        calculate_pixel_temp(hd);
        if(hd->var_ctrl.print_state == 4)
        {

        }else
        {
          print_final_array(hd);
        }
        hd->var_ctrl.print_state = 0;
      }else if (hd->var_ctrl.print_state == 2)
      {
        print_RAM_array(hd);
        hd->var_ctrl.print_state = 0;
      }else
      if (hd->var_ctrl.print_state == 3)
      {
        print_calc_steps2(hd);
        hd->var_ctrl.print_state = 0;
      }
  }

}


/********************************************************************
 ********************************************************************
    - - - PART 2: HTPAd FUNCTIONS - - -
    calcPixC()
    calculate_pixel_temp()
    pixel_masking()
    readblockinterrupt()
    read_eeprom()
    read_EEPROM_byte( uint8_t addr)
    read_sensor_register()
    sort_data()
    write_calibration_settings_to_sensor()
    write_sensor_byte( uint8_t addr, uint8_t input)
 ********************************************************************
 ********************************************************************/



/********************************************************************
   Function:      calcPixC
   Description:   calculates the pixel constants with the unscaled
                  values from EEPROM
 *******************************************************************/
void calcPixC(htpad_t* hd)
{

  /* uses the formula from datasheet:

                     PixC_uns[m][n]*(PixCmax-PixCmin)               hd->eep.epsilon   GlobalGain
      PixC[m][n] = ( -------------------------------- + PixCmin ) * ------- * ----------
                                  65535                               100        1000
  */

  double pixcij;
  hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap

  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      pixcij = (double)hd->eep.pixcmax;
      pixcij -= (double)hd->eep.pixcmin;
      pixcij /= (double)65535.0;
      pixcij *= (double) * hd->eep.pixc2;
      pixcij += (double)hd->eep.pixcmin;
      pixcij /= (double)100.0;
      pixcij *= (double)hd->eep.epsilon;
      pixcij /= (double)10000.0;
      pixcij *= (double)hd->eep.globalgain;
      pixcij += 0.5;

      *hd->eep.pixc2 = (uint32_t)pixcij;
      hd->eep.pixc2++;
    }
  }
  hd->eep.lastepsilon = hd->eep.epsilon;
}


/********************************************************************
   Function:        calculate_pixel_temp()
   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table
 *******************************************************************/
void calculate_pixel_temp(htpad_t* hd)
{

  int64_t vij_pixc_and_pcscaleval;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
  signed long pixel;
  hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap


  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (hd->ss_dat.Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = hd->ss_dat.Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;


  for (int m = 0; m < DevConst.PixelPerColumn; m++)
  {
    for (int n = 0; n < DevConst.PixelPerRow; n++)
    {

      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      pixel = (signed long) hd->ss_dat.data_pixel[m][n];

      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)hd->eep.thgrad[m][n] * (int32_t)hd->ss_dat.ptat_av_uint16) / (int32_t)hd->var_ctrl.gradscale_div);
      pixel -= (int32_t)hd->eep.thoffset[m][n];

      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= hd->ss_dat.eloffset[m % DevConst.RowPerBlock][n];
      }
      else { // bottom half
        pixel -= hd->ss_dat.eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }

      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2) {      // top half
        hd->var_ctrl.vddcompgrad_n = hd->eep.vddcompgrad[m % DevConst.RowPerBlock][n];
        hd->var_ctrl.vddcompoff_n = hd->eep.vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else {       // bottom half
        hd->var_ctrl.vddcompgrad_n = hd->eep.vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        hd->var_ctrl.vddcompoff_n = hd->eep.vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = hd->var_ctrl.vddcompgrad_n * hd->ss_dat.ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / hd->var_ctrl.vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + hd->var_ctrl.vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( hd->ss_dat.vdd_av_uint16 - hd->eep.vddth1 - ((hd->eep.vddth2 - hd->eep.vddth1) / (hd->eep.ptatth2 - hd->eep.ptatth1)) * (hd->ss_dat.ptat_av_uint16  - hd->eep.ptatth1));
      vdd_calc_steps = vdd_calc_steps / hd->var_ctrl.vddscoff_div;
      pixel -= vdd_calc_steps;

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / *hd->eep.pixc2);
      hd->eep.pixc2++;
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += hd->eep.globaloff;

      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      hd->ss_dat.data_pixel[m][n] = (uint16_t)pixel;

    }
  }

  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  pixel_masking(hd);


}


/********************************************************************
   Function:      calc_timert(uint8_t clk, uint8_t mbit)
   Description:   calculate the duration of the timer which reads the sensor blocks
 *******************************************************************/
uint16_t calc_timert(uint8_t clk, uint8_t mbit)
{

  float a;
  uint16_t calculated_timer_duration;

  float Fclk_float = 12000000.0 / 63.0 * (double)clk + 1000000.0;    // calc clk in Hz
  a = 32.0 * ((float)pow(2, (uint8_t)(mbit & 0b00001111)) + 4.0) / Fclk_float;

  calculated_timer_duration =(uint16_t)(0.98 * a * 1000000); // c in s | timer_duration in µs
  return calculated_timer_duration;
}




/********************************************************************
   Function:        void pixel_masking()
   Description:     repair dead pixel by using the average of the neighbors
 *******************************************************************/
void pixel_masking(htpad_t* hd)
{

  uint8_t number_neighbours[ALLOWED_DEADPIX];
  uint32_t temp_defpix[ALLOWED_DEADPIX];

  for (int i = 0; i < hd->eep.nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half
    if (hd->eep.deadpixadr[i] < (uint16_t)(NUMBER_OF_PIXEL / 2)) {

      if ( (hd->eep.deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) - 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (hd->eep.deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) - 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (hd->eep.deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW)][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (hd->eep.deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) + 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (hd->eep.deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) + 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (hd->eep.deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) + 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (hd->eep.deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW)][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (hd->eep.deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) - 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

    }

    // bottom half
    else {

      if ( (hd->eep.deadpixmask[i] & 1 )  == 1 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) + 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (hd->eep.deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) + 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (hd->eep.deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW)][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (hd->eep.deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) - 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (hd->eep.deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) - 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (hd->eep.deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) - 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (hd->eep.deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW)][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (hd->eep.deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + hd->ss_dat.data_pixel[(hd->eep.deadpixadr[i] / PIXEL_PER_ROW) + 1][(hd->eep.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    hd->ss_dat.data_pixel[hd->eep.deadpixadr[i] / PIXEL_PER_ROW][hd->eep.deadpixadr[i] % PIXEL_PER_ROW] = temp_defpix[i];

  }

}


/********************************************************************
   Function:        void readblockinterrupt()
   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when hd->var_ctrl.read_eloffset_next_pic is set)
 *******************************************************************/
void readblockinterrupt(htpad_t* hd)
{

  uint8_t bottomblock;


  hd->var_ctrl.ReadingRoutineEnable = 0;
  __HAL_TIM_SET_COUNTER(hd->htim, 0);//Reset counter

  // wait for end of conversion bit (~27ms)

  // check EOC bit
  read_sensor_register(hd, STATUS_REGISTER, (uint8_t*)&hd->ss_dat.statusreg, 1);
  while ((hd->ss_dat.statusreg & 0x01) == 0)
  {
    read_sensor_register(hd,  STATUS_REGISTER, (uint8_t*)&hd->ss_dat.statusreg, 1);
  }
  // get data of top half:
  read_sensor_register(hd, TOP_HALF, (uint8_t*)&hd->ss_dat.RAMoutput[hd->var_ctrl.read_block_num], BLOCK_LENGTH);
  // get data of bottom half:
  bottomblock = (uint8_t)((uint8_t)(NUMBER_OF_BLOCKS + 1) * 2 - hd->var_ctrl.read_block_num - 1);
  read_sensor_register(hd,  BOTTOM_HALF, (uint8_t*)&hd->ss_dat.RAMoutput[bottomblock], BLOCK_LENGTH);


  hd->var_ctrl.read_block_num++;

  if (hd->var_ctrl.read_block_num < NUMBER_OF_BLOCKS)
  {

    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  x  |  x  |   1   |    0     |   0   |    1   |
    write_sensor_byte(hd, SENSOR_ADDRESS, CONFIGURATION_REGISTER, (uint8_t)(0x09 + (0x10 * hd->var_ctrl.read_block_num) + (0x04 * hd->var_ctrl.switch_ptat_vdd)));
  }else
  {
    //*******************************************************************
    // all blocks for the current image are sampled, now check if its time
    // to get new hd->pve_buf.electrical offsets and/or for switching PTAT and VDD
    //*******************************************************************

    if (hd->var_ctrl.read_eloffset_next_pic)
    {
      hd->var_ctrl.read_eloffset_next_pic = 0;

      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
      write_sensor_byte(hd, SENSOR_ADDRESS, CONFIGURATION_REGISTER, (uint8_t)(0x0B + (0x04 * hd->var_ctrl.switch_ptat_vdd)));
      hd->pve_buf.new_offsets = 1;
    }else
    {
      if (hd->var_ctrl.picnum > 1)  hd->var_ctrl.state = 1; // hd->var_ctrl.state = 1 means that all required blocks are sampled
      hd->var_ctrl.picnum++; // increase the picture counter
      // check if the next sample routine should include hd->pve_buf.electrical offsets
      if ((uint8_t)(hd->var_ctrl.picnum % READ_ELOFFSET_EVERYX) == 0)
        hd->var_ctrl.read_eloffset_next_pic = 1;


      if (DevConst.PTATVDDSwitch)
        hd->var_ctrl.switch_ptat_vdd ^= 1;


      hd->var_ctrl.read_block_num = 0;

      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
      write_sensor_byte(hd, SENSOR_ADDRESS, CONFIGURATION_REGISTER, (uint8_t)(0x09 + (0x04 * hd->var_ctrl.switch_ptat_vdd)));
    }
  }

  __HAL_TIM_SET_AUTORELOAD(hd->htim, hd->var_ctrl.timert);
  hd->var_ctrl.ReadingRoutineEnable = 1;
}


/********************************************************************
   Function:        void read_eeprom()
   Description:     read all values from eeprom
 *******************************************************************/
void read_eeprom(htpad_t* hd)
{
  int m = 0;
  int n = 0;
  uint8_t b[4];
  hd->eep.id = read_EEPROM_byte(hd, E_ID4) << 24 | read_EEPROM_byte(hd, E_ID3) << 16 | read_EEPROM_byte(hd, E_ID2) << 8 | read_EEPROM_byte(hd, E_ID1);
  hd->eep.mbit_calib = read_EEPROM_byte(hd, E_MBIT_CALIB);
  hd->eep.bias_calib = read_EEPROM_byte(hd, E_BIAS_CALIB);
  hd->eep.clk_calib = read_EEPROM_byte(hd, E_CLK_CALIB);
  hd->eep.bpa_calib = read_EEPROM_byte(hd, E_BPA_CALIB);
  hd->eep.pu_calib = read_EEPROM_byte(hd, E_PU_CALIB);
  hd->eep.mbit_user = read_EEPROM_byte(hd, E_MBIT_USER);
  hd->eep.bias_user = read_EEPROM_byte(hd, E_BIAS_USER);
  hd->eep.clk_user = read_EEPROM_byte(hd, E_CLK_USER);
  hd->eep.bpa_user = read_EEPROM_byte(hd, E_BPA_USER);
  hd->eep.pu_user = read_EEPROM_byte(hd, E_PU_USER);
  hd->eep.vddth1 = read_EEPROM_byte(hd, E_VDDTH1_2) << 8 | read_EEPROM_byte(hd, E_VDDTH1_1);
  hd->eep.vddth2 = read_EEPROM_byte(hd, E_VDDTH2_2) << 8 | read_EEPROM_byte(hd, E_VDDTH2_1);
  hd->eep.vddscgrad = read_EEPROM_byte(hd, E_VDDSCGRAD);
  hd->eep.vddscoff = read_EEPROM_byte(hd, E_VDDSCOFF);
  hd->eep.ptatth1 = read_EEPROM_byte(hd, E_PTATTH1_2) << 8 | read_EEPROM_byte(hd, E_PTATTH1_1);
  hd->eep.ptatth2 = read_EEPROM_byte(hd, E_PTATTH2_2) << 8 | read_EEPROM_byte(hd, E_PTATTH2_1);
  hd->eep.nrofdefpix = read_EEPROM_byte(hd, E_NROFDEFPIX);
  hd->eep.gradscale = read_EEPROM_byte(hd, E_GRADSCALE);
  hd->eep.tablenumber = read_EEPROM_byte(hd, E_TABLENUMBER2) << 8 | read_EEPROM_byte(hd, E_TABLENUMBER1);
  hd->eep.arraytype = read_EEPROM_byte(hd, E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(hd, E_PTATGR_1);
  b[1] = read_EEPROM_byte(hd, E_PTATGR_2);
  b[2] = read_EEPROM_byte(hd, E_PTATGR_3);
  b[3] = read_EEPROM_byte(hd, E_PTATGR_4);
  hd->eep.ptatgr_float = *(float*)b;
  b[0] = read_EEPROM_byte(hd, E_PTATOFF_1);
  b[1] = read_EEPROM_byte(hd, E_PTATOFF_2);
  b[2] = read_EEPROM_byte(hd, E_PTATOFF_3);
  b[3] = read_EEPROM_byte(hd, E_PTATOFF_4);
  hd->eep.ptatoff_float = *(float*)b;
  b[0] = read_EEPROM_byte(hd, E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(hd, E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(hd, E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(hd, E_PIXCMIN_4);
  hd->eep.pixcmin = *(float*)b;
  b[0] = read_EEPROM_byte(hd, E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(hd, E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(hd, E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(hd, E_PIXCMAX_4);
  hd->eep.pixcmax = *(float*)b;
  hd->eep.epsilon = read_EEPROM_byte(hd, E_EPSILON);
  hd->eep.globaloff = read_EEPROM_byte(hd, E_GLOBALOFF);
  hd->eep.globalgain = read_EEPROM_byte(hd, E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(hd, E_GLOBALGAIN_1);


  // for (int m = 0; m < DevConst.PixelPerColumn; m++) {
  //   for (int n = 0; n < DevConst.PixelPerRow; n++) {

  // --- DeadPixAdr ---
  for (int i = 0; i < hd->eep.nrofdefpix; i++) {
    hd->eep.deadpixadr[i] = read_EEPROM_byte(hd, E_DEADPIXADR + 2 * i + 1 ) << 8 | read_EEPROM_byte(hd, E_DEADPIXADR + 2 * i);
    if (hd->eep.deadpixadr[i] > (uint16_t)(DevConst.NumberOfPixel / 2)) {  // adaptedAdr:
      hd->eep.deadpixadr[i] = (uint16_t)(DevConst.NumberOfPixel) + (uint16_t)(DevConst.NumberOfPixel / 2) - hd->eep.deadpixadr[i] + 2 * (uint16_t)(hd->eep.deadpixadr[i] % DevConst.PixelPerRow ) - DevConst.PixelPerRow;
    }
  }


  // --- DeadPixMask ---
  for (int i = 0; i < hd->eep.nrofdefpix; i++) {
    hd->eep.deadpixmask[i] = read_EEPROM_byte(hd, E_DEADPIXMASK + i);
  }


  // --- Thgrad_ij, ThOffset_ij and P_ij ---
  m = 0;
  n = 0;
  hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap // reset pointer to initial address
  // top half
  for (int i = 0; i < (uint16_t)(DevConst.NumberOfPixel / 2); i++) {
    hd->eep.thgrad[m][n] = read_EEPROM_byte(hd, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_THGRAD + 2 * i);
    hd->eep.thoffset[m][n] = read_EEPROM_byte(hd, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_THOFFSET + 2 * i);
    *(hd->eep.pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(hd, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_PIJ + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (uint8_t)(DevConst.PixelPerColumn - 1);
  n = 0;
  for (int i = (uint16_t)(DevConst.NumberOfPixel / 2); i < (uint16_t)(DevConst.NumberOfPixel); i++) {
    hd->eep.thgrad[m][n] = read_EEPROM_byte(hd, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_THGRAD + 2 * i);
    hd->eep.thoffset[m][n] = read_EEPROM_byte(hd, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_THOFFSET + 2 * i);
    *(hd->eep.pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(hd, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_PIJ + 2 * i);
    n++;

    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

  //---VddCompGrad and VddCompOff---
  // top half
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < (uint16_t)(DevConst.PixelPerBlock); i++) {
    hd->eep.vddcompgrad[m][n] = read_EEPROM_byte(hd, E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_VDDCOMPGRAD + 2 * i);
    hd->eep.vddcompoff[m][n] = read_EEPROM_byte(hd, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = (uint8_t)(DevConst.RowPerBlock * 2 - 1);
  n = 0;
  for (int i = (uint16_t)(DevConst.PixelPerBlock); i < (uint16_t)(DevConst.PixelPerBlock * 2); i++) {
    hd->eep.vddcompgrad[m][n] = read_EEPROM_byte(hd, E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_VDDCOMPGRAD + 2 * i);
    hd->eep.vddcompoff[m][n] = read_EEPROM_byte(hd, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(hd, E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  DevConst.PixelPerRow) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

}





/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)
   Description:     read sensor register
 *******************************************************************/
void read_sensor_register(htpad_t* hd, uint16_t addr, uint8_t *dest, uint16_t n)
{
	HAL_I2C_Mem_Read(hd->hi2c, SENSOR_ADDRESS << 1, addr, I2C_MEMADD_SIZE_8BIT, dest, n, HAL_MAX_DELAY);
}


/********************************************************************
   Function:        void sort_data()
   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd
 *******************************************************************/
void sort_data(htpad_t* hd)
{

  uint32_t sum = 0;
  uint16_t pos = 0;

  for (int m = 0; m < DevConst.RowPerBlock; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      /*
         for example: a normal line of hd->ss_dat.RAMoutput for HTPAd80x64 looks like:
         hd->ss_dat.RAMoutput[0][] = [ PTAT(MSB), PTAT(LSB), DATA0[MSB], DATA0[LSB], DATA1[MSB], DATA1[LSB], ... , DATA640[MSB], DATA640LSB];
                                                      |
                                                      |-- DATA_Pos = 2 (first data uint8_t)
      */
      pos = (uint16_t)(2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);



      /******************************************************************************************************************
        new PIXEL values
      ******************************************************************************************************************/
      for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
        // top half
        hd->ss_dat.data_pixel[m + i * DevConst.RowPerBlock][n] =
          (uint16_t)(hd->ss_dat.RAMoutput[i][pos] << 8 | hd->ss_dat.RAMoutput[i][pos + 1]);
        // bottom half
        hd->ss_dat.data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.RowPerBlock][n] =
          (uint16_t)(hd->ss_dat.RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | hd->ss_dat.RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]);
      }


      /******************************************************************************************************************
        new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
      ******************************************************************************************************************/
      if (hd->var_ctrl.picnum % ELOFFSETS_BUFFER_SIZE == 1) {
        if ((!hd->ss_dat.eloffset[m][n]) || (hd->var_ctrl.picnum < ELOFFSETS_FILTER_START_DELAY)) {
          // top half
          hd->ss_dat.eloffset[m][n] = (uint16_t)(hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          // bottom half
          hd->ss_dat.eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (uint16_t)(hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          hd->pve_buf.use_eloffsets_buffer = 1;

        }
        else {
          // use a moving average filter
          // top half
          sum = (uint32_t)hd->ss_dat.eloffset[m][n] * (uint32_t)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (uint32_t)(hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
          hd->ss_dat.eloffset[m][n] = (uint16_t)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
          // bottom half
          sum = (uint32_t)hd->ss_dat.eloffset[2 * DevConst.RowPerBlock - 1 - m][n] * (uint32_t)(ELOFFSETS_BUFFER_SIZE - 1);
          sum += (uint32_t)(hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | hd->ss_dat.RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
          hd->ss_dat.eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (uint16_t)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
        }
      }

    }

  }



  /******************************************************************************************************************
    new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (hd->var_ctrl.switch_ptat_vdd == 1) {
    sum = 0;
    // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
      // block top half
      sum += (uint16_t)(hd->ss_dat.RAMoutput[i][DevConst.PTATPos] << 8 | hd->ss_dat.RAMoutput[i][DevConst.PTATPos + 1]);
      // block bottom half
      sum += (uint16_t)(hd->ss_dat.RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | hd->ss_dat.RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]);
    }
    hd->ss_dat.ptat_av_uint16 = (uint16_t)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
    hd->ss_dat.Ta = (uint16_t)((uint16_t)hd->ss_dat.ptat_av_uint16 * (float)hd->eep.ptatgr_float + (float)hd->eep.ptatoff_float);


    hd->pve_buf.ptat_buffer[hd->pve_buf.ptat_i] = hd->ss_dat.ptat_av_uint16;
    hd->pve_buf.ptat_i++;
    if (hd->pve_buf.ptat_i == PTAT_BUFFER_SIZE) {
      if (hd->pve_buf.use_ptat_buffer == 0) {
        printf(" | PTAT buffer complete");
        hd->pve_buf.use_ptat_buffer = 1;
      }
      hd->pve_buf.ptat_i = 0;
    }

    if (hd->pve_buf.use_ptat_buffer) {
      // now overwrite the old ptat average
      sum = 0;
      for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
        sum += hd->pve_buf.ptat_buffer[i];
      }
      hd->ss_dat.ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
    }


  }


  /******************************************************************************************************************
    new VDD values (store them in VDD buffer and calculate the average for pixel compensation
  ******************************************************************************************************************/
  if (hd->var_ctrl.switch_ptat_vdd == 0) {
    sum = 0;
    // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
    for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
      // block top half
      sum += (uint16_t)(hd->ss_dat.RAMoutput[i][DevConst.VDDPos] << 8 | hd->ss_dat.RAMoutput[i][DevConst.VDDPos + 1]);
      // block bottom half
      sum += (uint16_t)(hd->ss_dat.RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | hd->ss_dat.RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]);
    }
    hd->ss_dat.vdd_av_uint16 = (uint16_t)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));


    // write into vdd buffer
    hd->pve_buf.vdd_buffer[hd->pve_buf.vdd_i] = hd->ss_dat.vdd_av_uint16;
    hd->pve_buf.vdd_i++;
    if (hd->pve_buf.vdd_i == VDD_BUFFER_SIZE) {
      if (hd->pve_buf.use_vdd_buffer == 0) {
        printf(" | VDD buffer complete");
        hd->pve_buf.use_vdd_buffer = 1;
      }
      hd->pve_buf.vdd_i = 0;
    }
    if (hd->pve_buf.use_vdd_buffer) {
      sum = 0;
      for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
        sum += hd->pve_buf.vdd_buffer[i];
      }
      // now overwrite the old vdd average
      hd->ss_dat.vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
    }

  }

}


/********************************************************************
   Function:        void write_calibration_settings_to_sensor()
   Description:     write calibration data (from eeprom) to trim registers (sensor)
 *******************************************************************/
void write_calibration_settings_to_sensor(htpad_t* hd)
{

  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER1, hd->eep.mbit_calib);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER2, hd->eep.bias_calib);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER3, hd->eep.bias_calib);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER4, hd->eep.clk_calib);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER5, hd->eep.bpa_calib);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER6, hd->eep.bpa_calib);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER7, hd->eep.pu_calib);
  HAL_Delay(5);
}


/********************************************************************
   Function:        void read_EEPROM_byte(uint16_t eeaddress )
   Description:     read eeprom register as 8
   Dependencies:    register address (address)
 *******************************************************************/
void write_EEPROM_byte(htpad_t* hd, uint16_t address, uint8_t content )
{
  uint8_t data[] = {address >> 8, address & 0xff,content};
  HAL_I2C_Master_Transmit(hd->hi2c, EEPROM_ADDRESS << 1, data, 3, 100);
}


/********************************************************************
   Function:        void read_EEPROM_byte(uint16_t eeaddress )
  Description:     read eeprom register as 8
  Dependencies:    register address (address)
*******************************************************************/
uint8_t read_EEPROM_byte(htpad_t* hd, uint16_t address)
{
  uint8_t rdata = 0xFF;
  uint8_t pData[] = {address >> 8, address & 0xFF};

  HAL_I2C_Master_Transmit(hd->hi2c, EEPROM_ADDRESS << 1, pData, 2, 100);
  HAL_I2C_Master_Receive(hd->hi2c, EEPROM_ADDRESS << 1, &rdata, 1, 100);
  return rdata;
}



/********************************************************************
   Function:        void write_sensor_byte( uint16_t addr)
   Description:     write to sensor register
   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
void write_sensor_byte(htpad_t* hd, uint8_t deviceaddress, uint8_t registeraddress, uint8_t input)
{

  uint8_t byte_dat[]= {registeraddress, input };
  HAL_I2C_Master_Transmit(hd->hi2c, deviceaddress << 1, byte_dat, 2, 100);

}



/********************************************************************
   Function:        void write_user_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_user_settings_to_sensor(htpad_t* hd)
{

  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER1, hd->eep.mbit_user);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER2, hd->eep.bias_user);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER3, hd->eep.bias_user);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER4, hd->eep.clk_user);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER5, hd->eep.bpa_user);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER6, hd->eep.bpa_user);
  HAL_Delay(5);
  write_sensor_byte(hd, SENSOR_ADDRESS, TRIM_REGISTER7, hd->eep.pu_user);
  HAL_Delay(5);
}






/********************************************************************
 ********************************************************************
    - - - PART 4: SERIAL FUNCTIONS - - -
  checkSerial()
  print_eeprom_header()
  print_eeprom_hex()
  print_menu()
 ********************************************************************
 ********************************************************************/



/********************************************************************
   Function:        print_final_array()
   Description:
 *******************************************************************/
void print_final_array(htpad_t* hd)
{
  printf("\n\n---final array ---\n");
  for (int m = 0; m < DevConst.PixelPerColumn; m++)
  {
    for (int n = 0; n < DevConst.PixelPerRow; n++)
    {
      printf("%d",hd->ss_dat.data_pixel[m][n]-2731);
      printf("\t");
    }
    printf("\n");
  }
}

/********************************************************************
   Function:        print_RAM_array()
   Description:
 *******************************************************************/
void print_RAM_array(htpad_t* hd)
{
  printf("\n\n\n---ram array ---\n");
  for (int m = 0; m < (2 * NUMBER_OF_BLOCKS + 2); m++)
  {
    for (int n = 0; n < BLOCK_LENGTH; n++)
    {
      printf("%x",hd->ss_dat.RAMoutput[m][n]);
      printf("\t");
    }
    printf("\n");
  }
  printf("\n\n\n");
}


/********************************************************************
   Function:        checkSerial()
   Description:
 *******************************************************************/
void checkSerial(htpad_t* hd)
{
	uint8_t serial_input;
  if(serial_available())
  {
	  serial_input = serial_read();
  }else
  {
	  return;
  }
  switch (serial_input)
  {
    case 0xFF:
      //nothing
      break;

    case 'a':
        hd->var_ctrl.print_state = 1;
      break;
    case 'b':
        hd->var_ctrl.print_state = 2;
      break;

    case 'c':
        hd->var_ctrl.print_state = 3;
      break;


    case 'm':
      while (hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      print_menu();
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'd':
      while (hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      print_eeprom_hex(hd);
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'e':
      while ( hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      print_eeprom_header(hd);
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'f':
      while ( hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      printf("\n\n\n---VddCompGrad---\n");
      for (int m = 0; m < (DevConst.RowPerBlock * 2); m++) {
        for (int n = 0; n <  DevConst.PixelPerRow; n++) {
          printf("%d\t", hd->eep.vddcompgrad[m][n]);
        }
        printf("\n");
      }
      printf("\n\n\n");
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'g':
      while (hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      printf("\n\n\n---VddCompOff---\n");
      for (int m = 0; m < ( DevConst.RowPerBlock * 2); m++) {
        for (int n = 0; n <  DevConst.PixelPerRow; n++) {
          printf("%d\t", hd->eep.vddcompoff[m][n]);
        }
        printf("\n");
      }
      printf("\n\n\n");
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'h':
      while (hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      printf("\n\n\n---ThGrad---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n <  DevConst.PixelPerRow; n++) {
          printf("%d\t", hd->eep.thgrad[m][n]);
        }
        printf("\n");
      }
      printf("\n\n\n");
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;



    case 'i':
      while ( hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      // print ThOffset in serial monitor
      printf("\n\n\n---ThOffset---\n");
      for (int m = 0; m < DevConst.PixelPerColumn; m++) {
        for (int n = 0; n < DevConst.PixelPerRow; n++) {
          printf("%d\t", hd->eep.thoffset[m][n]);
        }
        printf("\n");
      }
      printf("\n\n\n");
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'j':
      while (hd->var_ctrl.state);
      hd->var_ctrl.ReadingRoutineEnable = 0;
      // print PixC in serial monitor
      printf("\n\n\n---PixC---\n");
      hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap
      for (int m = 0; m < DevConst.PixelPerColumn; m++)
      {
        for (int n = 0; n < DevConst.PixelPerRow; n++)
        {
          printf("%ld\t",*(hd->eep.pixc2 + m * DevConst.PixelPerRow + n));
        }
        printf("\n");
      }
      printf("\n\n\n");
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'k':
    	hd->var_ctrl.ReadingRoutineEnable = 0;
      __HAL_TIM_SET_COUNTER(hd->htim,0);
      printf("\n\n\n---Increase emissivity---\n");
      printf("old emissivity: \t%d\n",hd->eep.epsilon);
      printf("new emissivity: \t");
      if (hd->eep.epsilon < 100) {
    	  hd->eep.epsilon++;

        write_EEPROM_byte(hd,E_EPSILON, hd->eep.epsilon);


        // calculate pixcij with new hd->eep.epsilon
        hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap
        double d = (double)hd->eep.epsilon / (double)hd->eep.lastepsilon;
        for (int m = 0; m < DevConst.PixelPerColumn; m++) {
          for (int n = 0; n < DevConst.PixelPerRow; n++) {
            *(hd->eep.pixc2 + m * DevConst.PixelPerRow + n) = (uint32_t)((double) * (hd->eep.pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
          }
        }
        hd->eep.lastepsilon = hd->eep.epsilon;
        printf("%d (new emissivity is stored in the EEPROM now)\n",hd->eep.epsilon);
      }
      else {
        printf("%d (you cannot set the emissivity higher than 100%%)\n",hd->eep.epsilon);
      }
      HAL_Delay(1000);
      __HAL_TIM_SET_AUTORELOAD(hd->htim, hd->var_ctrl.timert);
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;

    case 'l':
      hd->var_ctrl.ReadingRoutineEnable = 0;
      __HAL_TIM_SET_COUNTER(hd->htim,0);
      printf("\n\n\n---Decrease emissivity---");
      printf("\nold emissivity: \t%d",hd->eep.epsilon);
      printf("\nnew emissivity: \t");
      if (hd->eep.epsilon > 0) {
        hd->eep.epsilon--;
        write_EEPROM_byte(hd, E_EPSILON, hd->eep.epsilon);
        // calculate pixcij with new hd->eep.epsilon
        hd->eep.pixc2 = hd->eep.pixc2_0; // set pointer to start address of the allocated heap
        double d = (double)hd->eep.epsilon / (double)hd->eep.lastepsilon;
        for (int m = 0; m < DevConst.PixelPerColumn; m++) {
          for (int n = 0; n < DevConst.PixelPerRow; n++) {
            *(hd->eep.pixc2 + m * DevConst.PixelPerRow + n) = (uint32_t)((double) * (hd->eep.pixc2 + m * DevConst.PixelPerRow + n) * (double)d);
          }
        }
        hd->eep.lastepsilon = hd->eep.epsilon;
        printf("%d (new emissivity is stored in the EEPROM now)",hd->eep.epsilon);
      }
      else {
        printf("%d",hd->eep.epsilon);
        printf(" (you cannot set the emissivity lower as 0%)");
      }
      HAL_Delay(1000);
      __HAL_TIM_SET_AUTORELOAD(hd->htim, hd->var_ctrl.timert);
      hd->var_ctrl.ReadingRoutineEnable = 1;
      break;
  }
}


/********************************************************************
   Function:        print_calc_steps()
   Description:     print every needed step for temperature calculation + pixel masking
 *******************************************************************/
void print_calc_steps2(htpad_t* hd)
{
  int64_t vij_pixc_and_pcscaleval;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
  signed long pixel;
  hd->eep.pixc2 = hd->eep.pixc2_0;

  printf("\n\ncalculate the average of VDD and PTAT buffer\n");

  printf("PTATbuf[%d",PTAT_BUFFER_SIZE);
  printf("] = { ");
  for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
    printf("%d",hd->pve_buf.ptat_buffer[i]);
    if (i < (PTAT_BUFFER_SIZE - 1))
      printf(" , ");
    else
      printf(" }");
  }
  printf("\nPTAT_average = %d",hd->ss_dat.ptat_av_uint16);

  printf("\nVDDbuf[%d",VDD_BUFFER_SIZE);
  printf("] = { ");
  for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
    printf("%d",hd->pve_buf.vdd_buffer[i]);
    if (i < (VDD_BUFFER_SIZE - 1))
      printf(" , ");
    else
      printf(" }");
  }
  printf("\nVDD_average = %d",hd->ss_dat.vdd_av_uint16);

  printf("\n\ncalculate ambient temperatur (hd->ss_dat.Ta)\n");
  printf("hd->ss_dat.Ta = %d",hd->ss_dat.ptat_av_uint16);
  printf(" * ");
  printf("%.5f",hd->eep.ptatgr_float);
  printf(" + ");
  printf("%.5f",hd->eep.ptatoff_float);
  printf(" = %d",hd->ss_dat.Ta);
  printf(" (Value is given in dK)");


  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (hd->ss_dat.Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = hd->ss_dat.Ta - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;

  printf("\n\nprint all calculation steps for each pixel\n");
  printf("table columns:\n");
  printf("No\tpixel number\n");
  printf("i\trepresents the row of the pixel\n");
  printf("j\trepresents the column of the pixel\n");
  printf("Vij\tis row pixel voltages (digital); readout from the RAM\n");
  printf("I\tis the thermal offset compensated voltage\n");
  printf("II\tis the thermal and electrical offset compensated voltage\n");
  printf("III\tis the Vdd compensated voltage\n");
  printf("IV\tis the sensivity compensated IR voltage\n");
  printf("T[dK]\tis final pixel temperature in dK (deci Kelvin)\n");
  printf("T[°C]\tis final pixel temperature in °C\n");

  printf("\n\nNo\ti\tj\tVij\tI\tII\tIII\tIV\tT[dK]\tT[°C]\n");
  printf("-----------------------------------------------------------------------------\n");


  for (int m = 0; m < DevConst.PixelPerColumn; m++) {
    for (int n = 0; n < DevConst.PixelPerRow; n++) {

      printf("%d",m * DevConst.PixelPerRow + n);
      printf("\t%d",m);
      printf("\t%d",n);
      /******************************************************************************************************************
         step 1: use a variable with bigger data format for the compensation steps
       ******************************************************************************************************************/
      pixel = (signed long) hd->ss_dat.data_pixel[m][n];
      printf("\t%ld", pixel);
      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)hd->eep.thgrad[m][n] * (int32_t)hd->ss_dat.ptat_av_uint16) / (int32_t)hd->var_ctrl.gradscale_div);
      pixel -= (int32_t)hd->eep.thoffset[m][n];
      printf("\t%ld",pixel);
      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      if (m < DevConst.PixelPerColumn / 2) { // top half
        pixel -= hd->ss_dat.eloffset[m % DevConst.RowPerBlock][n];
      }
      else { // bottom half
        pixel -= hd->ss_dat.eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      printf("\t%ld",pixel);
      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      // first select VddCompGrad and VddCompOff for pixel m,n:
      if (m < DevConst.PixelPerColumn / 2) {      // top half
        hd->var_ctrl.vddcompgrad_n = hd->eep.vddcompgrad[m % DevConst.RowPerBlock][n];
        hd->var_ctrl.vddcompoff_n = hd->eep.vddcompoff[m % DevConst.RowPerBlock][n];
      }
      else {       // bottom half
        hd->var_ctrl.vddcompgrad_n = hd->eep.vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
        hd->var_ctrl.vddcompoff_n = hd->eep.vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
      }
      // now do the vdd calculation
      vdd_calc_steps = hd->var_ctrl.vddcompgrad_n * hd->ss_dat.ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / hd->var_ctrl.vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + hd->var_ctrl.vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( hd->ss_dat.vdd_av_uint16 - hd->eep.vddth1 - ((hd->eep.vddth2 - hd->eep.vddth1) / (hd->eep.ptatth2 - hd->eep.ptatth1)) * (hd->ss_dat.ptat_av_uint16  - hd->eep.ptatth1));
      vdd_calc_steps = vdd_calc_steps / hd->var_ctrl.vddscoff_div;
      pixel -= vdd_calc_steps;
      printf("\t%ld",pixel);
      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
      pixel =  (int32_t)(vij_pixc_and_pcscaleval / *hd->eep.pixc2);
      hd->eep.pixc2++;
      printf("\t%ld",pixel);
      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      table_row = pixel + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += hd->eep.globaloff;
      printf("\t%ld",pixel);
      printf("\t %.0f\n",(float)((pixel - 2732) / 10.0));
      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      hd->ss_dat.data_pixel[m][n] = (uint16_t)pixel;

    }
  }

  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  pixel_masking(hd);


}



/********************************************************************
   Function:        print_eeprom_header()
   Description:
 *******************************************************************/
void print_eeprom_header(htpad_t* hd)
{
  printf("data\t\tregister\ttype\t\tvalue\n");
  printf("------------------------------------------------------------\n");
  printf("PixCmin\t\t0x00-0x03\tfloat\t\t");
  printf("%0.0f",hd->eep.pixcmin);
  printf("PixCmax\t\t0x04-0x07\tfloat\t\t");
  printf("%0.0f",hd->eep.pixcmax);
  printf("gradScale\t0x08\t\tuint8_t\t");
  printf("%d\n",hd->eep.gradscale);
  printf("TN\t\t0x0B-0x0C\tuint16_t\t");
  printf("%d\n",hd->eep.tablenumber);
  printf("hd->eep.epsilon\t\t0x0D\t\tuint8_t\t");
  printf("%d\n",hd->eep.epsilon);
  printf("MBIT(calib)\t0x1A\t\tuint8_t\t");
  printf("%d\n",hd->eep.mbit_calib);
  printf("BIAS(calib)\t0x1B\t\tuint8_t\t");
  printf("%d\n",hd->eep.bias_calib);
  printf("CLK(calib)\t0x1C\t\tuint8_t\t");
  printf("%d\n",hd->eep.clk_calib);
  printf("BPA(calib)\t0x1D\t\tuint8_t\t");
  printf("%d\n",hd->eep.bpa_calib);
  printf("PU(calib)\t0x1E\t\tuint8_t\t");
  printf("%d\n",hd->eep.pu_calib);
  printf("Arraytype\t0x22\t\tuint8_t\t");
  printf("%d\n",hd->eep.arraytype);
  printf("VDDTH1\t\t0x26-0x27\tuint16_t\t");
  printf("%d\n",hd->eep.vddth1);
  printf("VDDTH2\t\t0x28-0x29\tuint16_t\t");
  printf("%d\n",hd->eep.vddth2);
  printf("PTAT-gradient\t0x34-0x37\tfloat\t\t");
  printf("%.4f\n",hd->eep.ptatgr_float);
  printf("PTAT-offset\t0x38-0x3B\tfloat\t\t");
  printf("%.4f\n",hd->eep.ptatoff_float);
  printf("PTAT(Th1)\t0x3C-0x3D\tuint16_t\t");
  printf("%d\n",hd->eep.ptatth1);
  printf("PTAT(Th2)\t0x3E-0x3F\tuint16_t\t");
  printf("%d\n",hd->eep.ptatth2);
  printf("VddScGrad\t0x4E\t\tuint8_t\t");
  printf("%d\n",hd->eep.vddscgrad);
  printf("VddScOff\t0x4F\t\tuint8_t\t");
  printf("%d\n",hd->eep.vddscoff);
  printf("GlobalOff\t0x54\t\tsigned char\t");
  printf("%d\n",hd->eep.globaloff);
  printf("GlobalGain\t0x55-0x56\tuint16_t\t");
  printf("%d\n",hd->eep.globalgain);
  printf("SensorID\t0x74-0x77\tuint32_t\t");
  printf("%ld\n",hd->eep.id);

}


/********************************************************************
   Function:        print_eeprom_hex()
   Description:     print eeprom contint as hex values
 *******************************************************************/
void print_eeprom_hex(htpad_t* hd)
{

  printf("\n\n\n---PRINT EEPROM (HEX)---\n");
  printf("\n\n\t\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    printf("- ");
  }

  for (uint16_t i = 0; i < EEPROM_SIZE; i++) {


    if (i % 16 == 0) {
      printf("\n");

      if (i < E_DEADPIXADR) {
        printf("HEADER\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else if (i < 0x00D0) {
        printf("DEADPIX\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else if (i < E_VDDCOMPGRAD) {
        printf("FREE\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else if (i < E_VDDCOMPOFF) {
        printf("VDDGRAD\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else if (i < E_THGRAD) {
        printf("VDDOFF\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else if (i < E_THOFFSET) {
        printf("THGRAD\t0x");
	    printf("%X",i);
        printf("\t|\t");
      }
      else if (i < E_PIJ) {
        printf("THOFF\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else if (i < (E_PIJ + 2*NUMBER_OF_PIXEL)) {
        printf("PIXC\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
      else {
        printf("FREE\t0x");
        printf("%X",i);
        printf("\t|\t");
      }
    }
    else {
      printf("\t");
    }

    printf("0x");
    if (read_EEPROM_byte(hd, i) < 0x10) {
      printf("0");
    }
    printf("%X",read_EEPROM_byte(hd, i));

  }

  printf("\n\n\n\ndone (m... back to menu)\n\n\n");
}


/********************************************************************
   Function:      print_menu()
   Description:
 *******************************************************************/
void print_menu(void)
{
  printf("\n\n\n***************************************************\n");
  printf("Application Shield                      /_/eimann\n");
  printf("for ESP32-DevkitC                      / /   Sensor\n");

  printf("\nYou can choose one of these options by sending the \ncharacter\n\n");
  printf("read SENSOR values:\n");
  printf("  a... final array temperatures (in deci Kelvin)\n");
  printf("  b... show all raw values (in digits)\n");
  printf("  c... show all calculation steps\n");
  printf("read EEPROM values:\n");
  printf("  d... whole eeprom content (in hexadecimal)\n");
  printf("  e... Header values\n");
  printf("  f... VddCompGrad\n");
  printf("  g... VddCompOff\n");
  printf("  h... ThGrad\n");
  printf("  i... ThOff\n");
  printf("  j... PixC (scaled)\n");
  printf("write/change EEPROM values:\n");
  printf("  k... increase emissivity by 1\n");
  printf("  l... decrease emissivity by 1\n");
  printf("\t\t\t\t\tver2.2 (dp)");
  printf("***************************************************\n\n\n\n");
}





