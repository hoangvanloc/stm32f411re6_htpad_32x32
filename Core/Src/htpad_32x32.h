/*
 * htpad_32x32.h
 *
 *  Created on: Oct 12, 2025
 *      Author: Loc
 */

#ifndef SRC_HTPAD_32X32_H_
#define SRC_HTPAD_32X32_H_
#include <stdbool.h>
#include <stdint.h>
#include "def.h"
#include "main.h"
// STRUCT WITH ALL SENSOR CHARACTERISTICS
typedef struct
{
  uint16_t NumberOfPixel;
  uint8_t NumberOfBlocks;
  uint8_t RowPerBlock;
  uint16_t PixelPerBlock;
  uint16_t PixelPerColumn;
  uint16_t PixelPerRow;
  uint8_t AllowedDeadPix;
  uint16_t TableNumber;
  uint16_t TableOffset;
  uint8_t PTATPos;
  uint8_t VDDPos;
  uint8_t PTATVDDSwitch;
  uint8_t CyclopsActive;
  uint8_t CyclopsPos;
  uint8_t DataPos;
}characteristics;
#define DevConstDEF \
{ \
  NUMBER_OF_PIXEL,\
  NUMBER_OF_BLOCKS, \
  ROW_PER_BLOCK, \
  PIXEL_PER_BLOCK, \
  PIXEL_PER_COLUMN, \
  PIXEL_PER_ROW, \
  ALLOWED_DEADPIX, \
  TABLENUMBER, \
  TABLEOFFSET, \
  PTAT_POS, \
  VDD_POS, \
  PTAT_VDD_SWITCH, \
  ATC_ACTIVE, \
  ATC_POS, \
  DATA_POS, \
}

typedef struct
{
	// EEPROM DATA
	uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
	uint8_t nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, lastepsilon, arraytype;
	uint8_t deadpixmask[ALLOWED_DEADPIX];
	uint8_t globaloff;
	uint16_t thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
	uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
	uint16_t deadpixadr[ALLOWED_DEADPIX * 2];
	uint16_t thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
	uint16_t vddcompgrad[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
	uint16_t vddcompoff[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
	uint32_t id, ptatoff;
	float ptatgr_float, ptatoff_float, pixcmin, pixcmax;
	// use a heap allocated memory to store the pixc instead of a nxm array
	uint32_t *pixc2_0; // start address of the allocated heap memory
	uint32_t *pixc2; // increasing address pointer
}eeprom_t;




// SENSOR DATA
typedef struct
{
	uint16_t data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
	uint8_t RAMoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH];
	uint16_t eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
	uint8_t statusreg;
	uint16_t Ta, ptat_av_uint16, vdd_av_uint16;
}sensor_dat_t;


/*
  ss_dat.RAMoutput is the place where the raw values are saved

  example, order for 80x64:
  ss_dat.RAMoutput[0][]... data from block 0 top
  ss_dat.RAMoutput[1][]... data from block 1 top
  ss_dat.RAMoutput[2][]... data from block 2 top
  ss_dat.RAMoutput[3][]... data from block 3 top
  RAMutput[4][]... electrical offset top
  ss_dat.RAMoutput[5][]... electrical offset bottom
  ss_dat.RAMoutput[6][]... data from block 3 bottom
  ss_dat.RAMoutput[7][]... data from block 2 bottom
  ss_dat.RAMoutput[8][]... data from block 1 bottom
  ss_dat.RAMoutput[9][]... data from block 0 bottom

*/


// BUFFER for PTAT,VDD and elOffsets
typedef struct
{
	// PTAT:
	uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
	uint8_t use_ptat_buffer;
	uint8_t ptat_i;
	// VDD:
	uint16_t vdd_buffer[VDD_BUFFER_SIZE];
	uint8_t use_vdd_buffer;
	uint8_t vdd_i;
	// electrical offsets:
	uint8_t use_eloffsets_buffer;
	uint8_t new_offsets;
}pve_buf_t;



typedef struct
{
	// PROGRAMM CONTROL
	bool switch_ptat_vdd;
	uint8_t adr_offset;
	uint16_t picnum;
	uint8_t state;
	uint8_t read_block_num; // start with electrical offset
	uint8_t read_eloffset_next_pic;
	bool ReadingRoutineEnable;
	// OTHER
	uint32_t gradscale_div;
	uint32_t vddscgrad_div;
	uint32_t vddscoff_div;
	int32_t vddcompgrad_n;
	int32_t vddcompoff_n;
	uint8_t NewDataAvailable;
	uint16_t timert;
}var_ctrl_t;

typedef struct
{
	eeprom_t eep;
	sensor_dat_t ss_dat;
	pve_buf_t pve_buf;
	var_ctrl_t var_ctrl;
	I2C_HandleTypeDef* hi2c;
	TIM_HandleTypeDef* htim;
}htpad_t;

void read_eeprom(htpad_t* hd);
void write_sensor_byte(htpad_t* hd, uint8_t deviceaddress, uint8_t registeraddress, uint8_t input);
void calcPixC(htpad_t* hd);
uint16_t calc_timert(uint8_t clk, uint8_t mbit);
void print_menu(void);
void read_eeprom(htpad_t* hd );
void write_sensor_byte(htpad_t* hd,uint8_t deviceaddress, uint8_t registeraddress, uint8_t input);
void write_calibration_settings_to_sensor(htpad_t* hd);
void readblockinterrupt(htpad_t* hd);
void checkSerial(htpad_t* hd);
void sort_data(htpad_t* hd);
void calculate_pixel_temp(htpad_t* hd);
void print_RAM_array(htpad_t* hd);
void print_calc_steps2(htpad_t* hd);
void print_final_array(htpad_t* hd);
void pixel_masking(htpad_t* hd);
void print_eeprom_hex(htpad_t* hd);
void print_eeprom_header(htpad_t* hd);
void read_sensor_register(htpad_t* hd, uint16_t addr, uint8_t *dest, uint16_t n);
uint8_t read_EEPROM_byte(htpad_t* hd, uint16_t address);

void setup(htpad_t* hd);
void loop(htpad_t* hd);

#endif /* SRC_HTPAD_32X32_H_ */
