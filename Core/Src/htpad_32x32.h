/*
 * htpad_32x32.h
 *
 *  Created on: Oct 12, 2025
 *      Author: Loc
 */

#ifndef SRC_HTPAD_32X32_H_
#define SRC_HTPAD_32X32_H_

#define RXBUFFERSIZE              2

void read_eeprom(void);
void write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input);
void write_calibration_settings_to_sensor(void);
void calcPixC(void);
uint16_t calc_timert(uint8_t clk, uint8_t mbit);
void print_menu(void);
void read_eeprom(void);
void write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input);
void write_calibration_settings_to_sensor(void);
void readblockinterrupt(void);
void checkSerial(void);
void sort_data(void);
void calculate_pixel_temp(void);
void print_RAM_array(void);
void print_calc_steps2(void);
void print_final_array(void);
void pixel_masking(void);
void print_eeprom_hex(void);
void print_eeprom_header(void);
void read_sensor_register(uint16_t addr, uint8_t *dest, uint16_t n);
uint8_t read_EEPROM_byte(uint16_t address);

void setup(void);
void loop(void);

#endif /* SRC_HTPAD_32X32_H_ */
