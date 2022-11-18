/*
 * common.h
 *
 *  Created on: Sep 15, 2022
 *      Author: Gintaras
 */

#ifndef COMMON_H_
#define COMMON_H_

typedef struct app_sensor_data
{
	float dps_temperature;
	float dps_pressure;

	double bmp_temperature;
	double bmp_pressure;

	int16_t bmi_acc_x;
	int16_t bmi_acc_y;
	int16_t bmi_acc_z;

	int16_t bmi_gyr_x;
	int16_t bmi_gyr_y;
	int16_t bmi_gyr_z;

	float bme_temperature;
	float bme_pressure;
	float bme_humidity;
	float bme_gas_resistance;
	uint8_t bme_gas_index;

	uint16_t sgp_sraw_voc;
	int32_t sgp_voc_index;

	int32_t sht_temperature;
	int32_t sht_humidity;

	uint8_t battery_lvl;

}sensor_data_t;



#endif /* COMMON_H_ */
