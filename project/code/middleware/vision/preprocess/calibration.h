#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

constexpr uint16_t calibrated_width = 70;
constexpr uint16_t calibrated_height = 53;
constexpr uint16_t calibrated_size = 70 * 53;
constexpr uint16_t aligned_calibrated_size = 70 * 54;

void image_calibration(uint8_t* dest_image_data, const uint8_t* source_image_data);

extern const uint8_t calibration_mask[3710];

#endif