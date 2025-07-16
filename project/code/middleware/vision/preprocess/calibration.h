#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

constexpr uint16_t calibrated_width = 88;
constexpr uint16_t calibrated_height = 42;
constexpr uint16_t calibrated_size = calibrated_width * calibrated_height;
constexpr uint16_t aligned_calibrated_size = 88 * 42;

void image_calibration(uint8_t* dest_image_data, const uint8_t* source_image_data);

extern const uint8_t calibration_mask[4224];

#endif