
#ifndef BINARIZATION_H
#define BINARIZATION_H

#include "zf_common_headfile.h"

uint8 calculate_otsu_threshold(const uint8* image_data, uint32 size);

uint8 calculate_otsu_threshold_with_mask(const uint8* image_data, const uint8* mask, uint32 size);
uint8 advanced_otsu_threshold_with_mask(const uint8* source_image_data, const uint8* mask, uint32 size);
void apply_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size, uint8 threshold);

void apply_binarization_with_mask(const uint8* source_image_data, uint8* dest_image_data, const uint8* mask, uint32 size, uint8 threshold);

uint8 image_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size);

uint8 image_binarization_with_mask(const uint8* source_image_data, uint8* dest_image_data, const uint8* mask, uint32 size);

#endif // BINARIZATION_H