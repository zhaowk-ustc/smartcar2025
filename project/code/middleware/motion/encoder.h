#ifndef ENCODER_H
#define ENCODER_H

#include "zf_common_headfile.h"
#include "../module_base.h"

class Encoder : public Module
{
public:
    enum class Direction
    {
        CCW,
        CW
    };

    struct encoder_config
    {
        encoder_index_enum encoder;
        encoder_channel1_enum count_pin;
        encoder_channel2_enum dir_pin;
        Direction direction;
    };

    Encoder() = delete;
    Encoder(const encoder_config& config)
        : encoder_(config.encoder),
        count_pin_(config.count_pin),
        dir_pin_(config.dir_pin),
        direction_(config.direction),
        output_count_(nullptr)
    {
    }

    void init() override
    {
        encoder_dir_init(encoder_, count_pin_, dir_pin_);
        encoder_clear_count(encoder_);
    }

    void update() override
    {
        int16 raw_count = encoder_get_count(encoder_);
        *output_count_ = (direction_ == Direction::CW) ? -raw_count : raw_count;
        encoder_clear_count(encoder_);
    }

    void reset() override
    {
        encoder_clear_count(encoder_);
        *output_count_ = 0;
    }

    // 指针连出接口
    void connect_outputs(int16* count_ptr)
    {
        output_count_ = count_ptr;
    }

private:
    const encoder_index_enum encoder_;
    const encoder_channel1_enum count_pin_;
    const encoder_channel2_enum dir_pin_;
    const Direction direction_;

    int16* output_count_;
};
#endif // ENCODER_H