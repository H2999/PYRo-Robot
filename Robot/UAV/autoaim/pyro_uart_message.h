#ifndef PYRO_STRUCT_H
#define PYRO_STRUCT_H
#include <cstdint>

struct FrameHeader
{
    uint8_t sof;
}__attribute__((packed));

struct FrameTailer
{
    uint16_t crc16;
    uint8_t end;
}__attribute__((packed));

struct RxFrameTailer
{
    uint16_t crc16;
}__attribute__((packed));

struct InputData
{
    float curr_yaw;
    float curr_pitch;
    float curr_roll;
    float curr_speed;
    uint8_t state;
    uint8_t autoaim;
    uint8_t enemy_color;
}__attribute__((packed));

struct OutputData
{
    uint8_t fire;
    float shoot_yaw;
    float shoot_pitch;
    float shoot_dist;
}__attribute__((packed));

struct StateBytes
{
    FrameHeader frame_header;
    InputData input_data;
    FrameTailer frame_tailer;
}__attribute__((packed));

struct OperateBytes
{
    FrameHeader frame_header;
    OutputData output_data;
    RxFrameTailer frame_tailer;
}__attribute__((packed));

#endif // PYRO_STRUCT_H

