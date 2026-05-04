#ifndef __PYRO_AUTOAIM_DRV_H__
#define __PYRO_AUTOAIM_DRV_H__

#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_def.h"
#include "message_buffer.h"

namespace pyro
{

class autoaim_drv_t
{
public:
#pragma pack(push, 1)

//发送给自瞄的数据
struct tx_data_t
{
    float curr_yaw;
    float curr_pitch;
    float curr_roll;
    float curr_speed;
    uint8_t shoot_delay;
    uint8_t state;
    uint8_t autoaim;
    uint8_t enemy_color;
};

//自瞄回传的数据
struct rx_data_t
{
    uint8_t fire;
    float shoot_yaw;
    float shoot_pitch;
    float shoot_dist;
};
#pragma pack(pop)

#ifdef AUTOAIM_UART
    static autoaim_drv_t &get_instance();
#endif

    void start_rx() const;

    tx_data_t &get_tx_data();

    status_t send_data() const;

    /**
     * @brief Gets the latest target data from PC.
     */
    [[nodiscard]] const rx_data_t &get_target_data() const;

    /**
     * @brief Checks connection status with PC.
     */
    [[nodiscard]] bool check_online() const;

  private:

    explicit autoaim_drv_t(uart_drv_t *uart_handle);

    ~autoaim_drv_t();

    class autoaim_task_t final : public task_base_t
    {
      public:
        explicit autoaim_task_t(autoaim_drv_t *owner_ptr)
            : task_base_t("autoaim_task",256 , 256, priority_t::NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        autoaim_drv_t *_owner;
    };
#pragma pack(push, 1)

    struct frame_header_t
    {
        uint8_t sof;
    };

    // 发送给 PC 的尾部带有 \n
    struct tx_tailer_t
    {
        uint16_t crc16;
        uint8_t end;
    };

    // 接收 PC 的尾部不带 \n
    struct rx_tailer_t
    {
        uint16_t crc16;
    };

    struct tx_packet_t
    {
        frame_header_t header;
        tx_data_t data;
        tx_tailer_t tailer;
    };

    struct rx_packet_t
    {
        frame_header_t header;
        rx_data_t data;
        rx_tailer_t tailer;
    };

#pragma pack(pop)

    uart_drv_t *_uart_drv;
    autoaim_task_t *_task;   // The internal task instance
    tx_packet_t *_tx_buffer; // DMA buffer
    MessageBufferHandle_t _rx_msg_buf;

    tx_data_t _tx_payload{}; // 缓存用户修改的待发数据
    rx_data_t _latest_target{};
    bool _is_online;

    static constexpr uint8_t FRAME_SOF = 0xA5;

    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size,
                     BaseType_t& xHigherPriorityTaskWoken) const;

    static status_t error_check(const rx_packet_t *buf);
    void unpack(const rx_packet_t *buf);
};

} // namespace pyro

#endif // __PYRO_AUTOAIM_DRV_H__