#include "pyro_autoaim_drv.h"
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"
#include "pyro_crc.h"
#include <cstring>

#include "pyro_dwt_drv.h"
float now = 0;
float last = 0;
float delta;

namespace pyro
{
status_t autoaim_drv_t::autoaim_task_t::init()
{
    if (_owner)
    {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void autoaim_drv_t::autoaim_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

#ifdef AUTOAIM_UART
//获取实例
autoaim_drv_t &autoaim_drv_t::get_instance()
{
    static autoaim_drv_t instance(&AUTOAIM_UART);
    return instance;
}
#endif

autoaim_drv_t::autoaim_drv_t(uart_drv_t *uart_handle)
    : _uart_drv(uart_handle), _task(nullptr), _tx_buffer(nullptr),
      _rx_msg_buf(nullptr), _is_online(false)
{
    memset(&_latest_target, 0, sizeof(_latest_target));
    memset(&_tx_payload, 0, sizeof(_tx_payload));

    constexpr size_t buf_size = sizeof(tx_packet_t);
    _tx_buffer = static_cast<tx_packet_t *>(pvPortDmaMalloc(buf_size));

    if (_tx_buffer)
    {
        memset(_tx_buffer, 0, buf_size);
    }

    _task = new autoaim_task_t(this);
}

autoaim_drv_t::~autoaim_drv_t()
{
    if (_task)
    {
        _task->stop();
        delete _task;
        _task = nullptr;
    }

    if (_uart_drv)
    {
        _uart_drv->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    }

    if (_tx_buffer)
    {
        vPortFree(_tx_buffer);
        _tx_buffer = nullptr;
    }

    if (_rx_msg_buf)
    {
        vMessageBufferDelete(_rx_msg_buf);
        _rx_msg_buf = nullptr;
    }
}

/* Public Control Methods ----------------------------------------------------*/
void autoaim_drv_t::start_rx() const
{
    if (_task && _uart_drv && _tx_buffer)
    {
        _task->start();
    }
}

/* Logic Implementation (Private) --------------------------------------------*/

void autoaim_drv_t::init_impl()
{
    if (_rx_msg_buf == nullptr)
    {
        _rx_msg_buf = xMessageBufferCreate(128);
    }

    if (_rx_msg_buf == nullptr)
        return;

    _uart_drv->add_rx_event_callback(
        [this](const uint8_t *p, const uint16_t size,
               BaseType_t &task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
}

void autoaim_drv_t::run_loop_impl()
{
    while (true)
    {
        static rx_packet_t pkt;
        static size_t xReceivedBytes;

        if (xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt),
                                  portMAX_DELAY) == sizeof(rx_packet_t))
        {
            _is_online = true;
        }

        while (_is_online)
        {
            xReceivedBytes =
                xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt), pdMS_TO_TICKS(50));

            if (xReceivedBytes == sizeof(rx_packet_t))
            {
                if (error_check(&pkt) == PYRO_OK)
                {
                    unpack(&pkt);
                }
            }
            else if (xReceivedBytes == 0)
            {
                _is_online = false;
            }
        }
    }
}

bool autoaim_drv_t::rx_callback(const uint8_t *p_data, const uint16_t size,
                                 BaseType_t &xHigherPriorityTaskWoken) const
{
    // RX Packet 不含 \n，严格匹配长度
    if (size == sizeof(rx_packet_t) && p_data[0] == FRAME_SOF)
    {
        xMessageBufferSendFromISR(_rx_msg_buf, p_data, sizeof(rx_packet_t),
                                  &xHigherPriorityTaskWoken);
        return true;
    }
    return false;
}



status_t autoaim_drv_t::error_check(const rx_packet_t *buf)
{

    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const *>(buf),
                                sizeof(rx_packet_t)))
    {
        return PYRO_ERROR;
    }
    //通信丢包测试
    // last = now;
    //
    // now = dwt_drv_t::get_timeline_ms();
    //
    // delta = now - last;
    return PYRO_OK;
}

void autoaim_drv_t::unpack(const rx_packet_t *buf)
{
    memcpy(&_latest_target, &buf->data, sizeof(rx_data_t));
}

/* Transmission --------------------------------------------------------------*/
autoaim_drv_t::tx_data_t &autoaim_drv_t::get_tx_data()
{
    return _tx_payload;
}

status_t autoaim_drv_t::send_data() const
{
    if (!_tx_buffer || !_uart_drv)
        return PYRO_ERROR;

    // 1. 填充帧头
    _tx_buffer->header.sof = FRAME_SOF;

    // 2. 拷贝业务数据
    memcpy(&_tx_buffer->data, &_tx_payload, sizeof(tx_data_t));

    // 3. 尾部填充 \n (必须放在 CRC 之后的位置，但要在计算 CRC 之前赋值以保逻辑统一)
    _tx_buffer->tailer.end = '\n';

    // 4. 计算 CRC16，排除了最后的 \n 长度
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_buffer),
                           sizeof(tx_packet_t) - 1);

    // 5. 触发 DMA 发送
    return _uart_drv->write(reinterpret_cast<uint8_t *>(_tx_buffer),
                            sizeof(tx_packet_t));
}

/* Getters -------------------------------------------------------------------*/
const autoaim_drv_t::rx_data_t &autoaim_drv_t::get_target_data() const
{
    return _latest_target;
}

bool autoaim_drv_t::check_online() const
{
    return _is_online;
}

} // namespace pyro