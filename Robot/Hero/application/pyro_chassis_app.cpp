#include "pyro_module_base.h"
#include "pyro_mec_chassis.h"
#include "pyro_mutex.h"
#include "pyro_com_canrx.h"
#include "pyro_com_cantx.h"
#include "pyro_referee.h"


static pyro::mec_chassis_t *mec_chassis_ptr               = nullptr;
static pyro::mec_cmd_t *mec_cmd_ptr                       = nullptr;
// static pyro::dr16_drv_t::dr16_ctrl_t const *dr16_ctrl_ptr = nullptr;
static void chassis_rxcmd();
static void shoot_tx();


extern "C"
{

    void hero_chassis_thread(void *argument)
    {
        while (true)
        {
            shoot_tx();
            chassis_rxcmd();
            mec_chassis_ptr->set_command(*mec_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can2, 0x101);
        mec_cmd_ptr     = new pyro::mec_cmd_t();
        mec_chassis_ptr = pyro::mec_chassis_t::instance();
        // dr16_ctrl_ptr   = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
        //     pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        mec_chassis_ptr->start();
        xTaskCreate(hero_chassis_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}


void shoot_tx()
{
    static float last_speed = 0.0f;
    float current_speed =
        pyro::referee_drv_t::get_instance()->get_data().shoot.initial_speed;
    pyro::can_tx_drv_t::clear(0x103);
    uint16_t current_heat  = pyro::referee_drv_t::get_instance()->get_data().power_heat.shooter_42mm_barrel_heat;
    uint16_t heat_limit = pyro::referee_drv_t::get_instance()->get_data().robot_status.shooter_barrel_heat_limit;
    if (last_speed != current_speed)
    {
        pyro::can_tx_drv_t::clear(0x102);
        pyro::can_tx_drv_t::add_data(0x102, 32, current_speed);
        pyro::can_tx_drv_t::send(0x102,pyro::can_hub_t::get_instance()->hub_get_can_obj(pyro::can_hub_t::can2));
        last_speed = current_speed;
    }

    pyro::can_tx_drv_t::add_data(0x103, 16, current_heat);
    pyro::can_tx_drv_t::add_data(0x103, 16, heat_limit);
    pyro::can_tx_drv_t::send(
    0x103, pyro::can_hub_t::get_instance()->hub_get_can_obj(
               pyro::can_hub_t::which_can::can2));
}


void chassis_rxcmd()
{
    std::array<uint8_t, 8> raw_data{};
    pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can2, 0x101,
                                 raw_data);
    mec_cmd_ptr->vx =
        4.0f * static_cast<float>(static_cast<int8_t>(raw_data[0])) / 127.0f;
    if (abs(mec_cmd_ptr->vx) < 0.1f)
    {
        mec_cmd_ptr->vx = 0;
    }
    mec_cmd_ptr->vy =
        2.0f * static_cast<float>(static_cast<int8_t>(raw_data[1])) / 127.0f;
    if (abs(mec_cmd_ptr->vy) < 0.1f)
    {
        mec_cmd_ptr->vy = 0;
    }
    mec_cmd_ptr->wz =
        5.0f * static_cast<float>(static_cast<int8_t>(raw_data[2])) / 127.0f;
    if (abs(mec_cmd_ptr->wz) < 0.1f)
    {
        mec_cmd_ptr->wz = 0;
    }
    if (abs(mec_cmd_ptr->vx) + abs(mec_cmd_ptr->vy) > 1.0f)
    {
        mec_cmd_ptr->wz = 0.5f * mec_cmd_ptr->wz;
    }
    mec_cmd_ptr->mode = static_cast<pyro::cmd_base_t::mode_t>(raw_data[3]);
}
