add_library(UAV OBJECT
        application/pyro_ui_app.cpp
        booster/fsm/pyro_booster_reset_state.cpp
        booster/booster_config.h
        gimbal/gimbal_config.h
        application/pyro_ui_app.cpp
        WS2812/WS2812.cpp
        WS2812/WS2812.h)

target_sources(UAV
        PRIVATE
        application/pyro_booster_app.cpp
        application/pyro_gimbal_app.cpp
        application/pyro_init_thread.cpp
        application/pyro_mission_planer.cpp
        application/pyro_autoaim_com.cpp

        autoaim/pyro_autoaim_drv.cpp

        booster/fsm/pyro_auto_aim_state.cpp
        booster/fsm/pyro_booster_active_state.cpp
        booster/fsm/pyro_booster_continue_fire_state.cpp
        booster/fsm/pyro_booster_middle_state.cpp
        booster/fsm/pyro_booster_passive_state.cpp
        booster/fsm/pyro_booster_single_fire_state.cpp
        booster/fsm/pyro_booster_stall_state.cpp
        booster/pyro_uav_booster.cpp

        gimbal/pyro_uav_gimbal.cpp
        gimbal/fsm/pyro_active_state.cpp
        gimbal/fsm/pyro_gimbal_auto_state.cpp
        gimbal/fsm/pyro_gimbal_rc_state.cpp
        gimbal/fsm/pyro_passive_state.cpp
        gimbal/feedforward_kf/forecast_v.cpp

        application/pyro_ui_app.cpp
)

target_include_directories(UAV
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/application
        ${CMAKE_CURRENT_SOURCE_DIR}/autoaim
        ${CMAKE_CURRENT_SOURCE_DIR}/booster
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/feedforward_kf
)

target_link_libraries(UAV PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)

target_compile_definitions(PYRo PUBLIC
        DR16_UART=PYRO_UART5
        REFEREE_UART=PYRO_UART1
        AUTOAIM_UART=PYRO_UART10
        VT03_UART=PYRO_UART7
        JCOM_DEBUG_PORT=PYRO_UART10
)
