add_library(Hero OBJECT)

target_sources(Hero PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}/booster/pyro_quad_booster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_passive_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_active_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_homing_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_ready_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_stall_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_interim_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_quad_busy_state.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/pyro_direct_gimbal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_direct_active_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_direct_passive_state.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/pyro_mec_chassis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/fsm/pyro_mec_passive_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/fsm/pyro_mec_active_state.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_chassis_app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_gimbal_app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_booster_app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_ui_app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_pc_com.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_init_thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_mission_planer.cpp

)
target_include_directories(Hero PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/booster
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/application
)
target_link_libraries(Hero PUBLIC stm32cubemx dsppp_lib PYRo)
