add_library(UAV OBJECT)

target_sources(UAV PRIVATE

        ${CMAKE_CURRENT_SOURCE_DIR}/booster/pyro_uav_booster.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_auto_aim_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_booster_active_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_booster_continue_fire_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_booster_middle_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_booster_passive_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_booster_single_fire_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_booster_stall_state.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/pyro_uav_gimbal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_active_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_gimbal_auto_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_gimbal_rc_state.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_passive_state.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_gimbal_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_booster_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_pc_communicate.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_init_thread.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_mission_planer.cpp

)
target_include_directories(UAV PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/booster
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}/application
)
target_link_libraries(UAV PUBLIC stm32cubemx dsppp_lib PYRo)
