cmake_minimum_required(VERSION 3.22.1)

###############################
# Toolchain
include("../toolchain.cmake")

###############################
# Project
project(OnekeyProBTFW_APP LANGUAGES C CXX ASM)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

message(STATUS "${CMAKE_PROJECT_NAME} Build Type: ${CMAKE_BUILD_TYPE}")

###############################
# NRF SDK
set(NRF_SDK_ROOT "${PROJECT_SOURCE_DIR}/../ble-firmware")

set(
  NRF_SDK_INC
  ${NRF_SDK_ROOT}/components
  ${NRF_SDK_ROOT}/components/ble/ble_advertising
  ${NRF_SDK_ROOT}/components/ble/ble_dtm
  ${NRF_SDK_ROOT}/components/ble/ble_link_ctx_manager
  ${NRF_SDK_ROOT}/components/ble/ble_racp
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ancs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ans_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_bas
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_bas_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_cscs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_cts_c
  # ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dis
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_gls
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hids
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hrs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hrs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hts
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ias
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ias_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lbs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lbs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lls
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_nus
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_nus_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_rscs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_rscs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_tps
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_fido
  ${NRF_SDK_ROOT}/components/ble/common
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gatt
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gq
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_qwr
  ${NRF_SDK_ROOT}/components/ble/peer_manager
  # ${NRF_SDK_ROOT}/components/boards
  ${NRF_SDK_ROOT}/components/libraries/atomic
  ${NRF_SDK_ROOT}/components/libraries/atomic_fifo
  ${NRF_SDK_ROOT}/components/libraries/atomic_flags
  ${NRF_SDK_ROOT}/components/libraries/balloc
  ${NRF_SDK_ROOT}/components/libraries/bootloader
  # ${NRF_SDK_ROOT}/components/libraries/bootloader/ble_dfu
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu
  # ${NRF_SDK_ROOT}/components/libraries/bsp
  # ${NRF_SDK_ROOT}/components/libraries/button
  # ${NRF_SDK_ROOT}/components/libraries/cli
  ${NRF_SDK_ROOT}/components/libraries/crc16
  ${NRF_SDK_ROOT}/components/libraries/crc32
  ${NRF_SDK_ROOT}/components/libraries/crypto
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/cc310
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/cc310_bl
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/cifra
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/mbedtls
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_sw
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/optiga
  # ${NRF_SDK_ROOT}/components/libraries/csense
  # ${NRF_SDK_ROOT}/components/libraries/csense_drv
  ${NRF_SDK_ROOT}/components/libraries/delay
  ${NRF_SDK_ROOT}/components/libraries/ecc
  ${NRF_SDK_ROOT}/components/libraries/experimental_section_vars
  # ${NRF_SDK_ROOT}/components/libraries/experimental_task_manager
  ${NRF_SDK_ROOT}/components/libraries/fds
  ${NRF_SDK_ROOT}/components/libraries/fifo
  ${NRF_SDK_ROOT}/components/libraries/fstorage
  # ${NRF_SDK_ROOT}/components/libraries/gfx
  ${NRF_SDK_ROOT}/components/libraries/gpiote
  ${NRF_SDK_ROOT}/components/libraries/hardfault
  ${NRF_SDK_ROOT}/components/libraries/hci
  # ${NRF_SDK_ROOT}/components/libraries/led_softblink
  ${NRF_SDK_ROOT}/components/libraries/log
  ${NRF_SDK_ROOT}/components/libraries/log/src
  # ${NRF_SDK_ROOT}/components/libraries/low_power_pwm
  ${NRF_SDK_ROOT}/components/libraries/mem_manager
  ${NRF_SDK_ROOT}/components/libraries/memobj
  ${NRF_SDK_ROOT}/components/libraries/mpu
  ${NRF_SDK_ROOT}/components/libraries/mutex
  ${NRF_SDK_ROOT}/components/libraries/pwm
  ${NRF_SDK_ROOT}/components/libraries/pwr_mgmt
  ${NRF_SDK_ROOT}/components/libraries/queue
  ${NRF_SDK_ROOT}/components/libraries/ringbuf
  ${NRF_SDK_ROOT}/components/libraries/scheduler
  # ${NRF_SDK_ROOT}/components/libraries/sdcard
  # ${NRF_SDK_ROOT}/components/libraries/sensorsim
  ${NRF_SDK_ROOT}/components/libraries/slip
  ${NRF_SDK_ROOT}/components/libraries/sortlist
  ${NRF_SDK_ROOT}/components/libraries/spi_mngr
  ${NRF_SDK_ROOT}/components/libraries/stack_guard
  ${NRF_SDK_ROOT}/components/libraries/stack_info
  ${NRF_SDK_ROOT}/components/libraries/strerror
  ${NRF_SDK_ROOT}/components/libraries/svc
  ${NRF_SDK_ROOT}/components/libraries/timer
  ${NRF_SDK_ROOT}/components/libraries/twi_mngr
  # ${NRF_SDK_ROOT}/components/libraries/twi_sensor
  ${NRF_SDK_ROOT}/components/libraries/uart
  # ${NRF_SDK_ROOT}/components/libraries/usbd
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/audio
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/cdc
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/cdc/acm
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid/generic
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid/kbd
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid/mouse
  # ${NRF_SDK_ROOT}/components/libraries/usbd/class/msc
  ${NRF_SDK_ROOT}/components/libraries/util
  ${NRF_SDK_ROOT}/components/softdevice/common
  ${NRF_SDK_ROOT}/components/softdevice/s132/headers
  ${NRF_SDK_ROOT}/components/softdevice/s132/headers/nrf52
  ${NRF_SDK_ROOT}/components/toolchain/cmsis/include
  ${NRF_SDK_ROOT}/external/fprintf
  ${NRF_SDK_ROOT}/external/mbedtls/include
  ${NRF_SDK_ROOT}/external/micro-ecc/micro-ecc
  # ${NRF_SDK_ROOT}/external/nrf_cc310/include
  ${NRF_SDK_ROOT}/external/nrf_oberon
  ${NRF_SDK_ROOT}/external/nrf_oberon/include
  ${NRF_SDK_ROOT}/external/nrf_tls/mbedtls/nrf_crypto/config
  ${NRF_SDK_ROOT}/external/segger_rtt
  ${NRF_SDK_ROOT}/external/utf_converter
  ${NRF_SDK_ROOT}/integration/nrfx
  ${NRF_SDK_ROOT}/integration/nrfx/legacy
  ${NRF_SDK_ROOT}/modules/nrfx
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/include
  ${NRF_SDK_ROOT}/modules/nrfx/hal
  ${NRF_SDK_ROOT}/modules/nrfx/mdk
)

set(
  NRF_SDK_SRC
  ${NRF_SDK_ROOT}/components/ble/ble_advertising/ble_advertising.c
  ${NRF_SDK_ROOT}/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c
  ${NRF_SDK_ROOT}/components/ble/ble_racp/ble_racp.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_bas/ble_bas.c
  # ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu.c
  # ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu_bonded.c
  # ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu_unbonded.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dis/ble_dis.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lbs/ble_lbs.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_nus/ble_nus.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_fido/ble_fido.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_advdata.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_conn_params.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_conn_state.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_srv_common.c
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gatt/nrf_ble_gatt.c
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gq/nrf_ble_gq.c
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_qwr/nrf_ble_qwr.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/auth_status_tracker.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/gatt_cache_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/gatts_cache_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/id_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/nrf_ble_lesc.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_data_storage.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_database.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_id.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_manager_handler.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/pm_buffer.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/security_dispatcher.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/security_manager.c
  ${NRF_SDK_ROOT}/components/libraries/atomic/nrf_atomic.c
  ${NRF_SDK_ROOT}/components/libraries/atomic_fifo/nrf_atfifo.c
  ${NRF_SDK_ROOT}/components/libraries/atomic_flags/nrf_atflags.c
  ${NRF_SDK_ROOT}/components/libraries/balloc/nrf_balloc.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_svci.c
  ${NRF_SDK_ROOT}/components/libraries/crc16/crc16.c
  ${NRF_SDK_ROOT}/components/libraries/crc32/crc32.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_init.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng_mbedtls.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_chacha_poly_aead.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_eddsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hmac.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_aead.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_aes.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_aes_shared.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_eddsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_error.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hkdf.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hmac.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_init.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_rng.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_shared.c
  ${NRF_SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c
  ${NRF_SDK_ROOT}/components/libraries/fds/fds.c
  ${NRF_SDK_ROOT}/components/libraries/fifo/app_fifo.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage_sd.c
  ${NRF_SDK_ROOT}/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c
  ${NRF_SDK_ROOT}/components/libraries/hardfault/hardfault_implementation.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_backend_rtt.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_backend_serial.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_default_backends.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_str_formatter.c
  ${NRF_SDK_ROOT}/components/libraries/mem_manager/mem_manager.c
  ${NRF_SDK_ROOT}/components/libraries/memobj/nrf_memobj.c
  ${NRF_SDK_ROOT}/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c
  ${NRF_SDK_ROOT}/components/libraries/queue/nrf_queue.c
  ${NRF_SDK_ROOT}/components/libraries/ringbuf/nrf_ringbuf.c
  ${NRF_SDK_ROOT}/components/libraries/scheduler/app_scheduler.c
  ${NRF_SDK_ROOT}/components/libraries/sortlist/nrf_sortlist.c
  ${NRF_SDK_ROOT}/components/libraries/strerror/nrf_strerror.c
  ${NRF_SDK_ROOT}/components/libraries/timer/app_timer2.c
  ${NRF_SDK_ROOT}/components/libraries/timer/drv_rtc.c
  ${NRF_SDK_ROOT}/components/libraries/uart/app_uart_fifo.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error_weak.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_util_platform.c
  ${NRF_SDK_ROOT}/components/libraries/util/nrf_assert.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c
  ${NRF_SDK_ROOT}/external/fprintf/nrf_fprintf.c
  ${NRF_SDK_ROOT}/external/fprintf/nrf_fprintf_format.c
  ${NRF_SDK_ROOT}/external/mbedtls/library/aes.c
  ${NRF_SDK_ROOT}/external/mbedtls/library/ctr_drbg.c
  ${NRF_SDK_ROOT}/external/segger_rtt/SEGGER_RTT.c
  ${NRF_SDK_ROOT}/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c
  ${NRF_SDK_ROOT}/external/segger_rtt/SEGGER_RTT_printf.c
  ${NRF_SDK_ROOT}/external/utf_converter/utf.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_rng.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_uart.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_clock.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_rng.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_rtc.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_spim.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_timer.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_twi.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uarte.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_wdt.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_nvmc.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/prs/nrfx_prs.c
  ${NRF_SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52.S
  ${NRF_SDK_ROOT}/modules/nrfx/mdk/system_nrf52.c
  ${NRF_SDK_ROOT}/modules/nrfx/soc/nrfx_atomic.c
)

set(NRF_SDK_LINK
  ${NRF_SDK_ROOT}/external/nrf_oberon/lib/cortex-m4/hard-float/liboberon_3.0.1.a
)

###############################
# micro-ecc

set(MicroECC_DIR ${NRF_SDK_ROOT}/external/micro-ecc)

add_library(
  MicroECC STATIC
  ${MicroECC_DIR}/micro-ecc/uECC.c
)

target_include_directories(
  MicroECC PUBLIC
  ${MicroECC_DIR}/micro-ecc
  ${NRF_SDK_ROOT}/components/toolchain/cmsis/include
)

target_compile_options(
  MicroECC PRIVATE
  # Optimization flags
  -Os -g0 -s
  # Uncomment the line below to enable link time optimization
  # -flto=auto
  # C flags common to all targets
  -DFLOAT_ABI_HARD
  -DuECC_ENABLE_VLI_API=0
  -DuECC_OPTIMIZATION_LEVEL=3
  -DuECC_SQUARE_FUNC=0
  -DuECC_SUPPORT_COMPRESSED_POINT=0
  -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1
  -Wall -Werror -Wno-unused-function
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # keep every function in a separate section, this allows linker to discard unused ones
  -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums 
  # heap and stack (why set them for a library?)
  # -D__HEAP_SIZE=4096
  # -D__STACK_SIZE=4096
)

target_link_options(
  MicroECC PRIVATE
  # Linker flags
  -Os -g0 -s
  # let linker dump unused sections
  -Wl,--gc-sections
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # use newlib in nano version
  --specs=nano.specs -lc -lnosys -lm
  -L${NRF_SDK_ROOT}/modules/nrfx/mdk -T${MicroECC_DIR}/nrf52hf_armgcc/armgcc/ext_micro_ecc_gcc_nRF5x.ld
)

###############################
# Target

add_executable(
  ${CMAKE_PROJECT_NAME} 
  main.c
)

target_sources(
  ${CMAKE_PROJECT_NAME} PRIVATE 
  ${NRF_SDK_SRC}
  ../drivers/nrf_i2c.c
  ../drivers/nrf_uicr.c
  ../drivers/nrf_flash.c
  ../drivers/pmu/pmu.c
  ../drivers/pmu/ntc_util.c
  ../drivers/pmu/axp216.c
  ../drivers/pmu/axp2101.c
  ../drivers/light/lm36011.c
  data_transmission.c
  ecdsa.c
  power_manage.c
  flashled_manage.c
  device_config.c
  dfu_upgrade.c
)

target_include_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${NRF_SDK_INC}
  ../artifacts # this is only for the dfu_upgrade_payload.h
  ../drivers
  ../drivers/pmu
  ../drivers/light
)

execute_process(
	COMMAND	git rev-parse HEAD
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_VERSION
)
add_definitions('-DBUILD_ID=\"${GIT_VERSION}\"')

target_compile_definitions(
  ${CMAKE_PROJECT_NAME} PRIVATE
  # debug handling
  $<$<CONFIG:Debug>:
  DEBUG
  >
  # project flags
  APP_TIMER_V2
  APP_TIMER_V2_RTC1_ENABLED
  CONFIG_GPIO_AS_PINRESET
  FLOAT_ABI_HARD
  NRF52
  NRF52832_XXAA
  NRF_SD_BLE_API_VERSION=7
  S132
  SOFTDEVICE_PRESENT
  # BUTTONLESS_ENABLED
  BOND_ENABLE
  # NRF_DFU_TRANSPORT_BLE
  MBEDTLS_CONFIG_FILE="nrf_crypto_mbedtls_config.h"
  NRF_APP_VERSION=0x00000001
  NRF_APP_VERSION_ADDR=0x1D000
  NRF_CRYPTO_MAX_INSTANCE_COUNT=1
  # heap and stack
  __HEAP_SIZE=8192
  __STACK_SIZE=8192
)

target_compile_options(
  ${CMAKE_PROJECT_NAME} PRIVATE

  # Build type handling
  $<$<CONFIG:Debug>:
  -Og -ggdb
  >
  $<$<CONFIG:Release>:
  -Os -g0
  >
  
  # Uncomment the line below to enable link time optimization
  # -flto=auto
  #  -Wall -Werror
  -Wall -Wno-comment -Wno-array-bounds
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # keep every function in a separate section, this allows linker to discard unused ones
  -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums
)

target_link_options(
  ${CMAKE_PROJECT_NAME} PRIVATE
  # Build type handling
  $<$<CONFIG:Debug>:
  -Og -ggdb
  -DDEBUG
  >
  $<$<CONFIG:Release>:
  -Os -g0
  >
  # Linker flags
  # let linker dump unused sections
  -Wl,--gc-sections
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # use newlib in nano version
  --specs=nano.specs -lc -lnosys -lm
  -L${NRF_SDK_ROOT}/modules/nrfx/mdk -T${PROJECT_SOURCE_DIR}/ble_app_gcc_nrf52.ld
)

target_link_libraries(
  ${CMAKE_PROJECT_NAME} PRIVATE
  ${NRF_SDK_LINK}
  MicroECC
  m # math
)

###############################
# bin and hex
add_custom_target(
  ${CMAKE_PROJECT_NAME}_bin ALL
  COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.bin
  DEPENDS ${CMAKE_PROJECT_NAME}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
)

add_custom_target(
  ${CMAKE_PROJECT_NAME}_hex ALL
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.hex
  DEPENDS ${CMAKE_PROJECT_NAME}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
)

###############################
# development related

# sdk config
add_custom_target(
  "sdk_config"
  COMMAND java -jar ${NRF_SDK_ROOT}/external_tools/CMSIS_Configuration_Wizard.jar sdk_config.h
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

###############################
# Install
install(
  FILES 
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
  # DESTINATION ${PROJECT_SOURCE_DIR}/../artifacts
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)