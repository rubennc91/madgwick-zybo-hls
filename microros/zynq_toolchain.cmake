# Toolchain de micro-ROS para el Cortex-A9 del Zynq-7000 (arm-none-eabi, FreeRTOS).
# Se copia a micro_ros_stm32cubemx_utils/microros_static_library/library_generation/toolchain.cmake
# (ver microros/README.md). Las tres lineas de FLAGS son las que se usaron en la build medida;
# las lineas del compilador son las habituales del toolchain original: si el tuyo difiere, conserva el original y cambia solo FLAGS.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

# -mfloat-abi=hard debe coincidir con la configuracion de la aplicacion en Vitis (ARM v7 gcc compiler).
# -frtti: tracetools usa typeid y falla con -fno-rtti.
set(FLAGS "-O2 -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -ffunction-sections -fdata-sections -fno-exceptions -DCLOCK_MONOTONIC=1 -frtti" CACHE STRING "")
set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS}" CACHE STRING "")
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS}" CACHE STRING "")

set(__BIG_ENDIAN__ 0)
