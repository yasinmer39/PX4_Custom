## This cmakelist.txt file was generated from
## the UAV Toolbox Support Package for PX4 Autopilots

set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")

add_definitions(
-DMODEL=connectedIO -DNUMST=1 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0 -DPX4 -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT -DNULL=0 -DPX4_CONNECTEDIO -DHOST_TARGET -DMW_PX4_POSIX_BUILD )

px4_add_module(
    MODULE modules__px4_simulink_app
    MAIN px4_simulink_app
    STACK_MAIN 2000
SRCS
   MW_uORB_Read.cpp
   MW_uORB_Write.cpp
   MW_PX4_TaskControl.cpp
   nuttxinitialize.cpp
   MW_PX4_Parameter.cpp
   MW_PX4_PWM.cpp
   MW_PX4_ActuatorDevice.cpp
   MW_PX4_Actuators.cpp
   customFunction.cpp
   IO_main.cpp
   uORBMetaData.c
   IO_checksum.c
   IO_debug.c
   IO_packet.c
   IO_server.c
   IO_standardperipherals.c
   IO_utilities.c
   PeripheralToHandle.c
   MW_PX4_rtiostream_tcpip.cpp
    COMPILE_FLAGS
       -fpermissive
       -Wno-narrowing
       -Wno-address-of-packed-member 
       -Wno-cast-align
    INCLUDES
       ${PX4_SOURCE_DIR}/src/modules/mavlink
       ${MAVLINK_LIBRARY_DIR}
       ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
       ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
       /home/user/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/px4/core/include
       /home/user/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/px4/core/ConnectedIO/include
       /usr/local/MATLAB/R2025b/toolbox/target/shared/svd/common/include
       /usr/local/MATLAB/R2025b/toolbox/target/shared/ioserver/ioserver/inc
       /tmp/px4_connectedIO/PX4HostTarget
)

get_target_property(PX4_SL_APP_COMPILE_FLAGS modules__px4_simulink_app COMPILE_OPTIONS)
list(REMOVE_ITEM PX4_SL_APP_COMPILE_FLAGS -Werror)
set_target_properties(modules__px4_simulink_app PROPERTIES COMPILE_OPTIONS "${PX4_SL_APP_COMPILE_FLAGS}")

