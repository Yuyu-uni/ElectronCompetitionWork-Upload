################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/tools" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/task" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Hardwares" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Debug" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1389864817: ../empty.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs2020/ccs/utils/sysconfig_1.24.0/sysconfig_cli.bat" --script "D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/empty.syscfg" -o "syscfg" -s "D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/device_linker.cmd: build-1389864817 ../empty.syscfg
syscfg/device.opt: build-1389864817
syscfg/device.cmd.genlibs: build-1389864817
syscfg/ti_msp_dl_config.c: build-1389864817
syscfg/ti_msp_dl_config.h: build-1389864817
syscfg/Event.dot: build-1389864817
syscfg: build-1389864817

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/tools" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/task" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Hardwares" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Debug" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source" -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/tools" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/task" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Hardwares" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Debug" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


