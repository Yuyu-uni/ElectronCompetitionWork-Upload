################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
tools/%.o: ../tools/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/tools" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/task" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Hardwares" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general" -I"D:/PROJECTS/ElecDsinCpet/DianaSai-Control/UploadProjLib/ti_generalctrl/TI_general/Debug" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/M0_SDK/mspm0_sdk_2_04_00_06/source" -gdwarf-3 -MMD -MP -MF"tools/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


