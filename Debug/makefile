################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /opt/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./ak8975.obj" \
"./art-logger_work_ver1.obj" \
"./comp_dcm.obj" \
"./ff.obj" \
"./i2cm_drv.obj" \
"./mmc-ek-tm4c1294xl.obj" \
"./mpu9150.obj" \
"./tm4c1294ncpdt_startup_ccs.obj" \
"./uartstdio.obj" \
"./ustdlib.obj" \
"../tm4c1294ncpdt.cmd" \
$(GEN_CMDS__FLAG) \
-l"C:\ti\TivaWare_C_Series-2.1.2.111\driverlib\ccs\Debug\driverlib.lib" \
-l"C:\ti\TivaWare_C_Series-2.1.2.111\sensorlib\ccs\Debug\sensorlib.lib" \
-llibc.a \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
art-logger_final_work_ver1.out \

EXE_OUTPUTS__QUOTED += \
"art-logger_final_work_ver1.out" \

BIN_OUTPUTS += \
art-logger_final_work_ver1.hex \

BIN_OUTPUTS__QUOTED += \
"art-logger_final_work_ver1.hex" \


# All Target
all: art-logger_final_work_ver1.out

# Tool invocations
art-logger_final_work_ver1.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"/opt/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --diag_warning=225 --diag_wrap=off --display_error_number --printf_support=full -z -m"art-logger_final_work_ver1.map" --stack_size=8192 --heap_size=0 -i"/opt/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/lib" -i"/opt/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="art-logger_final_work_ver1_linkInfo.xml" --rom_model -o "art-logger_final_work_ver1.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

art-logger_final_work_ver1.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"/opt/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armhex"  -o "art-logger_final_work_ver1.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ak8975.obj" "art-logger_work_ver1.obj" "comp_dcm.obj" "ff.obj" "i2cm_drv.obj" "mmc-ek-tm4c1294xl.obj" "mpu9150.obj" "tm4c1294ncpdt_startup_ccs.obj" "uartstdio.obj" "ustdlib.obj" 
	-$(RM) "ak8975.d" "art-logger_work_ver1.d" "comp_dcm.d" "ff.d" "i2cm_drv.d" "mmc-ek-tm4c1294xl.d" "mpu9150.d" "tm4c1294ncpdt_startup_ccs.d" "uartstdio.d" "ustdlib.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

