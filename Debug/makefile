################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /Applications/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major

GEN_OPTS__FLAG := @"/Users/jangsangjin/workspace_v10/tirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03/Debug/configPkg/compiler.opt" 
GEN_CMDS__FLAG := -Wl,-T"/Users/jangsangjin/workspace_v10/tirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03/Debug/configPkg/linker.cmd" 

ORDERED_OBJS += \
"./CC1310_LAUNCHXL.o" \
"./CC1310_LAUNCHXL_fxns.o" \
"./RadioTask.o" \
"./SensorTask.o" \
"./ccfg.o" \
"./main.o" \
"./quectel_gps.o" \
"./easylink/EasyLink.o" \
"./easylink/easylink_config.o" \
"./smartrf_settings/smartrf_settings.o" \
"./smartrf_settings/smartrf_settings_predefined.o" \
-Wl,-T"../CC1310_LAUNCHXL_TIRTOS.lds" \
-l:"ti/display/lib/display.am3g" \
-l:"ti/grlib/lib/gcc/m3/grlib.a" \
-l:"third_party/spiffs/lib/gcc/m3/spiffs_cc26xx.a" \
-l:"ti/drivers/rf/lib/rf_multiMode_cc13x0.am3g" \
-l:"ti/drivers/lib/drivers_cc13x0.am3g" \
-l:"ti/dpl/lib/dpl_cc13x0.am3g" \
$(GEN_CMDS__FLAG) \
-l:"ti/devices/cc13x0/driverlib/bin/gcc/driverlib.lib" \
-lgcc \
-lc \
-lm \
-lnosys \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include easylink/subdir_vars.mk
-include smartrf_settings/subdir_vars.mk
-include subdir_rules.mk
-include easylink/subdir_rules.mk
-include smartrf_settings/subdir_rules.mk
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
CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.out \

EXE_OUTPUTS__QUOTED += \
"CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.out" \

BIN_OUTPUTS += \
CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.hex \

BIN_OUTPUTS__QUOTED += \
"CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.hex" \


# All Target
all: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.out"

dependents:
	-cd /Users/jangsangjin/workspace_v10/tirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03/Debug && $(MAKE) clean all

# Tool invocations
CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.out: $(OBJS) $(LDS_SRCS) $(GEN_CMDS) /Users/jangsangjin/workspace_v10/tirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03/Debug/libtirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03.a
	@echo 'Building target: "$@"'
	@echo 'Invoking: GNU Linker'
	"/Applications/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1" -DDeviceFamily_CC13X0 -DCCFG_FORCE_VDDR_HH=0 -DSUPPORT_PHY_CUSTOM -DSUPPORT_PHY_50KBPS2GFSK -DSUPPORT_PHY_625BPSLRM -DSUPPORT_PHY_5KBPSSLLR -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -march=armv7-m -mthumb -mfloat-abi=soft -Wl,-Map,"CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.map" -nostartfiles -static -Wl,--gc-sections -L"/Users/jangsangjin/ti/simplelink_cc13x0_sdk_4_20_01_03/source" -L"/Users/jangsangjin/ti/simplelink_cc13x0_sdk_4_20_01_03/kernel/tirtos/packages" -L"/Users/jangsangjin/ti/simplelink_cc13x0_sdk_4_20_01_03/kernel/tirtos/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/lib/thumb/v7-m" -L"/Applications/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/lib" -march=armv7-m -mthumb --specs=nano.specs -o"CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: GNU Objcopy Utility'
	"/Applications/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy"  $(EXE_OUTPUTS__QUOTED) "CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc.hex"
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "CC1310_LAUNCHXL.o" "CC1310_LAUNCHXL_fxns.o" "RadioTask.o" "SensorTask.o" "ccfg.o" "main.o" "quectel_gps.o" "easylink/EasyLink.o" "easylink/easylink_config.o" "smartrf_settings/smartrf_settings.o" "smartrf_settings/smartrf_settings_predefined.o" 
	-$(RM) "CC1310_LAUNCHXL.d" "CC1310_LAUNCHXL_fxns.d" "RadioTask.d" "SensorTask.d" "ccfg.d" "main.d" "quectel_gps.d" "easylink/EasyLink.d" "easylink/easylink_config.d" "smartrf_settings/smartrf_settings.d" "smartrf_settings/smartrf_settings_predefined.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
/Users/jangsangjin/workspace_v10/tirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03/Debug/libtirtos_builds_CC1310_LAUNCHXL_release_gcc_4_20_01_03.a:

-include ../makefile.targets

