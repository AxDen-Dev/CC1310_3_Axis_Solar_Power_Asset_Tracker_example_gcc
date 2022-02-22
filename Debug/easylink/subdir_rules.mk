################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
easylink/%.o: ../easylink/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"/Applications/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1" -c -mcpu=cortex-m3 -march=armv7-m -mthumb -mfloat-abi=soft -DDeviceFamily_CC13X0 -DCCFG_FORCE_VDDR_HH=0 -DSUPPORT_PHY_CUSTOM -DSUPPORT_PHY_50KBPS2GFSK -DSUPPORT_PHY_625BPSLRM -DSUPPORT_PHY_5KBPSSLLR -I"/Users/jangsangjin/workspace_v10/CC1310_3_Axis_Solar_Power_Asset_Tracker_example_gcc" -I"/Users/jangsangjin/ti/simplelink_cc13x0_sdk_4_20_01_03/source/ti/posix/gcc" -I"/Users/jangsangjin/ti/simplelink_cc13x0_sdk_4_20_01_03/kernel/tirtos/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/include/newlib-nano" -I"/Users/jangsangjin/ti/simplelink_cc13x0_sdk_4_20_01_03/kernel/tirtos/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/include" -I"/Applications/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include" -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"easylink/$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


