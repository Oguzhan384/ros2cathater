################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-230966529: ../c2000.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"/opt/ti/sysconfig_1.27.0/sysconfig_cli.sh" --script "/home/amir/workspace_v12/C2000_EncoderandADC/c2000.syscfg" -o "syscfg" -s "/opt/ti/c2000/C2000Ware_26_01_00_00/.metadata/sdk.json" -d "F2837xD" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-230966529 ../c2000.syscfg
syscfg/board.h: build-230966529
syscfg/board.cmd.genlibs: build-230966529
syscfg/board.opt: build-230966529
syscfg/board.json: build-230966529
syscfg/pinmux.csv: build-230966529
syscfg/c2000ware_libraries.cmd.genlibs: build-230966529
syscfg/c2000ware_libraries.opt: build-230966529
syscfg/c2000ware_libraries.c: build-230966529
syscfg/c2000ware_libraries.h: build-230966529
syscfg/clocktree.h: build-230966529
syscfg: build-230966529

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/home/amir/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="/home/amir/workspace_v12/C2000_EncoderandADC" --include_path="/home/amir/workspace_v12/C2000_EncoderandADC/device" --include_path="/opt/ti/c2000/C2000Ware_26_01_00_00/driverlib/f2837xd/driverlib" --include_path="/home/amir/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --define=_LAUNCHXL_F28379D --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="/home/amir/workspace_v12/C2000_EncoderandADC/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/home/amir/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="/home/amir/workspace_v12/C2000_EncoderandADC" --include_path="/home/amir/workspace_v12/C2000_EncoderandADC/device" --include_path="/opt/ti/c2000/C2000Ware_26_01_00_00/driverlib/f2837xd/driverlib" --include_path="/home/amir/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --define=_LAUNCHXL_F28379D --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="/home/amir/workspace_v12/C2000_EncoderandADC/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


