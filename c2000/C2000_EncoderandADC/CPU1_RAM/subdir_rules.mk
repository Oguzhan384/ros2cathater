################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-522435504: ../c2000.syscfg
	@echo 'SysConfig - building file: "$<"'
	"/home/oguzhan/ti/ccs2100/ccs/utils/sysconfig_1.28.0/sysconfig_cli.sh" -s "/home/oguzhan/ti/C2000Ware_26_01_00_00/.metadata/sdk.json" -d "F2837xD" --script "/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC/c2000.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-522435504 ../c2000.syscfg
syscfg/board.h: build-522435504
syscfg/board.cmd.genlibs: build-522435504
syscfg/board.opt: build-522435504
syscfg/board.json: build-522435504
syscfg/pinmux.csv: build-522435504
syscfg/c2000ware_libraries.cmd.genlibs: build-522435504
syscfg/c2000ware_libraries.opt: build-522435504
syscfg/c2000ware_libraries.c: build-522435504
syscfg/c2000ware_libraries.h: build-522435504
syscfg/clocktree.h: build-522435504
syscfg: build-522435504

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'C2000 Compiler - building file: "$<"'
	"/home/oguzhan/ti/ccs2100/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC" --include_path="/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC/device" --include_path="/home/oguzhan/ti/C2000Ware_26_01_00_00/driverlib/f2837xd/driverlib" --include_path="/home/oguzhan/ti/ccs2100/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/include" --define=_LAUNCHXL_F28379D --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'C2000 Compiler - building file: "$<"'
	"/home/oguzhan/ti/ccs2100/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC" --include_path="/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC/device" --include_path="/home/oguzhan/ti/C2000Ware_26_01_00_00/driverlib/f2837xd/driverlib" --include_path="/home/oguzhan/ti/ccs2100/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/include" --define=_LAUNCHXL_F28379D --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="/home/oguzhan/ros2cathater/ros2cathater/c2000/C2000_EncoderandADC/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


