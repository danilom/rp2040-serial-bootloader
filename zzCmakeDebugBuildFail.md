
=======
Adding -DCMAKE_BUILD_TYPE=Debug

Conclusions:
- Fails due to being bigger than 12k (linker failiure0
- Does NOT enable the DEBUG symbol for main.c
	(no message "Debug including stdio_usb_init")

Output:

D:\rp2040-serial-bootloader\build-debug>cmake -G "MinGW Makefiles" .. -DCMAKE_BUILD_TYPE=Debug
Using PICO_SDK_PATH from environment ('C:\VSARM\sdk\pico\pico-sdk')
PICO_SDK_PATH is C:/VSARM/sdk/pico/pico-sdk
Defaulting PICO_PLATFORM to rp2040 since not specified.
Defaulting PICO platform compiler to pico_arm_gcc since not specified.
PICO compiler is pico_arm_gcc
-- The C compiler identification is GNU 12.2.1
-- The CXX compiler identification is GNU 12.2.1
-- The ASM compiler identification is GNU
-- Found assembler: C:/VSARM/armcc/bin/arm-none-eabi-gcc.exe
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: C:/VSARM/armcc/bin/arm-none-eabi-gcc.exe - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: C:/VSARM/armcc/bin/arm-none-eabi-g++.exe - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
Build type is Debug
Using regular optimized debug build (set PICO_DEOPTIMIZED_DEBUG=1 to de-optimize)
Defaulting PICO target board to pico since not specified.
Using board configuration from C:/VSARM/sdk/pico/pico-sdk/src/boards/include/boards/pico.h
-- Found Python3: C:/Python311/python.exe (found version "3.11.3") found components: Interpreter
TinyUSB available at C:/VSARM/sdk/pico/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040; enabling build support for USB.
Compiling TinyUSB with CFG_TUSB_DEBUG=1
BTstack available at C:/VSARM/sdk/pico/pico-sdk/lib/btstack
cyw43-driver available at C:/VSARM/sdk/pico/pico-sdk/lib/cyw43-driver
Pico W Bluetooth build support available.
lwIP available at C:/VSARM/sdk/pico/pico-sdk/lib/lwip
mbedtls available at C:/VSARM/sdk/pico/pico-sdk/lib/mbedtls
-- Configuring done
-- Generating done
-- Build files have been written to: D:/rp2040-serial-bootloader/build-debug

D:\rp2040-serial-bootloader\build-debug>jpo-make

D:\rp2040-serial-bootloader\build-debug>echo Run inside 'build' dir.
Run inside 'build' dir.

D:\rp2040-serial-bootloader\build-debug>mingw32-make
Scanning dependencies of target bs2_default
[  1%] Building ASM object pico-sdk/src/rp2_common/boot_stage2/CMakeFiles/bs2_default.dir/compile_time_choice.S.obj
[  2%] Linking ASM executable bs2_default.elf
[  2%] Built target bs2_default
Scanning dependencies of target bs2_default_padded_checksummed_asm
[  4%] Generating bs2_default.bin
[  5%] Generating bs2_default_padded_checksummed.S
[  5%] Built target bs2_default_padded_checksummed_asm
Scanning dependencies of target ELF2UF2Build
[  7%] Creating directories for 'ELF2UF2Build'
[  8%] No download step for 'ELF2UF2Build'
[ 10%] No update step for 'ELF2UF2Build'
[ 11%] No patch step for 'ELF2UF2Build'
[ 12%] Performing configure step for 'ELF2UF2Build'
-- The C compiler identification is GNU 8.1.0
-- The CXX compiler identification is GNU 8.1.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: C:/VSARM/mingw/mingw32/bin/gcc.exe - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: C:/VSARM/mingw/mingw32/bin/g++.exe - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: D:/rp2040-serial-bootloader/build-debug/elf2uf2
[ 14%] Performing build step for 'ELF2UF2Build'
Scanning dependencies of target elf2uf2
[ 50%] Building CXX object CMakeFiles/elf2uf2.dir/main.cpp.obj
[100%] Linking CXX executable elf2uf2.exe
[100%] Built target elf2uf2
[ 15%] No install step for 'ELF2UF2Build'
[ 17%] Completed 'ELF2UF2Build'
[ 17%] Built target ELF2UF2Build
Scanning dependencies of target bootloader
[ 18%] Building C object CMakeFiles/bootloader.dir/main.c.obj
[ 20%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c.obj
[ 21%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_gpio/gpio.c.obj
[ 22%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_platform/platform.c.obj
[ 24%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_claim/claim.c.obj
[ 25%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_sync/sync.c.obj
[ 27%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_irq/irq.c.obj
[ 28%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S.obj
[ 30%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_sync/sem.c.obj
[ 31%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_sync/lock_core.c.obj
[ 32%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_sync/mutex.c.obj
[ 34%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_sync/critical_section.c.obj
[ 35%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_time/time.c.obj
[ 37%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_time/timeout_helper.c.obj
[ 38%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_timer/timer.c.obj
[ 40%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_util/datetime.c.obj
[ 41%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_util/pheap.c.obj
[ 42%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/common/pico_util/queue.c.obj
[ 44%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_uart/uart.c.obj
[ 45%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_clocks/clocks.c.obj
[ 47%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_pll/pll.c.obj
[ 48%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_vreg/vreg.c.obj
[ 50%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c.obj
[ 51%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_xosc/xosc.c.obj
[ 52%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_divider/divider.S.obj
[ 54%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_runtime/runtime.c.obj
[ 55%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_printf/printf.c.obj
[ 57%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S.obj
[ 58%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c.obj
[ 60%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_divider/divider.S.obj
[ 61%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_double/double_aeabi.S.obj
[ 62%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_double/double_init_rom.c.obj
[ 64%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_double/double_math.c.obj
[ 65%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S.obj
[ 67%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S.obj
[ 68%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_float/float_aeabi.S.obj
[ 70%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_float/float_init_rom.c.obj
[ 71%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_float/float_math.c.obj
[ 72%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S.obj
[ 74%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c.obj
[ 75%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S.obj
[ 77%] Building ASM object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_standard_link/crt0.S.obj
[ 78%] Building CXX object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp.obj
[ 80%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c.obj
[ 81%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_stdio/stdio.c.obj
[ 82%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/pico_stdio_uart/stdio_uart.c.obj
[ 84%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_dma/dma.c.obj
[ 85%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/hardware_flash/flash.c.obj
[ 87%] Building C object CMakeFiles/bootloader.dir/C_/VSARM/sdk/pico/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Source/system_RP2040.c.obj
[ 88%] Linking CXX executable bootloader.elf
c:/vsarm/armcc/bin/../lib/gcc/arm-none-eabi/12.2.1/../../../../arm-none-eabi/bin/ld.exe: bootloader.elf section `.text' will not fit in region `FLASH'
c:/vsarm/armcc/bin/../lib/gcc/arm-none-eabi/12.2.1/../../../../arm-none-eabi/bin/ld.exe: region `FLASH' overflowed by 10172 bytes
collect2.exe: error: ld returned 1 exit status
mingw32-make[2]: *** [CMakeFiles\bootloader.dir\build.make:737: bootloader.elf] Error 1
mingw32-make[1]: *** [CMakeFiles\Makefile2:1795: CMakeFiles/bootloader.dir/all] Error 2
mingw32-make: *** [Makefile:102: all] Error 2
