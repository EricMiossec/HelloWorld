*****************************************************************************
*** Files Organization                                                    ***
*****************************************************************************

--{root}                - ChibiOS/RT directory.
  +--readme.txt         - This file.
  +--documentation.html - Shortcut to the web documentation page.
  +--todo.txt           - Current plan (development/unstable versions only).
  +--license.txt        - GPL license text.
  +--exception.txt      - GPL exception text (stable releases only).
  +--boards/            - Board support files.
  +--demos/             - Demo projects.
  +--docs/              - Documentation.
  |  +--html/           - Local HTML documentation (after rebuild).
  |  +--reports/        - Test reports.
  |  +--src/            - Documentation source files (required for rebuild).
  |  +--rsc/            - Documentation resource files (required for rebuild).
  |  +--Doxyfile        - Doxygen project file (required for rebuild).
  |  +--index.html      - Local documentation access (after rebuild).
  +--ext/               - External libraries, not part of ChibiOS/RT.
  +--os/                - ChibiOS/RT files.
  |  +--hal/            - Hardware Abstraction Layer.
  |  |  +--include/     - HAL high level headers.
  |  |  +--src/         - HAL high level source.
  |  |  +--platforms/   - HAL low level drivers implementations.
  |  |  |  +--AT91SAM7/ - Drivers for AT91SAM7 platform.
  |  |  |  +--AVR/      - Drivers for AVR platform.
  |  |  |  +--LPC11xx/  - Drivers for LPC11xx platform.
  |  |  |  +--LPC13xx/  - Drivers for LPC13xx platform.
  |  |  |  +--LPC214x/  - Drivers for LPC214x platform.
  |  |  |  +--MSP430/   - Drivers for MSP430 platform.
  |  |  |  +--SPC56x/   - Drivers for SPC56x/MPC563xx platforms.
  |  |  |  +--STM32/    - Drivers for STM32 platform (common).
  |  |  |  +--STM32F1xx/- Drivers for STM32F1xx platform.
  |  |  |  +--STM32F2xx/- Drivers for STM32F2xx platform.
  |  |  |  +--STM32F4xx/- Drivers for STM32F4xx platform.
  |  |  |  +--STM32L1xx/- Drivers for STM32L1xx platform.
  |  |  |  +--STM8L/    - Drivers for STM8L platform.
  |  |  |  +--STM8S/    - Drivers for STM8S platform.
  |  |  |  +--Posix/    - Drivers for x86 Linux/OSX simulator platform.
  |  |  |  +--Win32/    - Drivers for x86 Win32 simulator platform.
  |  |  +--templates/   - Driver template files.
  |  |     +--meta/     - Driver meta templates.
  |  +--ports/          - Port files for the various architectures.
  |  |  +--GCC/         - Ports for the GCC compiler.
  |  |  |  +--ARM/      - Port files for ARM7 and ARM9 architectures.
  |  |  |  +--ARMCMx/   - Port files for ARMCMx architectures (ARMv6/7-M).
  |  |  |  +--PPC/      - Port files for PowerPC architecture.
  |  |  |  +--AVR/      - Port files for AVR architecture.
  |  |  |  +--MSP430/   - Port files for MSP430 architecture.
  |  |  |  +--SIMIA32/  - Port files for SIMIA32 simulator architecture.
  |  |  +--IAR/         - Ports for the IAR compiler.
  |  |  |  +--ARMCMx/   - Port files for ARMCMx architectures (ARMv6/7-M).
  |  |  +--RVCT/        - Ports for the Keil RVCT compiler.
  |  |  |  +--ARMCMx/   - Port files for ARMCMx architectures (ARMv6/7-M).
  |  |  +--cosmic/      - Ports for the Cosmic compiler.
  |  |  |  +--STM8/     - Port files for STM8 architecture.
  |  |  +--RC/          - Ports for the Raisonance compiler.
  |  |     +--STM8/     - Port files for STM8 architecture.
  |  +--kernel/         - Kernel portable files.
  |  |  +--include/     - Kernel headers.
  |  |  +--src/         - Kernel source.
  |  |  +--templates/   - Kernel port template files.
  |  +--various/        - Various portable support files.
  +--test/              - Kernel test suite source code.
  |  +--coverage/       - Code coverage project.
  +--testhal/           - HAL integration test demos.
  |  +--LPC11xx/        - LPC11xx HAL demos.
  |  +--LPC13xx/        - LPC11xx HAL demos.
  |  +--STM32F1xx/      - STM32F1xx HAL demos.
  |  +--STM32F4xx/      - STM32F4xx HAL demos (valid for STM32F2xx too).
  |  +--STM32L1xx/      - STM32L1xx HAL demos.
  |  +--STM8S/          - STM8S HAL demos.
  +--tools              - Various tools.
     +--eclipse         - Eclipse enhancements.

*****************************************************************************
*** Releases                                                              ***
*****************************************************************************

*** 2.4.1 ***
- FIX: Fixed stack misalignment on Posix-MacOSX (bug 3495487).
- FIX: Fixed STM8S HSI clock initialization error (bug 3489727).
- FIX: Fixed MMC over SPI driver performs an unnecessary SPI read (bug
  3486930).
- FIX: Fixed Realtime counter initialization in STM32 HALs (bug 3485500).
- FIX: Fixed PPC port broken when CH_DBG_SYSTEM_STATE_CHECK is activated
  (bug 3485667).
- FIX: Fixed missing PLL3 check in STM32F107 HAL (bug 3485278).
- FIX: Fixed ADC maximum frequency limit in STM32F2/F4 ADC drivers (bug
  3484947).
- FIX: Fixed various minor documentation errors (bug 3484942).

*** 2.4.0 ***
- NEW: Implemented new makefile system for ARM GCC ports, now objects,
  listings and output files are generated into a "build" directory and not
  together with sources, also implemented a simplified output log mode.
  Now makefiles and load script files are requirements and trigger a
  rebuild if touched.
- NEW: Improved Cortex-Mx port, now the Cortex-M4 are Cortex-M4F are also
  supported. Improved startup files.
- NEW: Added a new hook THREAD_CONTEXT_SWITCH_HOOK() that allows to insert
  code just before a context switch. For example this hook could be used
  in oder to implement advanced power management schemes.
- NEW: Improved Event Flags subsystems in the kernel.
- NEW: Added a macro THD_STATE_NAMES to chthreads.h. This macro is an
  initializer for string arrays containing thread state names.
- NEW: Added a new debug option CH_DBG_SYSTEM_STATE_CHECK that ensures the
  correct API call protocol. If an API is invoked out of the correct context
  then the kernel panics with a debug message.
- NEW: Added the new CMSIS 2.1 headers, now CMSIS resides into a shared
  location: ./os/ports/common/ARMCMx/CMSIS. Old CMSIS files have been
  removed from the various platforms.
- NEW: Removed all the ch.ld files from the ARMCMx demos, now the makefiles
  point to common ld files under the various ports. Less duplication and
  easier maintenance.
- NEW: Improved stack checking and reorganized memory map for the Cortex-Mx
  demos. Now stacks are allocated at the start of the RAM, an overflow of the
  exception stack now triggers an exception (it could go unnoticed before).
  The process stack is organized to be checked on context switch like other
  threads. Now all threads have an explicit stack boundary pointer.
- NEW: Now the port layer exports info regarding the compiler and the port
  options. The info are printed into the test reports. Date and time also
  added.
- NEW: Added initialization of the NVIC VTOR register to all Cortex-Mx (v7M)
  ports. Also added a port option CORTEX_VTOR_INIT to enforce a different
  default value into the register.
- NEW: Added "IRQ_STORM" test applications to those platforms supporting
  ISR preemption.
- NEW: Added a chprintf() function to ./os/various, it can print on any
  BaseChannel interface.
- NEW: Improved the mini shell, enhanced info command, optimizations and
  removed the shellPrint() and shellPrintLine() functions, now it uses
  chprintf() for output.
- NEW: USB driver model and USB implementation for STM32F1xx and STM32L1xx.
- NEW: USB CDC abstract driver.
- NEW: New driver models EXT, GPT, I2C, ICU, RTC, SDC, TM.
- NEW: Improved PWM and MAC driver models.
- NEW: Added support for STM32L1xx, STM32F2xx and STM32F4xx.
- NEW: Updated the STM32F1xx header file to the latest version 3.5.0 and fixed
  it in order to correct several bugs related to the XL family.
- NEW: Added an unified registers file for STM32: stm32.h. This file includes
  the appropriate vendor files then adds its own additional definitions.
- NEW: Added TIM8 support to the STM32 GPT, ICU and PWM drivers.
- NEW: DMA sharing in the STM32 HAL.
- NEW: ADC specific drivers for STM32L1xx, STM32F2xx and STM32F4xx.
- NEW: EXT driver AT91SAM7x and STM32(all).
- NEW: PAL driver for AVR.
- NEW: GPT driver for LPC11xx, LPC13xx and STM32(all).
- NEW: I2C driver for STM32(all).
- NEW: ICU driver for STM32(all).
- NEW: RTC driver for STM32F1xx.
- NEW: SDC driver for STM32F1xx.
- NEW: Added handling of USART6 to the STM32 serial driver.
- NEW: Added demos for the ST STM32F4-Discovery and STM32L1-Discovery kits.
- NEW: Updated AVR demos to use the new PAL driver.
- NEW: Now an error is generated at compile time when trying to enable the
  options CH_DBG_ENABLE_STACK_CHECK on ports that do not support it.
- NEW: Added a kernel-only Cortex-Mx demo as reference project for users not
  interested in the HAL but just want to use the ChibiOS/RT kernel.
  The demo is named ARMCM3-GENERIC-KERNEL and is defaulted to the STM32, in
  order to use it on other families or on the ARM Cortex-M0 just change the
  inclusion paths in the makefile.
- NEW: Integrated new FatFs version 0.8b.
- NEW: lwIP 1.4.0 has been integrated, this new version does not require
  custom hooks into the Thread structure and is thus much lighter.
- CHANGE: Now the callback associated to input queues is invoked before
  reading each character. Previously it was invoked only before going
  to sleep into the THD_STATE_WTQUEUE state.
- CHANGE: Removed the option CH_USE_NESTED_LOCK, lwIP no more requires it and
  it would have conflicted with CH_DBG_SYSTEM_STATE_CHECK which is far more
  useful.
- CHANGE: Renamed the scheduler functions chSchIsRescRequiredExI() to
  chSchIsPreemptionRequired(), chSchDoRescheduleI() to chSchDoReschedule(),
  chSysSwitchI() to chSysSwitch(). All those functions were special cases
  and not regular I-class APIs.
- CHANGE: Renamed the macros IDLE_THREAD_STACK_SIZE and INT_REQUIRED_STACK
  to PORT_IDLE_THREAD_STACK_SIZE and PORT_INT_REQUIRED_STACK for consistency.
- CHANGE: Removed the "old" Cortex-M3 port from the code, the current port
  has no drawbacks and the old port is now just a maintenance cost.
- CHANGE: Removed the CH_CURRP_REGISTER_CACHE option, it is GCC-specific so
  it does not belong to the kernel options. The feature will be eventually
  reimplemented as a port-specific option.
- CHANGE: Renamed the demo ARMCM3-STM32F107-GCC in ARMCM3-STM32F107 and added
  IAR and Keil projects.
- CHANGE: Now the ARMCM3-STM32F107 demo targets the board Olimex STM32-P107
  as default.
- CHANGE: Removed all the prefixes from the structure/union field names
  in the HAL subsystem.
- CHANGE: Updated the documentation to use Doxygen 1.7.4 which produces a much
  more readable output. Also modified the documentation layout to put functions
  and variables ahead of everything else in the group pages.
  Doxygen version below 1.7.4 cannot be used anymore because differences in
  templates. Note that now there are two Doxygen projects, one for generating
  the CHM file the other for plain HTML.

*** 2.2.8 ***
- NEW: Added new API chThdExitS() in order to allow atomic operations on
  thead exit.
- FIX: Fixed Extra initialization in STM32 SPI driver (bug 3436127).
- FIX: Fixed DMA priority setting error in STM32 UART driver (bug 3436125).
- FIX: Fixed DMA priority setting error in STM32 SPI driver (bug 3436124).
- FIX: Fixed broken support for UART5 in STM32 serial driver (bug 3434094).
- FIX: Fixed misplaced chRegSetThreadName() in ARM7-AT91SAM7S-FATFS-GCC demo
  (bug 3411780).
- FIX: Fixed missing UART5 definition in STM32 HAL (bug 3411774).
- FIX: The function chThdExit() triggers an error on shell return when the
  system state checker is enabled (bug 3411207).
- FIX: Some ARMCMx makefiles refer the file rules.mk in the ARM7 port (bug
  3411180).

*** 2.2.7 ***
- INFO: GCC test runs performed with YAGARTO 4.6.0.
- NEW: Added debug plugin for Eclipse under ./tools/eclipse.
- NEW: The debug macros chDbgCheck() and chDbgAssert() now can be externally
  redefined. The macro chDbgCheck() no more includes the line number in the
  description because incompatibility with the Cosmic compiler.
- NEW: Added a new functionality to the registry subsystem, now it is possible
  to associate a name to the threads using chRegSetThreadName. The main and
  idle threads have their name assigned by default.
- FIX: Fixed wrong check on CH_DBG_ENABLE_STACK_CHECK setting (bug 3387671).
- FIX: Fixed wrong APB1 frequency check (bug 3361039).
- FIX: Fixed missing state in shell demos (bug 3351556).
- CHANGE: Removed todo.txt file, it does not belong to the stable version.

*** 2.2.6 ***
- FIX: Fixed race condition in Cortex-Mx ports (bug 3317500).
- FIX: Fixed wrong macro check in STM32 UART driver (bug 3311999).

*** 2.2.5 ***
- FIX: Fixed STM32F107 demo build failure (bug 3294998).
- NEW: Reorganization of the Cortex-Mx ports in order to reduced code and
  comments duplication in the various headers (backported to 2.2.5).
- NEW: Improved the ARMv7-M sub-port now there are two modes: Compact and
  Advanced.
  The advanced mode is equivalent to the previous versions, the compact mode
  is new and makes the kernel *much* smaller and generally faster but does
  not support fast interrupts.
- NEW: Added to the ARMv6-M sub-port an option to use the PendSV exception
  instead of NMI for preemption.

*** 2.2.4 ***
- FIX: Fixed CodeBlocks demo broken (bug 3304718).
- FIX: Fixed race condition in output queues (bug 3303908).
- FIX: Fixed CH_USE_HEAP and CH_USE_MALLOC_HEAP conflict (bug 3303841).
- FIX: Fixed timeout problem in the lwIP interface layer (bug 3302420).
- FIX: Fixed invalid BRR() macro in AVR serial driver (bug 3299306).
- FIX: Fixed missing IRQ vectors amicable names for STM32 XL devices (bug
  3298889).
- FIX: Fixed wrong identifier in AVR serial driver (bug 3292084).
- FIX: Fixed wrong macro check for STM32 XL devices (bug 3291898).
- FIX: Fixed SPI driver restart in STM32 SPI driver implementation, also
  applied the same fix to the STM8S SPI driver (bug 3288758).
- FIX: Fixed missing state transition in ADC driver (bug 3288149).
- FIX: Fixed missing state transition in SPI driver (bug 3288112).
- NEW: Added an option to the kernel to not spawn the Idle Thread from within
  chSysInit(), this way the application can spawn a custom idle thread or
  even use the main() thread as idle thread.
- CHANGE: chiQGetFullI() and chOQGetFullI() become macros. The queues
  subsystem has been optimized and is no more dependent on semaphores.
  Note that the queues callbacks invocation policy has been slightly
  changed, see the documentation.

*** 2.2.3 ***
- FIX: Fixed insufficient idle thread stack in Cortex-M0-GCC port
  (bug 3226671).
- FIX: Fixed stack checking in Cortex-M0-GCC port (bug 3226657).
- FIX: Fixed wrong checks in PAL driver (bug 3224681).
- FIX: Fixed wrong checks in I/O Queues (bug 3219197).
- FIX: Fixed invalid assertion in adcConvert() (bug 3205410).
- NEW: Implemented stack checking in the Cortex-Mx RVCT port.
- NEW: Improved preemption implementation for the Cortex-M0, now it uses
  the NMI vector in order to restore the original context. The change makes
  IRQ handling faster and also saves some RAM/ROM space. The GCC port code
  now does not inline the epilogue code in each ISR saving significant ROM
  space for each interrupt handler in the system.

*** 2.2.2 ***
- FIX: Fixed race condition in CM0 ports, the fix also improves the
  ISR latency (bug 3193062).
- FIX: Fixed Cortex-Mx linker scripts alignment of __heap_base__, the
  correct alignment is now enforced at runtime into core_init() in order
  to make the OS integration easier (bug 3191112).
- FIX: Fixed error in function chCoreAllocI() function documentation (bug
  3191107).
- FIX: Fixed minor problem with memory pools (bug 3190512).
- NEW: Added I-Class functions to the MailBoxes subsystem, now it is
  possible to use them as a transport layer between ISRs and Threads.
- CHANGE: Swapped the numeric values of the TIME_IMMEDIATE and TIME_INFINITE
  constants. Fixed the relative documentation in various places.
- Documentation related fixes.

*** 2.2.1 ***
- FIX: Stack overflow in CM0 ports when nearing interrupts saturation (bug
  3187105).
- FIX: Fixed missing e200z test report (bug 3182611).
- FIX: Fixed error in _BSEMAPHORE_DATA macro (bug 3184139).
- FIX: Error in MAC driver (bug 3179783).
- FIX: Fixed wrong serial driver macros (bug 3173336).

*** 2.2.0 ***
- NEW: The Cortex-Mx port now also supports the IAR and Keil compilers.
- NEW: Improvements to the Cortex-Mx port.
- NEW: Improved ARM port supporting both ARM7 and ARM9.
- NEW: Support for binary semaphores in the kernel.
- NEW: Improved kernel hooks.
- NEW: Extensive improvements to the STM32 platform support.
- NEW: SPI drivers for the AT91SAM7x, LPC11xx, LPC13xx, LPC214x, STM8S,
  platforms.
- NEW: Unified STM8 port for both the Cosmic and the Raisonance compilers.
- NEW: Demos for the STM8S-Discovery, STM8L-Discovery, STM32VL-Discovery
  boards.
- NEW: Improved almost all the existing device driver models in the HAL.
- NEW: Added test/example applications for all the device drivers in the HAL.
- NEW: Added support for the STM8L platform.
- NEW: New UART unbuffered serial device driver model.
- NEW: Greatly improved documentation.
- NEW: Lots of other minor improvements and optimizations, see the change
  log of the 2.1.x development branch for details.

*** 2.0.10 ***
- FIX: Fixed missing lines from the STM32 PWM driver (bug 3154403).
- FIX: Fixed error in chIOGetxxxxxEventSource() macros (bug 3153550).
- FIX: Fixed switch condition error in STM32 PWM driver (bug 3152482).

*** 2.0.9 ***
- FIX: Fixed error in output queues static initializer (bug 3149141).
- FIX: Fixed extra notifications in input queues (bug 3148525).
- FIX: Fixed error in sdPutTimeout() macro (bug 3138763).
- FIX: Fixed preprocessing crt0.s fail (bug 3132293).
- Documentation related fixes.

*** 2.0.8 ***
- FIX: Fixed failed memory recovery by registry scan, improved the related
  test case (bug 3116888).
- FIX: Fixed PWM channels going to ACTIVE state when the pulse width is
  set to zero in the STM32 PWM driver (bug 3114481).
- FIX: Fixed PWM channels return to IDLE state in STM32 PWM driver (bug
  3114467).
- CHANGE: Bugs 3114467 and 3114481 have been fixed by backporting the 2.1.x
  PWM driver, there is a difference in the PWM callback parameters.

*** 2.0.7 ***
- FIX: Fixed typo in board name (bug 3113574).
- FIX: Fixed defective event wait functions with timeout (bug 3113443).

*** 2.0.6 ***
- FIX: Fixed typo in memstreams.h (bug 3089567).
- FIX: Fixed wrong macro check in LPC214x and AT91SAM7 serial drivers (bug
  3088776).
- FIX: Fixed non functioning option SPI_USE_MUTUAL_EXCLUSION=FALSE (bug
  3084764).
- FIX: Fixed wrong macro check in STM32 serial support (but 3078891).
- FIX: Fixed lwIP demo not working (bug 3076354).
- FIX: Fixed non functioning option CH_USE_NESTED_LOCKS (bug 3075544).
- CHANGE: The API chThdInit() has been renamed to chThdCreateI().

*** 2.0.5 ***
- FIX: Incorrect AT91SAM7X initialization, thanks Leszek (bug 3075354).
- FIX: Fixed race condition in function chSchGoSleepTimeoutS, thanks Bal�zs
  (bug 3074984).
- FIX: Fixed race condition in threads creation (bug 3069854).
- FIX: Fixed broken CH_DBG_ENABLE_STACK_CHECK option in legacy CM3 port (bug
  3064274).
- FIX: Fixed CAN_USE_SLEEP_MODE setting (bug 3064204).

*** 2.0.4 ***
- FIX: Fixed potential issue with GCC reorganizing instructions around "asm
  volatile" statements (bug 3058731).
- FIX: Fixed reduced ARM7 performance with GCC 4.5.x (bug 3056866).

*** 2.0.3 ***
- Tests reports regenerated using GCC 4.5.1, small performance improvements
  in all benchmarks.
- FIX: Fixed crash of the Posix simulator under Ubuntu 10.4 (bug 3055329).
- FIX: Fixed incorrect PLL2 setting in STM32 HAL (bug 3044770).
- FIX: Fixed wrong check on STM32_HCLK (bug 3044758).
- FIX: Fixed wrong condition check in STM32 PWM driver (bug 3041414).
- FIX: Corrupted IRQ stack in Cortex-Mx port (bug 3041117).
- FIX: Fixed a documentation error regarding the ADC driver function
  adcStartConversion() (bug 3039890).
- FIX: Fixed insufficient stack size for idle thread (bug 3033624).
- FIX: Fixed misspelled word in some chioch.h and chstreams.h macros (bug
  3031534).
- FIX: Fixed wrong macro check in the STM32 SPI driver (bug 3028562).

*** 2.0.2 ***
- FIX: Fixed invalid context restore in MSP430 port (bug 3027975).
- FIX: Fixed STM32 vectors file (bug 3026528).
- FIX: Fixed race condition in STM32 SPI driver (bug 3025854).
- FIX: Fixed H_LOCK and H_UNLOCK redefined with CH_USE_MALLOC_HEAP (bug
  3025549).
- FIX: Added option to enforce the stack alignment to 32 or 64 bits in the
  Cortex-Mx port (bug 3025133).
- NEW: Added friendly interrupt vectors names to the STM32 HAL (change request
  3023944).
- CHANGE: Removed the option -mabi=apcs-gnu from all the Cortex-Mx demos. The
  option is not compatible with the 64 bits stack alignment now default in
  the Cortex-Mx port. Note that the 64 bits alignment has a cost both as
  performance and as space but it is the "standard".

*** 2.0.1 ***
- FIX: Fixed notification order in input queues (bug 3020708).
- FIX: Fixed non functional CH_CURRP_REGISTER_CACHE option in the Cortex-M3
  port (bug 3020702).
- FIX: Fixed non functional CH_DBG_ENABLE_STACK_CHECK option in the Cortex-M3
  caused by GCC 4.5.0, the fix also improves the context switch performance
  because GCC 4.5.0 apparently was generating useless instructions within the
  very critical context switch code (bug 3019738).
- FIX: Fixed insufficient stack space assigned to the idle thread in
  Cortex-M3 port (bug 3019594).
- FIX: Fixed missing check in chIQReadTimeout() and chIQWriteTimeout() (bug
  3019158).
- FIX: Fixed instability in Mutexes subsystem (bug 3019099).
- NEW: Added timers clock macros to the STM32 clock tree HAL driver.

*** 2.0.0 ***
- NEW: Implemented the concept of thread references, this mechanism ensures
  that a dynamic thread's memory is not freed while some other thread still
  owns a reference to the thread. Static threads are not affected by the new
  mechanism. Two new APIs have been added: chThdAddRef() and chThdRelease().
- NEW: Now more than one thread can be waiting in chThdWait() as long they
  own a reference.
- NEW: Implemented a new threads registry subsystem, the registry allows to
  enumerate the active threads at runtime and/or from a debugger. This is
  a preparatory step for a dedicated ChibiOS/RT debugger.
- NEW: New chCoreFree() API that returns the core memory left.
- NEW: Added a PowerPC port and demo targeting the SPC563M/MPC563xM
  ST/Freescale automotive SOCs.
- NEW: Added STM8 port and demo targeting the Raisonance REva board
  with STM8S208RB piggyback.
- NEW: New unified ARM Cortex-Mx port, this port supports both the ARMv6M
  and ARMv7-M architectures (Cortex-M0/M1/M3/M4 so far). The new port also
  allow to easily add to new Cortex-M implementations by simply adding a
  parameters file (cmparams.h).
- NEW: Improved clock initialization for the STM32, now it is possible to
  configure the clock using any clock source and any HSE frequency.
- NEW: The STM32 clock tree parameters and checks are now calculated into
  a separate file in order to support multiple clock trees for different
  sub-families of the STM32 platform.
- NEW: Added separated clock trees for the STM32 LD/MD/HD sub-family and
  the CL sub-family. Now the selection of the sub-family is done in the
  board.h file, there is no more the need to put -DSTM32F10X_xx into
  the makefile.
- NEW: Added support for STM32/HD/CL UART4 and UART5, thanks Egon for the
  patch.
- NEW: Embedded Artists LPCxpresso Base Board support files added.
- NEW: LPC11xx support, drivers (Serial, PAL, HAL) and demo.
- NEW: LPC13xx support, drivers (Serial, PAL, HAL), demo and reports.
- NEW: The port layer now can "capture" the implementation of individual
  scheduler API functions in order to provide architecture-optimized
  versions. This is done because further scheduler optimizations are
  becoming increasingly pointless without considering architecture and
  compiler related constraints.
- NEW: Updated the STM32 FW Library files to latest version 3.3.0.
- NEW: AT91SAM7 HAL support for the DGBU UART peripheral, as SD3.
- NEW: Added a demo for the AT91SAM7S256 and board files for the Olimex
  SAM7-P256. The demo has been contributed by Alexander Kozaruk.
- NEW: Added core variant name macro in chcore.h and platform name in
  hal_lld.h, the info are printed in the test report and from the "info"
  shell command.
- NEW: Added BOARD_NAME macro to the various board.h files.
- NEW: Added a MemoryStream class under ./os/various.
- NEW: Added Mac OS-X support for the simulator. The Linux simulator has
  been renamed to Posix simulator in order to include this change in a
  single project.
- NEW: New articles, sections and various improvements to the documentation.
- NEW: Added to the simulators shell demos two new commands: threads and mem,
  that show the currently active threads (using the new registry) and the
  memory allocators state.
- NEW: New articles and guides in the documentation.
- OPT: New Cortex-M3 port code, *huge* performance improvements in all the
  context switching related benchmarks (up to 18% depending on the benchmark).
  The new code does no more require the use of the PendSV vector that is
  thus available to the user, it also saves four RAM bytes for each thread
  in the system. The old code is still available as a fall back option while
  the new one is being hardened by peer review and time, the two ports are
  perfectly interchangeable.
- OPT: Speed/size optimization to the events subsystem.
- OPT: Speed/size optimization to the mutexes subsystem.
- OPT: Speed/size optimization to the condvars subsystem.
- OPT: Speed/size optimization to the synchronous messages subsystem.
- OPT: Small size optimization in the semaphores subsystem.
- OPT: Minor optimizations in the "compact" code path.
- OPT: Optimization on the interface between scheduler and port layer, now
  the kernel is even smaller and the context switch performance improved
  quite a bit on all the supported architectures.
- OPT: Simplified the implementation of chSchYieldS() and made it a macro.
  The previous implementation was probably overkill and took too much space
  even if a bit faster.
- OPT: Internal optimization in the serial driver, it now is a bit smaller
  and uses less RAM (all architectures).

*** 1.4.3 ***
- FIX: Fixed centralized ARM makefile (bug 2992747).
- FIX: Fixed write problems in MMC_SPI driver (bug 2991714).
- FIX: Fixed wrong macro check in serial.h (bug 2989459).

*** 1.4.2 ***
- FIX: Fixed missing reschedule in chEvtSignal() (bug 2961208).
- FIX: Removed C99-style variables declarations (bug 2964418).
- Minor documentation fixes.

*** 1.4.1 ***
- FIX: Fixed wrong UART deinitialization sequence in LPC214x serial driver
  (bug 2953985).
- FIX: Fixed wrong PINSEL2 offset into lpc214x.h (bug 2953981).
- FIX: Fixed invalid UART-related macro in the LPC214x HAL (bug 2953195).
- FIX: Wrong prototype in template file chcore.c (bug 2951529).
- FIX: Fixed insufficient stack space for the idle thread in the ARMCM3 port
  when compiling without optimizations (bug 2946233).
- FIX: Fixed wrong notes on function chThdResume() (bug 2943160).
- FIX: Fixed missing dependencies check for CH_USE_DYNAMIC (bug 2942757).
- FIX: Fixed swapped thread states descriptions (bug 2938445).
- FIX_ Fixed C99-style variable declaration (bug 2938444).

*** 1.4.0 ***
- Full test cycle and test reports updated.
- NEW: Reorganized and rationalized the distribution tree and the
  documentation.
- NEW: Abstract Streams and I/O Channels mechanisms introduced.
- NEW: Added a new core memory manager.
- NEW: Improved Heap and Pools allocators.
- NEW: The I/O queues code has been improved, now there are 2 separate
  structures: InputQueue and OutputQueue.
- NEW: Added timeout specification to the I/O queues read/write primitives.
- NEW: Static initializers macros introduced for most kernel objects.
- NEW: Added new APIs chSchDoYieldS() and chThdYield().
- NEW: Improved and simplified kernel configuration files.
- MEW: Added new benchmarks and test cases.
- NEW: Added more test cases in order to improve the test suite code coverage
  (it was 74% in version 1.2.0, it is now close to 100%).
- NEW: Added a code coverage analysis application under ./tests/coverage.
- NEW: Added the test suite documentation to the general documentation.
- NEW: Linux x86 simulator demo added.
- NEW: Improved the Cortex-M3 preemption code.
- NEW: Added standard CMSIS 1.2.0 support to the Cortex-M3 port.
- NEW: Added support for the ST firmware library to the STM32 port.
- NEW: Added support for HD and CL STM32 devices.
- NEW: Improvements to the AT91SAM7 support.
- NEW: Improved makefiles and makefile fragments, now the paths are not fixed.
- NEW: Unified the initialization of the various drivers from a single HAL
  driver. The single drivers can be enabled or disabled from a HAL
  configuration file halconf.h.
- NEW: Hardware Abstraction Layer (HAL) with support for PAL, ADC, CAN, MAC,
  MMC/SD, PWM, Serial, SPI drivers. Added driver implementations to the
  various platforms.
- NEW: Added support for uIP, lwIP, FatFS external libraries, added demos.
- Many many other improvements and minor features.

*** 1.2.4 ***
- FIX: Fixed GCC 4.4.x aliasing warnings (bug 2846336).
- FIX: Modified linker scripts for GCC 4.4.x (bug 2846302).
- FIX: Fixed the CH_OPTIMIZE_SPEED option in the CM3 port (bug 2846278).
- FIX: Fixed GCC 4.4.x related problems in CM3 port (bug 2846162).
- FIX: Fixed LPC214x UART problem (bug 2841088).

*** 1.2.3 ***
- FIX: Fixed C99-style variable declarations (bug 2792919).
- FIX: Fixed instance of obsolete CH_USE_TERMINATE option in the C++ wrapper
  (bug 2796065).
- FIX: Insufficient stack allocated to the C++ LPC2148 demo (bug 2796069).
- FIX: Fixed errors in events test case (bug 2796081).
- CHANGE: Increased main stack size to 1KiB for all the ARMx demos, 2KiB for
  the C++ LPC2148 demo. This should make things easier for unexperienced
  users.

*** 1.2.2 ***
- FIX: Fixed macro in test.h (bug 2781176).
- FIX: Fixed @file tag in sam7x_serial.c (bug 2788573).
- FIX: Fixed sequence assertion in test.c (bug 2789377).
- FIX: Fixed test_cpu_pulse() incorrect behavior (bug 2789383).
- FIX: Fixed missing volatile modifier for p_time field in Thread structure
  (bug 2789501).
- CHANGE: Made the option CH_DBG_THREADS_PROFILING default to TRUE because it
  is now required in order to execute the whole test suite. Note that this
  option is very light so there is no real overhead in the system.
- Added a (harmless) workaround to the Cortex-M3 startup file in order to
  make the RIDE7 demo compile on an unmodified distribution.

*** 1.2.1 ***
- FIX: Fixed regression in MinGW demo (bug 2745153).
- FIX: Fixed problem with the timeout constant TIME_IMMEDIATE (bug 2755170).
- FIX: Fixed a problem in semaphores test case #2 (bug  2755195).
- FIX: Removed unused list functions (bug 2755230).
- FIX: Added the exception notes into the source headers (bug 2772129).
- FIX: Added license notice to several files (bug 2772160).
- FIX: Found new instances of the obsolete function chSysGetTime() in the
  C++ wrapper and in the WEB demo (bug 2772237).

*** 1.2.0 ***
- Full test cycle and test reports updated.
- NEW: Better separation between the port code and the system APIs, now an
  architecture-specific "driver" contains all the port related code.
  Port functions/macros are no more directly exposed as APIs to the user code.
- NEW: Added a configuration option to enable nested system locks/unlocks.
- NEW: Improved the interrupt handlers related code. Now interrupts are
  handled in a very similar way in every architecture. See the "Concepts"
  section and the "Writing interrupt handlers under ChibiOS/RT" article in the
  documentation.
- NEW: Added the chEvtSignal() and chEvtSignalI() APIs that allows direct
  thread signaling, much more efficient that chEvtBroadcast() when the target
  is a known single thread.
- NEW: Added a configuration option that enables the priority enqueuing on
  semaphores. It is defaulted to off because usually semaphores are used for
  I/O related tasks without hard realtime requirements.
- NEW: Now the all the options in chconf.h and the various driver headers
  can be overridden externally, as example from within the Makefile.
  The options are no mode a simple define but a define with an assigned
  TRUE/FALSE value within an #ifndef block.
- NEW: Idle thread hook macro added to the configuration file.
- NEW: Changed the ARM7 and Cortex-M3 startup files, now the action when
  the main() function returns can be overridden by redefining the symbol
  MainExitHandler.
- NEW: Mailboxes (asynchronous messages) subsystem and test cases added.
- NEW: Most APIs with a timeout specification now accept the constant
  TIME_IMMEDIATE (-1) that triggers an immediate timeout when trying to enter
  a sleep state.
- NEW: Mode flexible debug configuration options, removed the old CH_USE_DEBUG
  and CH_USE_TRACE. Replaced with CH_DBG_ENABLE_CHECKS, SCH_DBG_ENABLE_ASSERTS,
  CH_DBG_ENABLE_TRACE  and CH_DBG_FILL_THREADS.
- NEW: Added a debug option CH_DBG_THREADS_PROFILING for threads profiling.
  A field into the Thread structure counts the consumed time. The information
  is not used into the kernel, it is meant for debugging.
- NEW: Added a debug option CH_DBG_ENABLE_STACK_CHECK for stack overflow
  checking. The check is not performed in the kernel but in the port code.
  Currently only the ARM7 and ARMCM3 ports implements it.
- NEW: Unified makefiles for ARM7, ARMCM3 MSP430 projects, the new makefiles
  share a common part making them easier to maintain. Also reorganized the
  demo-specific part of the makefile, now it is easier to configure and the
  option can be overridden from outside.
- OPT: Improved ARM7 thumb port code, thanks to some GCC tricks involving
  registers usage now the kernel is much smaller, faster and most OS APIs
  use less RAM in stack frames (note, this is an ARM7 thumb mode specific
  optimization).
- OPT: Small optimization to the Cortex-M3 thread startup code, improved thread
  related performance scores and smaller code.
- OPT: Alternative, non-inlined and more compact, implementations for
  port_lock() and port_unlock() in the Cortex-M3 port when CH_OPTIMIZE_SPEED
  is FALSE.
- OPT: Improved ready list and priority ordered lists code, some space saved,
  better context switch performance.
- CHANGE: Now the API chThdSetPriority() returns the old priority instead
  of void.
- CHANGE: Modified the signature of the chMsgSendWithEvent() API, it now uses
  a more efficient event signaling method.
- CHANGE: Removed the field p_tid from the Thread structure and the related
  code, this improved the thread creation scores (~2%) and saves some RAM.
  The trace buffer field cse_tid is now populated with a simple hash of the
  thread pointer as thread identifier.
- CHANGE: Renamed the macros chSysIRQEnter() and chSysIRQExit() in
  CH_IRQ_PROLOGUE() and CH_IRQ_EPILOGUE() in order to make very clear that
  those are not functions but inlined code. Also introduced a new macro
  CH_IRQ_HANDLER that should be used when declaring an interrupt handler.
- CHANGE: Renamed several internal initialization functions by removing the
  "ch" prefix because could not be considered system APIs.
- CHANGE: Changed the chSemFastWaitS() macro in chSemFastWaitI() and
  chSemGetCounter() in chSemGetCounterI().
- Improved ARM7 and Cortex-M3 support, new configuration options.
- Introduced the concept of interrupt classes, see the documentation.
- Introduced the concept of system states, see the documentation.
- Huge improvements to the documentation.
- Articles and notes previously in the wiki now merged in the general
  documentation and updated, the wiki entries are obsolete and will be removed.
- New application notes and articles added.
- Added kernel size metrics to the test reports.
- Removed the inclusion graph from the documentation because the little
  info they add and the size of all the images. It is possible to configure
  Doxygen to have them again (and more graph types).
- Improvements to the test suite, added a new level of indirection that allows
  to make tests depend on the configuration options without have to put #ifs
  into the test main module. New benchmarks about semaphores and mutexes.
- Modified the test thread function to return the global test result flag.
- Removed testcond.c|h and moved the test cases into testmtx.c. Mutexes and
  condvars have to be tested together.
- Added architecture diagram to the documentation.

*** 1.0.2 ***
- FIX: Fixed priority inheritance problem with condvars (bug 2674756).
- FIX: Fixed a problem in time ranges (bug 2680425).
- Replaced ./docs/index.html with a direct shortcut to the documentation.

*** 1.0.1 ***
- NEW: Added to the STM32 demo makefile an option to build ChibiOS/RT with the
  full STM32 FWLib 2.03.
  Note that, except for the compile option, the library is not used by the
  OS nor supported.
- FIX: Fixed a problem into the STACK_ALIGN() macro.
- FIX: Fixed a problem with a wrong declaration of the PLL structure in the
  file lpc214x.h.
- FIX: Modified the default value for the STM32 HSI setup it was 1, it should
  be 0x10.
- FIX: Removed an obsolete constant (P_SUSPENDED) from thread.h.
- FIX: Removed unused field mp_grow in the MemoryPool structure.
- FIX: Fixed wrong assertions in chThdWait() and chHeapFree().
- FIX: Fixed a problem with some event APIs not showing in the documentation.

*** 1.0.0 ***
- License switch, added GPL exception, see exception.txt.
- Full test cycle and test reports updated.
- Renamed some occurrences of "Conditional Variable" in "Condition Variable" in
  the documentation.
- FIX: Fixed some images in the documentation because problems when seen in
  Internet Explorer.
