# TQL-5 Toolchain Identification (R4 / DO-178C residual)

> **Disclaimer.** This document captures the *identity* of the toolchain used
> to produce the PDU image so an auditor can reproduce the exact binary.  It
> does **not** constitute a DO-330 TQL-5 qualification.  Formal Level A
> submission requires the toolchain vendor (or a third-party qualifier such
> as **AdaCore GNATpro Safety-Critical** or **SEGGER Embedded Studio Pro
> Compiler Edition with the qualified GCC for ARM**) to deliver the
> qualification kit.  The PDU project does not, today, use a qualified
> compiler.

## 1. Compiler

| Attribute | Value |
|---|---|
| Vendor | xPack GNU Arm Embedded GCC (community build) |
| Front-end | g++ |
| Version | `12.3.1 20230626` |
| Architecture | `arm-none-eabi` |
| Host | `x86_64-pc-windows` |
| Binary path on the build host | `C:\Users\afifa\.platformio\packages\toolchain-gccarmnoneeabi\bin\arm-none-eabi-gcc.exe` |
| SHA256 of the gcc driver | `136BF1765B5C6CAE31BC31E147FA97CB6FBE1F972CD46951A67CA7A35EF9EF2A` |
| Standard | C++14 |
| Flags | `-Os -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics` |

The SHA256 above is the precise hash of the compiler driver on the build
machine that generated the current `firmware.bin` images.  It can be
re-captured at any time with:

```powershell
Get-FileHash -Algorithm SHA256 `
  C:\Users\afifa\.platformio\packages\toolchain-gccarmnoneeabi\bin\arm-none-eabi-gcc.exe
```

## 2. Linker

| Attribute | Value |
|---|---|
| Linker | GNU `arm-none-eabi-ld` (same xPack package) |
| Linker script | `framework-arduinoststm32 / variants / STM32F1xx / F103C8T_F103CB(T-U) / ldscript.ld` |
| Symbols relied on by the PDU image | `_etext`, `__bss_end__`, `_estack`, `__bss_start__`, `_sbss`, `_ebss`, `_sdata`, `_edata`, `_sidata` |

## 3. Other tools in the build chain

| Tool | Version | Qualification status |
|---|---|---|
| PlatformIO Core | 19.4.0 (`ststm32` platform) | Not DO-330 qualified |
| Arduino-Core-STM32 framework | 4.21100.0 (2.11.0) | Not DO-330 qualified |
| CMSIS | 2.50900.0 (5.9.0) | Vendor-supplied; STMicroelectronics does not certify |
| `objcopy` (for `firmware.bin`) | bundled with xPack GCC 12.3.1 | Not DO-330 qualified |

## 4. What a real Level A submission requires

To replace the residual R4 with an accepted artifact, the project must:

1. **Procure a qualified compiler.**  Practical options (2024-2026 market):
   - **AdaCore GNATpro Safety-Critical for ARM** (qualifies the GNAT/GCC
     toolchain to DO-178C/DO-330 TQL-1 in supported configurations).
   - **SEGGER Embedded Studio Pro / Compiler Edition** (TQL-5).
   - **IAR C/C++ Compiler for Arm with Functional Safety Edition**
     (TQL-1 to TQL-5 depending on configuration).
   - **GNAT GCC with the ECRobot DO-178C Toolchain Qualification Kit**.

2. **Re-build the image** with the qualified compiler.  The PDU source as
   shipped today compiles unmodified under any GCC ≥ 6.3, so swapping the
   compiler is not expected to introduce porting work.

3. **Capture the qualification kit identifiers** (TQL level, kit version,
   certified target list) in this document.

4. **Lock the toolchain in CI** so the published `firmware.bin` is always
   built by a known-qualified compiler version.

Until those four steps are complete, the PDU image inherits the lack of
toolchain qualification of community xPack GCC and CANNOT be claimed as
DO-178C Level A compliant.
