# Arm CMSIS-View Event Recorder 1.2.0

Copied from the installed official ARM.CMSIS-View.1.2.0 pack.
Upstream source/header are unchanged, licensed Apache-2.0 (see LICENSE).
https://github.com/ARM-software/CMSIS-View

Project configuration changes: 128 records, user timer (source 3), 1000 Hz.
User timer functions in User/src/arm_diag.c use HAL_GetTick (1 ms resolution).
RTE_Components.h is a local adapter selecting stm32h7xx.h.
The linker reserves 4 KB of noncached DTCM for EventRecorder.o ZI data.
128 records need approximately 164 + 16*128 = 2212 bytes, NOT just 2048 bytes.

This source is already in the Keil project. Do not select another Event Recorder
implementation in RTE. This is the DAP implementation, not semihosting/printf.
Changing ARM_DIAG_EVENT_ENABLE disables project event calls. Leave timer adapters
available if EventRecorder.c remains compiled.
