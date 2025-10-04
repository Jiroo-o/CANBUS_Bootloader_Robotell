# CANBUS Bootloader for STM32F103C8T6 (Robotell)


A custom bootloader for **STM32F103C8T6** that supports firmware flashing over **CAN bus**, designed to work with **Robotell CAN adapter** (or any CAN adapter that can send standard frames).

This repository contains two projects:

* `f103_can_bootloader` — the CAN bootloader (BL)
* `f103_App_TestBlink` — a minimal application (APP) for testing (LED blink)

> Note: Paths and names can be adjusted to your exact board. Defaults below match the typical "Blue Pill"-class boards.

---

## Features

* Flash firmware over **CAN** (standard ID)
* Simple, transparent **framing protocol** (START / DATA / END)
* **CRC32** integrity check before jumping to APP
* **LED PC13** status signals (active‑low)
* Works with Robotell CAN adapter (and generic PC CAN tools)

---

## Hardware

* MCU: **STM32F103C8T6** (Cortex‑M3, 64KB Flash)
* Clock: HSE PLLCLK 72MHz → SYSCLK 72MHz (typical)
* LED: **PC13** (active‑low)
* CAN: **CAN1** via transceiver (mcp2551)

  * Pins (remapped): **PB8 = CAN\_RX, PB9 = CAN\_TX** (`__HAL_AFIO_REMAP_CAN1_2()`)
  * Baudrate example: **1 Mbps** (adjust to your bus)

**Wiring (example)**

```
MCU PB8 (CAN_RX)  →  Transceiver RXD
MCU PB9 (CAN_TX)  →  Transceiver TXD
MCU GND           ↔  Transceiver GND
MCU 3V3           →  Transceiver VCC
CAN_H / CAN_L     ↔  Bus
```

---

## Memory Map (default)

* **Bootloader size**: 16 KB
* **APP start**: `0x08004000`
* **Vector remap**: bootloader jumps to APP after validation

Adjust linker scripts (`STM32F103C8TX_FLASH.ld`) of BL/APP accordingly.

---

## Build & Flash

You can build with **STM32CubeIDE** (managed build). Typical steps:

1. Open the project(s) in STM32CubeIDE
2. Select target (Debug/Release) and **Build**
3. Flash **bootloader** first to address `0x08000000`
4. Use the CAN flashing flow below to upload the APP image


---

## Boot Flow

1. Power on → Bootloader runs
2. **Boot window**: wait for START frame; if received, enter update mode
3. Otherwise, if existing APP is valid → jump to APP
4. During update mode: receive DATA frames and write to Flash
5. On END frame, verify **CRC32** → if match, finalize and jump to APP

**LED signals (example)**

* Slow blink = APP running
* Fast blink = BL update mode / flashing in progress

---

## CAN Protocol

Use **standard IDs**. Example default mapping:

* **START (ID `0x7FF`)**:

  * Payload: `['S','T','A','R', size(4B LE)]`
  * `size` = total bytes of the incoming firmware
* **DATA (ID `0x100`)**:

  * Payload: 8 bytes per frame (pad to 4‑byte alignment)
* **END (ID `0x7FE`)**:

  * Payload: `['E','N','D', 0, crc32(4B LE)]`
  * CRC32: Ethernet polynomial `0x04C11DB7`, init `0xFFFFFFFF`, reflect in/out, xorout `0xFFFFFFFF`

> You can change IDs/baudrate to match your system; keep them consistent on both PC tool and MCU.

---

## PC‑side Flashing (Concept)

Any CAN tool/adaptor that can send standard frames works. Basic sequence:

1. Send **START** with total size
2. Stream **DATA** frames with the firmware bytes
3. Send **END** with CRC32 of the whole firmware

Example pseudo‑code:

```text
send(ID=0x7FF, data=['S','T','A','R', size_le])
for chunk in split(fw, 8):
    send(ID=0x100, data=chunk_padded)
send(ID=0x7FE, data=['E','N','D', 0x00, crc32_le])
```

With Robotell adapter, use its PC utility or your own script to push frames in that order.

---

## Safety Notes

* Make sure power is stable during flashing
* Verify flash regions: BL must not overwrite APP (and vice versa)
* CRC mismatch should abort and keep the previous APP
* Consider write‑protection for BL in production

---

## Project Layout (example)

```
CANBUS_Bootloader_Robotell/
├─ f103_can_bootloader/        # bootloader
│  ├─ Core/ Drivers/ ...
│  └─ STM32F103C8TX_FLASH.ld
├─ f103_App_TestBlink/         # demo app
│  ├─ Core/ Drivers/ ...
│  └─ STM32F103C8TX_FLASH.ld
└─ README.md
```


## Acknowledgments

* STMicroelectronics HAL/LL drivers
* Community projects and docs around STM32 & CAN bootloaders

---

