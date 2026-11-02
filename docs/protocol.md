# SPI link protocol

The STM32 is the SPI master and reads and writes 32-bit FPGA registers. Every
frame carries a CRC; a frame that fails it is dropped and counted. The driver
is [`fpga_link.c`](../firmware/FPGA_Link/Core/Src/fpga_link.c); the FPGA side is
[`spi_link.v`](../rtl/spi_link.v).

## Frames

- SPI mode 0 (CPOL 0, CPHA 0), MSB first, CS is a GPIO driven by the MCU.
  Tested at 5.3 and 10.6 MHz.
- CRC-8/SMBUS: poly 0x07, init 0x00, no reflection, no final XOR; check value
  for "123456789" is 0xF4.
- Write, 6 bytes: `[0|addr7] [D31..24] [D23..16] [D15..8] [D7..0] [CRC(0..4)]`.
  Applied only if the CRC matches; otherwise dropped, counted in CRC_ERRS,
  and FAULT_N pulses low.
- Read, 7 bytes: MOSI `[1|addr7] [CRC(byte0)] 00 00 00 00 00`; MISO data in
  bytes 2–5, CRC over the data in byte 6. A bad header CRC returns FFFFFFFF
  with an inverted CRC.
- Burst write, 5 + 4N bytes (N = 1–16):
  `[0x7F] [start addr] [N-1] [CRC(0..2)] [N × 4 data bytes] [CRC(data)]`.
  All-or-nothing: the words are written only if both CRCs match. A rejected
  burst is counted once in CRC_ERRS.
- Burst read, 5 + 4N bytes: MOSI `[0xFF] [start addr] [N-1] [CRC(0..2)] 00 …`;
  MISO carries the N words from byte 4 on, then a CRC over all data bytes.
  A bad header CRC or N > 16 returns all-FF data with an inverted CRC.
- Burst addresses increment from the start address and wrap at 0x7F. A burst
  counts as one frame in FRAMES_OK.
- Frames cut short by releasing CS are discarded.
- Unmapped reads return 0xDEADBEEF. Incompatible changes bump VERSION, and
  the driver's `fpga_init` refuses a mismatch.

## Side-band signals

- **LOOP_TICK** (FPGA → MCU): the control-loop clock, TICK_DIV clocks of
  100 MHz (1 kHz by default). The FPGA latches its snapshot registers on the
  same edge.
- **FPGA_IRQ** (FPGA → MCU): high while any enabled IRQ_STATUS bit is set.
- **FAULT_N** (FPGA → MCU): low while a safety fault is latched; pulses low on
  a rejected frame.
- **FPGA_RST_N** (MCU → FPGA): resets the FPGA logic.

## Throughput (measured, 10.6 MHz)

| Access | Rate |
|---|---|
| Single 32-bit read or write | 73.8k accesses/s |
| 16-word bursts | 176.8k words/s |
| 1 kHz control step (5 frames, DMA, in interrupts) | ~112 µs per step, ~32 µs CPU |
