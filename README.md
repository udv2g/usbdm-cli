# usbdm-cli

A Zig command-line tool for the **FZ0622C** USBDM programmer/debugger.

## Commands

```text
usbdm list [--all]                        # enumerate BDM interfaces
usbdm version | probe | status            # firmware / capabilities / connection
usbdm connect --target hcs08              # bring up the target (retry ladder)
usbdm identify --target hcs12             # read the SDID, name the part(s)
usbdm reset [--normal|--special] [--hardware|--software|--power]
usbdm halt | go | step
usbdm regs | reg pc [0x1234]              # read/write core registers
usbdm read 0x1000 0x100 | write 0x80 aa bb | fill 0x80 0x40 0xff
usbdm dump 0x8000 0x4000 -o fw.s19        # bin/srec/ihex by --format or suffix
usbdm load image.s19                      # write an image into RAM (not flash)
usbdm pins bkgd=low reset=3state | release

usbdm program app.s19 --part mc9s08jm60   # erase affected sectors, program, verify
usbdm verify | erase [0xADDR] | blank-check --part ...
usbdm program app.s19 --part mc9s08sh16 --trim 33.6   # HCS08: re-trim the oscillator
usbdm unsecure | secure --part ...        # clear / set flash security
usbdm eeprom img.s19 --part mc9s12g240    # EEPROM/data-flash (S12G GMMC parts)
usbdm gdb --port 1234                     # GDB remote stub over TCP
```

Exit codes: `0` ok, `2` usage, `3` no device, `4` BDM/target, `5` flash,
`6` verify mismatch, `7` file I/O.

## Build

Requires **Zig 0.16**.

```sh
zig build                              # -> zig-out/bin/usbdm
zig build test                         # run tests
zig build -Dtarget=x86_64-linux-gnu    # cross-compile
zig build -Dsystem-libusb              # link system libusb
```

Builds natively on Linux/macOS/Windows. Windows uses WinUSB.

## `--sim`

Runs any command against a software virtual target. A CPU emulator
executes the real vendored flash routines, so `program` and `gdb` single-step
behave as on-target. USB-only commands (`probe`/`version`/`list`) are declined.

## Flash programming

Reference RAM flash-routine method: vendored routines run on the target CPU.
Geometry (`--part`) comes from a build-time table of 193 parts (vendored USBDM
device DB), which also backs `identify`. Families: HCS08/RS08, HCS12 (classic S12,
S12G, and S12XE/XS/XF on the paged map), CFV1, plus S12G EEPROM. Destructive
commands confirm unless `--force`. HCS12 flash operations start with a
special-mode reset, so the target does not need to be halted first.

### HCS12 flash

Three flash modules are covered, each by its vendored USBDM routine:

| Family | Routine | Parts |
| --- | --- | --- |
| Classic S12 | FTS | e.g. MC9S12DP256 |
| S12G (FTMRG) | GMMC | G48..G240, GA240, VR48/64 (G96 verified) |
| S12XE/XS/XF (XFTMRK) | XMMC | XEP100 family, XS64/128/256, XF512 (XET256 verified) |

Every HCS12 flash command starts with a special-mode hardware reset and a reconnect, so
a board running its own code can be flashed without halting it first (a running target
acks memory access but not the PC write that starts the routine). The FTMRG and XFTMRK
routines report an invalid divider unless `FCLKDIV` is already loaded, so the tool does
what the reference script does before running them: COP off, `FCLKDIV` from the measured
bus clock, `FPROT`/`EPROT` unprotected.

**Paged flash.** The S12G and S12X parts are programmed through the PPAGE window.
`pages`/`pageEnd` come from the device DB: G96 is pages 0x0A-0x0F (96K), and the flat
sliver below 0x4000 is left out because the pages already cover the whole flash. XMMC
entries size the window from the page count (1024K starts at page 0xC0).

**Image handling for FTMRG.**

- `program` and `verify` merge records that share one 8-byte write unit. FTMRG refuses a
  second write to a phrase, which sparse vector records (two vectors in one phrase) hit.
- If the image erases the security sector without defining the security byte, `program`
  writes the unsecured value (`FSEC` = 0xFE). An erased 0xFF reads as secured.

`--sim` refuses XFTMRK parts: there is no S12XE flash model, and the FTS one would report
a meaningless pass.

Verified on hardware: MC9S12XET256 (61 KB across both fixed windows and four PPAGE pages,
read back through the raw BDM) and MC9S12G96 (all segments of an EVSE image verified,
`FSEC` 0xFE, reset vector 0xC000). Not covered: neither image has been tested running on
the G96, and the other S12G parts use the same routine but are untested.

### HCS08 flash bring-up and oscillator trim

Before the routine runs, an HCS08 flash command does what `UsbdmFlashProgrammer` does:
a software special reset and halt (the device DB's default), COP off (`SOPT`), the ICS
clock set to a known speed, and `FCDIV` loaded from the *measured* bus clock (the BDC is
switched to the bus clock, `CLKSW=1`, so the pod's speed reading is the bus). `FCDIV` is
write-once per reset, which is why this has to come first.

The oscillator trim (`NVFTRIM`/`NVTRIM`, 0xFFAE/0xFFAF on ICS parts) lives in the same
sector as the vectors, so programming a normal image erases it and the part then runs
~30 % slow. Two ways to keep it:

- by default the chip's own trim bytes are copied into the image before the erase
  (only where the image leaves them unset);
- `--trim <kHz>` re-trims: the BDM clock is measured at a low speed while the trim is
  searched (binary search, then a +/-8 sweep with a least-squares fit), and the result is
  written into the image. `--part` is required. The frequency is the *internal reference*
  (e.g. 33.6 for an 8.6016 MHz bus on an ICSV2 part); `--nvloc <addr>` overrides where the
  two bytes go (default by clock type, from USBDM's DeviceData.cpp).

```
usbdm program app.s19 --part mc9s08sh16 --trim 33.6
```

Trim is implemented for the ICS clock variants (tested on ICSV2, MC9S08SH16); ICG and MCG
parts get the preserve-by-default behaviour only. A mass erase (`erase`, `unsecure`)
removes the trim and says so.

## GDB remote stub

`usbdm gdb` serves the GDB remote protocol over TCP: registers, memory,
single-step, `continue`/`vCont`, flash `load` (`vFlash*`), `monitor reset`/`halt`.
Hardware breakpoints on HCS08, classic-S12 HCS12, and CFV1; S12G, S12X and
watchpoints use GDB software breakpoints.

## Layout

- `usb` / `protocol` / `transport` / `session` - libusb wrapper, command set,
  bulk framing, typed commands
- `target` / `device` - register/status decode; per-device flash+RAM geometry + SDID map
- `ramflash` / `flash` - RAM flash-routine engine; BDM byte access
- `hwbreak` / `gdb` / `gdbstub` - hardware breakpoints; GDB RSP dispatch + framing
- `hexfile` / `hexdump` / `tty` - S-record/IHEX codec; output formatting
- `hcs08` / `clock` - HCS08 flash bring-up (reset, COP, ICS clock, FCDIV); oscillator
  trim search
- `sim` - software virtual dongle+target (`--sim`)
- `cpu08` / `cfv1` / `cpu12` - functional CPU + flash-module emulators
- `main` - CLI + GDB TCP server
- `vendor/` - pinned upstream: libusb, `Commands.h`, flash-routine blobs, device XML

## License

GPL-2.0-or-later. See [`LICENSE`](LICENSE) and
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md).
