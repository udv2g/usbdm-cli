# USBDM flash routines

- [Source tree @ 4f18e1b](https://github.com/podonoghue/usbdm-flash-routines/tree/4f18e1bf67138d5b23927258a31590cbda247863)
- License: GPL
- `HCS12-XMMC-XFTMRK-flash-program.s19` is from the USBDM 4.12 package
  ([usbdm-eclipse-makefiles-build](https://github.com/podonoghue/usbdm-eclipse-makefiles-build)
  `PackageFiles/DeviceData/HCS12/`), the same place the FTS and GMMC blobs here
  match byte for byte.
- The S08P routines (`HCS08-PAxx-small-flash-program.s19`, `HCS08-PTxx-flash-program.s19`,
  `HCS08-PBxx-flash-program.s19`) are from the same package at
  [4ffd8b2](https://github.com/podonoghue/usbdm-eclipse-makefiles-build/tree/4ffd8b230b540b832be4bf52224f5b5db5f8efbf/PackageFiles/DeviceData/HCS08),
  the commit the device XML is pinned to, with CR line endings stripped. They match the
  `GeneratedFiles/HCS08/` builds in the source tree above (PBxx is not built there) and
  the files a USBDM 4.12.1.262 install ships. Sources: `HCS08-PAxx-flash-program/Sources/main.asm`
  (PA2/PA4, small code), `HCS08-PTxx-flash-program/Sources/main.c` and
  `HCS08-PBxx-flash-program/Sources/main.c` (large code, HCS08 header layout).
