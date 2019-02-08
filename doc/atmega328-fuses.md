# atmega328p fuse configuration
Two fuse configuration can be written to the device.

# First Solution

!Not tested! Basically, same as "Second solution" without disabling the RESET functionality.

# Second Solution

AVRDude command
-U lfuse:w:0xe2:m -U hfuse:w:0x71:m -U efuse:w:0xfc:m

Notable configuration:
- 8MHz internal clock 6CK/14CK + 65ms
- CKDIV8: unprogrammed
- Boot Reset Vector Enabled: unprogrammed
- Preserve EEPROM: programmed
- Serial program downloading: unprogrammed
- Reset Disabled: programmed

See the atmega328-fuses-matrix document for specific information.
