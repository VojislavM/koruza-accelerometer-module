#TLV communication

Koruza move driver is using the UART for communicating with the rest of the Koruza unit. A chosen protocol is based on TLVs, which makes it extensible in a backwards-compatible way. Each protocol message looks like this:
```
[TLV#1] [TLV#2] ...
```

And each TLV looks like this:
```
[type: 1 byte] [length: 2 bytes] [value: length bytes]
```

All protocol values are in network byte order (big endian). The use of TLVs enables older protocol implementations to skip newer TLVs.

Commands supported by the command TLV.

* COMMAND_GET_STATUS
  * return motor position
* COMMAND_MOVE_MOTOR
  * send new motor position x and y coordinate
* COMMAND_HOMING
  * send command for homing routine
* COMMAND_CALIBRATION
  * do the calibration routine

