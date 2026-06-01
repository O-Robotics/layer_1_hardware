# WitMotion Standard Protocol Reference

This package primarily targets the JY901 and includes the older bundled PDF:

- `Witmotion JY901 Datasheet.pdf`

For newer register-read and SDK-oriented protocol behavior, the official WitMotion references used during driver review are:

- WIT Standard Communication Protocol
  - https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol
- WIT SDK landing page
  - https://wit-motion.gitbook.io/witmotion-sdk
- WitStandardProtocol_JY901 GitHub repository
  - https://github.com/WITMOTION/WitStandardProtocol_JY901

Key protocol details confirmed from the newer official documentation:

- All settings should unlock first with:
  - `FF AA 69 88 B5`
- Register read command:
  - `FF AA 27 REG 00`
- Register read reply:
  - `55 5F REG1L REG1H REG2L REG2H REG3L REG3H REG4L REG4H SUM`
- The read command returns 4 consecutive 16-bit registers starting at `REG`.

Relevant registers for `amr_sweeper_imu`:

- `0x02` `RSW` output content
- `0x03` `RRATE` output rate
- `0x04` `BAUD` serial baud rate
- `0x1B` `LEDOFF` LED control
- `0x23` `ORIENT` installation direction
- `0x24` `AXIS6` algorithm selection
- `0x27` `READADDR` register read command
- `0x63` is used by the bundled JY901 datasheet section `7.2.7` as the gyroscope automatic calibration command. It should not be assumed to be a normal readable persistent configuration register in this package.

Notes:

- The bundled JY901 PDF and the newer online WIT protocol docs are not identical. The local PDF remains the source of truth for the original JY901 write-side configuration sections used by this package.
- The newer online protocol documentation is the source used for the `0x27` register-read command and `0x5F` reply format.
- When behavior differs between the two documents, validate against the actual device on hardware before assuming a configuration write is effective.
