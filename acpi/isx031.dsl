DefinitionBlock ("", "SSDT", 2, "", "IMG_PTL", 0x20250920)
{
    External (_SB.PC00, DeviceObj)

    Include ("_ipu.dsl")

    Scope (\_SB.PC00)
    {
        Include ("_max96724.dsl")

        Device (SER4)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1138")
            }
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 1, "\\_SB.PC00.DES1", 0,,,) // type 0 for DPHY, DES1 local port 1 remote port 0
                I2cSerialBusV2 (0x40, ControllerInitiated, 0x00061A80,  // actual address
                        AddressingMode7Bit, "\\_SB.PC00.DES1",
                        0x00, ResourceConsumer,, Exclusive)
                GpioIo (
                    Exclusive,                  // Not shared
                    PullNone,                   // No need for pulls
                    0,                          // Debounce timeout
                    0,                          // Drive strength
                    IoRestrictionOutputOnly,    // Only used as output
                    "\\_SB.PC00.SER4",          // GPIO controller
                    0)                          // Must be 0
                {
                    0,                         // Pin 0
                }
            })
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.DES1
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "channel", 0 }, // channel 0
                    Package () { "i2c-alias-pool",  Package () { 0x54, 0x55 } }, // alias for cam0
                    Package () { "i2c-gate", 1 },
                    Package () { "mux-adapter", 0x30 },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to sensor
                    Package () { "mipi-img-port-1", "PRT1" }  // to des
                }
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                },
            })
            Name (PRT1, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (SER5)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1138")
            }
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 1, "\\_SB.PC00.DES1", 1,,,) // type 0 for DPHY, DES1 local port 1 remote port 1
                I2cSerialBusV2 (0x40, ControllerInitiated, 0x00061A80,  // actual address
                        AddressingMode7Bit, "\\_SB.PC00.DES1",
                        0x00, ResourceConsumer,, Exclusive)
                GpioIo (
                    Exclusive,                  // Not shared
                    PullNone,                   // No need for pulls
                    0,                          // Debounce timeout
                    0,                          // Drive strength
                    IoRestrictionOutputOnly,    // Only used as output
                    "\\_SB.PC00.SER5",          // GPIO controller
                    0)                          // Must be 0
                {
                    0,                         // Pin 0
                }
            })
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.DES1
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "channel", 1 },
                    Package () { "i2c-alias-pool",  Package () { 0x54, 0x55 } }, // alias for cam0
                    Package () { "i2c-gate", 1 },
                    Package () { "mux-adapter", 0x31 },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to sensor
                    Package () { "mipi-img-port-1", "PRT1" }  // to des
                }
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                },
            })
            Name (PRT1, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (SER6)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1138")
            }
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 1, "\\_SB.PC00.DES1", 2,,,) // type 0 for DPHY, DES1 local port 1 remote port 2
                I2cSerialBusV2 (0x40, ControllerInitiated, 0x00061A80,  // actual address
                        AddressingMode7Bit, "\\_SB.PC00.DES1",
                        0x00, ResourceConsumer,, Exclusive)
                GpioIo (
                    Exclusive,                  // Not shared
                    PullNone,                   // No need for pulls
                    0,                          // Debounce timeout
                    0,                          // Drive strength
                    IoRestrictionOutputOnly,    // Only used as output
                    "\\_SB.PC00.SER6",          // GPIO controller
                    0)                          // Must be 0
                {
                    0,                         // Pin 0
                }
            })
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.DES1
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "channel", 2 },
                    Package () { "i2c-alias-pool",  Package () { 0x54, 0x55 } }, // alias for cam0
                    Package () { "i2c-gate", 1 },
                    Package () { "mux-adapter", 0x32 },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to sensor
                    Package () { "mipi-img-port-1", "PRT1" }  // to des
                }
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                },
            })
            Name (PRT1, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (SER7)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1138")
            }
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 1, "\\_SB.PC00.DES1", 3,,,) // type 0 for DPHY, DES1 local port 1 remote port 3
                I2cSerialBusV2 (0x40, ControllerInitiated, 0x00061A80,  // actual address
                        AddressingMode7Bit, "\\_SB.PC00.DES1",
                        0x00, ResourceConsumer,, Exclusive)
                GpioIo (
                    Exclusive,                  // Not shared
                    PullNone,                   // No need for pulls
                    0,                          // Debounce timeout
                    0,                          // Drive strength
                    IoRestrictionOutputOnly,    // Only used as output
                    "\\_SB.PC00.SER7",          // GPIO controller
                    0)                          // Must be 0
                {
                    0,                         // Pin 0
                }
            })
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.DES1
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "channel", 3 },
                    Package () { "i2c-alias-pool",  Package () { 0x54, 0x55 } }, // alias for cam0
                    Package () { "i2c-gate", 1 },
                    Package () { "mux-adapter", 0x33 },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to sensor
                    Package () { "mipi-img-port-1", "PRT1" }  // to des
                }
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                },
            })
            Name (PRT1, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (CAM4)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1031") // ISX031
            }
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.SER4
            })
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 0, "\\_SB.PC00.SER4", 0,,,) // type 0 for DPHY, SER4 local port 0 remote port 0

                I2cSerialBusV2 (0x001A, ControllerInitiated, 0x00061A80, // actual address
                        AddressingMode7Bit, "\\_SB.PC00.SER4",
                        0x00, ResourceConsumer,, Exclusive)
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {

                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "mipi-img-clock-frequency", 96000000 }, // 9.6 MHz
                    Package () { "channel", Zero }, // channel 0
                    Package () { "reset-gpios", Package () { ^SER4, 0, 0, 0 } },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to ser
                },
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (CAM5)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1031") // ISX031
            }
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.SER5
            })
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 0, "\\_SB.PC00.SER5", 0,,,) // type 0 for DPHY, SER5 local port 0 remote port 0

                I2cSerialBusV2 (0x001A, ControllerInitiated, 0x00061A80, // actual address
                        AddressingMode7Bit, "\\_SB.PC00.SER5",
                        0x00, ResourceConsumer,, Exclusive)
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {

                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "mipi-img-clock-frequency", 96000000 }, // 9.6 MHz
                    Package () { "channel", Zero }, // channel 0
                    Package () { "reset-gpios", Package () { ^SER5, 0, 0, 0 } },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to ser
                },
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (CAM6)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1031") // ISX031
            }
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.SER6
            })
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 0, "\\_SB.PC00.SER6", 0,,,) // type 0 for DPHY, SER6 local port 0 remote port 0

                I2cSerialBusV2 (0x001A, ControllerInitiated, 0x00061A80, // actual address
                        AddressingMode7Bit, "\\_SB.PC00.SER6",
                        0x00, ResourceConsumer,, Exclusive)
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {

                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "mipi-img-clock-frequency", 96000000 }, // 9.6 MHz
                    Package () { "channel", Zero }, // channel 0
                    Package () { "reset-gpios", Package () { ^SER6, 0, 0, 0 } },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to ser
                },
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }

        Device (CAM7)
        {
            Method (_STA, 0, NotSerialized)  // _STA: Status
            {
                Return (0x0F)
            }
            Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
            {
                Return ("INTC1031") // ISX031
            }
            Name (_DEP, Package (0x01)  // _DEP: Dependencies
            {
                \_SB.PC00.SER7
            })
            Name(_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                CSI2Bus(DeviceInitiated, 1, 0, "\\_SB.PC00.SER7", 0,,,) // type 0 for DPHY, SER7 local port 0 remote port 0

                I2cSerialBusV2 (0x001A, ControllerInitiated, 0x00061A80, // actual address
                        AddressingMode7Bit, "\\_SB.PC00.SER7",
                        0x00, ResourceConsumer,, Exclusive)
            })
            Name (_DSD, Package ()  // _DSD: Device-Specific Data
            {

                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
                Package ()
                {
                    Package () { "mipi-img-clock-frequency", 96000000 }, // 9.6 MHz
                    Package () { "channel", Zero }, // channel 0
                    Package () { "reset-gpios", Package () { ^SER7, 0, 0, 0 } },
                },
                ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
                Package ()
                {
                    Package () { "mipi-img-port-0", "PRT0" }, // to ser
                },
            })

            Name (PRT0, Package()
            {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package ()
                {
                    Package () { "mipi-img-clock-lanes", 0 },
                    Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
                    Package () { "mipi-img-link-frequencies", Package() { 600000000 } }, // 600 MHz
                },
            })
        }
    }
}