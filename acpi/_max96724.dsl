Device (DES1)
{
    Name (_UID, Zero)  // _UID: Unique ID
    Method (_HID, 0, NotSerialized)  // _HID: Hardware ID
    {
        Return ("INTC1139")
    }
    Method (_CID, 0, NotSerialized)  // _CID: Compatible ID
    {
        Return ("INTC1139")
    }
    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
        Return (0x0F)
    }
    Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
    {
        CSI2Bus(DeviceInitiated, 0, 4, "\\_SB.PC00.IPU0", 2,,,) // type 0 for CPHY, IPU0 local port 4 remote port 2
        I2cSerialBusV2 (
            0x0027,                 // SlaveAddress
            ControllerInitiated,    // SlaveMode
            0x00061A80,             // ConnectionSpeed
            AddressingMode7Bit,     // AddressingMode
            "\\_SB.PC00.I2C2",      // ResourceSource
            0x00,                   // ResourceSourceIndex
            ResourceConsumer,       // ResourceUsage
            ,                      // DescriptorName
            Exclusive,              // ShareType    
            )
    })

    Name (_DEP, Package ()  // _DEP: Dependencies
    {
        \_SB.PC00.IPU0
    })

    Name (_DSD, Package ()
    {
        ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), /* Device Properties for _DSD */,
        Package ()
        {
            Package () { "i2c-alias-pool",  Package () { 0x44, 0x45, 0x46, 0x47 } },
            Package () { "i2c-adapter-pool", Package () { 0x20, 0x21, 0x22, 0x23 } },
        },
        ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
        Package ()
        {
            Package () { "mipi-img-port-0", "PRT0" }, // to ser4
            Package () { "mipi-img-port-1", "PRT1" }, // to ser5
            Package () { "mipi-img-port-2", "PRT2" }, // to ser6
            Package () { "mipi-img-port-3", "PRT3" }, // to ser7
            Package () { "mipi-img-port-4", "PRT4" }, // to ipu0
        }
    })
    Name (PRT0, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
            Package () { "mipi-img-clock-lanes", 0 },
            Package () { "mipi-img-data-lanes", Package () { 1, 2, 3, 4 } },
        },
    })

    Name (PRT1, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
            Package () { "mipi-img-clock-lanes", 0 },
            Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
        },

    })

    Name (PRT2, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
            Package () { "mipi-img-clock-lanes", 0 },
            Package () { "mipi-img-data-lanes", Package () { 1, 2, 3, 4 } },
        },
    })

    Name (PRT3, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
            Package () { "mipi-img-clock-lanes", 0 },
            Package () { "mipi-img-data-lanes", Package() { 1, 2, 3, 4 } },
        },

    })

    Name (PRT4, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
            Package () { "mipi-img-clock-lanes", 0 },
            Package () { "mipi-img-data-lanes", Package() { 1, 2 } },
            Package () { "mipi-img-link-frequencies", Package() { 1200000000 } }, // 1200 MHz
        },
    })
}
