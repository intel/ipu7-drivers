External (_SB.PC00.IPU0, DeviceObj)

Scope (\_SB.PC00.IPU0)
{
    Name (_DSD, Package () /* method */
    {
        ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b"), /* Hierarchical Data Extension */,
        Package () {
            Package () { "mipi-img-port-0", "PRT0" },// mipi port 0 to DES0 
            Package () { "mipi-img-port-2", "PRT2" },// mipi port 2 to DES1 
        },
    })
    Name (PRT0, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () { 
            Package () { "mipi-img-data-lanes", Package () { 1, 2, 3, 4 } }
        },
    })
    Name (PRT2, Package()
    {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () { 
            Package () { "mipi-img-data-lanes", Package () { 1, 2, 3, 4 } }
        },
    })    
}
