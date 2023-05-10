#!../../bin/linux-x86_64/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# Configure the device
ek9000Configure("EK9K1", "192.168.1.185", 502, 8)

ek9000ConfigureTerminal("EK9K1", "t1", "EL2008", 1)
ek9000ConfigureTerminal("EK9K1", "t2", "EL1004", 2)
ek9000ConfigureTerminal("EK9K1", "t3", "EL3314", 3)
ek9000ConfigureTerminal("EK9K1", "t4", "EL3064", 4)
ek9000ConfigureTerminal("EK9K1", "t5", "EL5042", 5)
ek9000ConfigureTerminal("EK9K1", "t6", "EL2008", 6)
ek9000ConfigureTerminal("EK9K1", "t7", "EL4004", 7)
el70x7Configure("EK9K1","EK9K1_MOTOR_PORT","m1", 8, "EL7041")

dbLoadRecords("../motor/db/motorUtil.db", "P=IOC:m1")

cd "${TOP}/iocBoot/${IOC}"

dbLoadRecords("../../db/EL2008.template", "TERMINAL=t1")
dbLoadRecords("../../db/EL1004.template", "TERMINAL=t2")
dbLoadRecords("../../db/EL3314.template", "TERMINAL=t3")
dbLoadRecords("../../db/EL3064.template", "TERMINAL=t4")
dbLoadRecords("../../db/EL5042.template", "TERMINAL=t5")
dbLoadRecords("../../db/EL2008.template", "TERMINAL=t6")
dbLoadRecords("../../db/EL4004.template", "TERMINAL=t7")

dbLoadTemplate("motor.substitutions")

iocInit

#el70x7SetParam EK9K1_MOTOR_PORT maximal-current 100
