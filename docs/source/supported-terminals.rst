=======================
Terminal Device Support
=======================

Analog Terminals
================

Beckhoff analog terminals usually have three different modes of presentation: signed, unsigned and MSB (most significant bit).
This module only supports the signed and unsigned representations, MSB is unsupported and will result in
an incorrect readback for negative values.

Thermocouple Terminals
======================

Thermocouple terminals, like the EL3314, must be set to the "Standard" mapping mode, which is the default. Setting this to "Compact"
will cause errors in the module and cause terminals to be mapped incorrectly.

Terminal parameters, such as the thermocouple type, may be configured using the web interface. 

Digital Terminals
=================

Most Beckhoff digital E-Bus terminals are supported, up to 18 channels. For input terminals, `mbbiDirect` or the `bi` records
may be used. Output terminals may use the `mbboDirect` or `bo` records. Regular `mbbi` or `mbbo` is **not** spported.

Encoder Terminals
=================

Several EL5XXX series encoder terminals are supported, using the `longin` record type.

Supported Terminals
===================

All terminals listed below are supported by this device support module. Unless explicitly stated
otherwise, terminal variants are also suppored (i.e. all variants of the EL3314 such as EL3314-0002 are supported)

* EL1001
* EL1002
* EL1004
* EL1008
* EL1012
* EL1014
* EL1018
* EL1024
* EL1034
* EL1084
* EL1088
* EL1094
* EL1098
* EL1104
* EL1114
* EL1124
* EL1134
* EL1144
* EL1184
* EL1202
* EL1382
* EL1702
* EL1712
* EL1722
* EL1804
* EL1808
* EL1809
* EL1814
* EL1819
* EL2001
* EL2002
* EL2004
* EL2008
* EL2022
* EL2024
* EL2042
* EL2084
* EL2088
* EL2124
* EL2794
* EL2808
* EL2816
* EL3001
* EL3002
* EL3004
* EL3008
* EL3012
* EL3014
* EL3021
* EL3022
* EL3024
* EL3041
* EL3042
* EL3044
* EL3048
* EL3051
* EL3052
* EL3054
* EL3058
* EL3061
* EL3062
* EL3064
* EL3068
* EL3101
* EL3102
* EL3104
* EL3111
* EL3112
* EL3114
* EL3121
* EL3122
* EL3124
* EL3141
* EL3142
* EL3144
* EL3151
* EL3152
* EL3154
* EL3161
* EL3162
* EL3164
* EL3174
* EL3202
* EL3314
* EL3312
* EL3311
* EL3681
* EL4001
* EL4002
* EL4004
* EL4008
* EL4011
* EL4012
* EL4014
* EL4018
* EL4021
* EL4022
* EL4024
* EL4028
* EL4031
* EL4032
* EL4034
* EL4038
* EL4102
* EL4104
* EL4114
* EL4112
* EL4134
* EL4132
* EL4122
* EL5001
* EL5002
* EL5042

