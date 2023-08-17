==============
Ccommissioning
==============

Setting up a rail
=================

IOC Configuration
=================

The following iocsh command may be used to configure an ek9000 device:
.. code::
    
    ek9000Configure(deviceName, ip, port, temrinal_count)

`deviceName` will be used to refer to the device later. 

`terminal_count` defines how many terminals will be assocated with the device.

For each terminal, the following iocsh command must be used to configure it:
.. code::

    ek9000ConfigureTerminal

