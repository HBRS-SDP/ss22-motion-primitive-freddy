### [Overleaf presentation link](https://www.overleaf.com/project/6261b1d80143364855e58040)


### Previous semester SDP git repo: 

Link: https://github.com/HBRS-SDP/ws21-kelo-500-motion-control

### To-do:

1. [ ]change values for rectangle robot configuration (4 active wheels) in `PlatformToWheelInverseKinematicsSolver.c` 


### How to get active wheels (slaves) indexes:

Go to `SOEM/build/test/linux/slaveinfo` directory and run `sudo ./slaveinfo <your-ethernet-port-id>` (Note: the slaveinfo file should be executable)

[FYI: get the ethernet port id by running `sudo ifconfig`]

Example: `sudo ./slaveinfo enp5s0`

--> Look at the output and find the "**Name**" field with "**KELOD105**" in it.
Notedown the index of the "**Slave**" field.

--> Example: for the given sample output, the indexes are **{3, 5, 7, 9}**.

##### Sample output:

```
SOEM (Simple Open EtherCAT Master)
Slaveinfo
Starting slaveinfo
ec_init on enp5s0 succeeded.
9 slaves found and configured.
Calculated workcounter 15

Slave:1
 Name:KELO_ROBILE
 Output size: 96bits
 Input size: 544bits
 State: 4
 Delay: 0[ns]
 Has DC: 1
 DCParentport:0
 Activeports:1.1.0.0
 Configured address: 1001
 Man: 004c4f43 ID: 02100101 Rev: 00000001
 SM0 A:1000 L: 128 F:00010026 Type:1
 SM1 A:1080 L: 128 F:00010022 Type:2
 SM2 A:1100 L:  12 F:00010064 Type:3
 SM3 A:1400 L:  68 F:00010020 Type:4
 FMMU0 Ls:00000000 Ll:  12 Lsb:0 Leb:7 Ps:1100 Psb:0 Ty:02 Act:01
 FMMU1 Ls:0000009c Ll:  68 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 128 rd: 128 MBX protocols : 0c
 CoE details: 23 FoE details: 01 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:2
 Name:KELORH53
 Output size: 0bits
 Input size: 0bits
 State: 4
 Delay: 680[ns]
 Has DC: 1
 DCParentport:1
 Activeports:1.1.1.0
 Configured address: 1002
 Man: 004c4f43 ID: 45485542 Rev: 00000001
 FMMUfunc 0:0 1:0 2:0 3:0
 MBX length wr: 0 rd: 0 MBX protocols : 00
 CoE details: 00 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:3
 Name:KELOD105
 Output size: 288bits
 Input size: 1376bits
 State: 4
 Delay: 1380[ns]
 Has DC: 1
 DCParentport:1
 Activeports:1.0.0.0
 Configured address: 1003
 Man: 00000500 ID: 02001001 Rev: 000003fc
 SM0 A:1000 L: 512 F:00010026 Type:1
 SM1 A:1200 L: 512 F:00010022 Type:2
 SM2 A:1400 L:  36 F:00010024 Type:3
 SM3 A:1500 L: 172 F:00010020 Type:4
 FMMU0 Ls:0000000c Ll:  36 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:02 Act:01
 FMMU1 Ls:000000e0 Ll: 172 Lsb:0 Leb:7 Ps:1500 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 512 rd: 512 MBX protocols : 0c
 CoE details: 13 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:4
 Name:KELORH53
 Output size: 0bits
 Input size: 0bits
 State: 4
 Delay: 2740[ns]
 Has DC: 1
 DCParentport:2
 Activeports:1.1.1.0
 Configured address: 1004
 Man: 004c4f43 ID: 45485542 Rev: 00000001
 FMMUfunc 0:0 1:0 2:0 3:0
 MBX length wr: 0 rd: 0 MBX protocols : 00
 CoE details: 00 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:5
 Name:KELOD105
 Output size: 288bits
 Input size: 1376bits
 State: 4
 Delay: 3420[ns]
 Has DC: 1
 DCParentport:1
 Activeports:1.0.0.0
 Configured address: 1005
 Man: 00000500 ID: 02001001 Rev: 000003fc
 SM0 A:1000 L: 512 F:00010026 Type:1
 SM1 A:1200 L: 512 F:00010022 Type:2
 SM2 A:1400 L:  36 F:00010024 Type:3
 SM3 A:1500 L: 172 F:00010020 Type:4
 FMMU0 Ls:00000030 Ll:  36 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:02 Act:01
 FMMU1 Ls:0000018c Ll: 172 Lsb:0 Leb:7 Ps:1500 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 512 rd: 512 MBX protocols : 0c
 CoE details: 13 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:6
 Name:KELORH53
 Output size: 0bits
 Input size: 0bits
 State: 4
 Delay: 4760[ns]
 Has DC: 1
 DCParentport:2
 Activeports:1.1.1.0
 Configured address: 1006
 Man: 004c4f43 ID: 45485542 Rev: 00000001
 FMMUfunc 0:0 1:0 2:0 3:0
 MBX length wr: 0 rd: 0 MBX protocols : 00
 CoE details: 00 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:7
 Name:KELOD105
 Output size: 288bits
 Input size: 1376bits
 State: 4
 Delay: 5460[ns]
 Has DC: 1
 DCParentport:1
 Activeports:1.0.0.0
 Configured address: 1007
 Man: 00000500 ID: 02001001 Rev: 000003fc
 SM0 A:1000 L: 512 F:00010026 Type:1
 SM1 A:1200 L: 512 F:00010022 Type:2
 SM2 A:1400 L:  36 F:00010024 Type:3
 SM3 A:1500 L: 172 F:00010020 Type:4
 FMMU0 Ls:00000054 Ll:  36 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000238 Ll: 172 Lsb:0 Leb:7 Ps:1500 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 512 rd: 512 MBX protocols : 0c
 CoE details: 13 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:8
 Name:KELORH53
 Output size: 0bits
 Input size: 0bits
 State: 4
 Delay: 6820[ns]
 Has DC: 1
 DCParentport:2
 Activeports:1.1.0.0
 Configured address: 1008
 Man: 004c4f43 ID: 45485542 Rev: 00000001
 FMMUfunc 0:0 1:0 2:0 3:0
 MBX length wr: 0 rd: 0 MBX protocols : 00
 CoE details: 00 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0

Slave:9
 Name:KELOD105
 Output size: 288bits
 Input size: 1376bits
 State: 4
 Delay: 7520[ns]
 Has DC: 1
 DCParentport:1
 Activeports:1.0.0.0
 Configured address: 1009
 Man: 00000500 ID: 02001001 Rev: 000003fc
 SM0 A:1000 L: 512 F:00010026 Type:1
 SM1 A:1200 L: 512 F:00010022 Type:2
 SM2 A:1400 L:  36 F:00010024 Type:3
 SM3 A:1500 L: 172 F:00010020 Type:4
 FMMU0 Ls:00000078 Ll:  36 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:02 Act:01
 FMMU1 Ls:000002e4 Ll: 172 Lsb:0 Leb:7 Ps:1500 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 512 rd: 512 MBX protocols : 0c
 CoE details: 13 FoE details: 00 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
End slaveinfo, close socket
End program
```


# Bringing up Freddy

`mkdir freddy`

`cd freddy`

`git clone https://github.com/rosym-project/robif2b.git`

`git clone https://github.com/OpenEtherCATsociety/SOEM.git`

`cd SOEM/`

`mkdir build`

`mkdir install`

`cd build/`

`cmake -DCMAKE_INSTALL_PREFIX=~/freddy/install ..`

`make`

`make install`

`cd .. && cd robif2b`

`mkdir build`

`cmake -DCMAKE_INSTALL_PREFIX=~/sdp/install -DENABLE_ETHERCAT=ON -DENABLE_KELO=ON ..`


#### How to build `robif2b` :

 - `cmake -DCMAKE_INSTALL_PREFIX=/home/kvnptl/work/sdp_ws/install ..`
 - `cmake -DCMAKE_C_FLAGS="-I/home/kvnptl/work/sdp_ws/install/include" -DENABLE_ETHERCAT=ON -DENABLE_KELO=ON ..`

### NOTE: WIP To be updated 
