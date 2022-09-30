<div align="center">
  <h1 align="center">Motion Primitive for Robile (Freddy)</h1>
  <h3 align="center">
    Ramping behaviour of the robile platform using motion primitives.
  </h3>
</div>

The aim of the project is performing a controlled movement over the ramp using motion primitives for the robile platform robot. Based on previous semester SDP (WS2021) [Kelo 500 motion control](https://github.com/HBRS-SDP/ws21-kelo-500-motion-control) which uses force control.

[<div align="center"><img width="460" src="media/images/Robile.png"></div>](https://griffig.xyz)

<p align="center">
  You can find videos regarding the project <a href="https://drive.google.com/drive/folders/1rVMj4RdJPmCHwqase1NZHOby6o1TtAJh?usp=sharing">here</a>!
</p>

<p align="center">
  Project milestone deliverables can be found <a href="https://drive.google.com/drive/u/3/folders/1boct6apoQLtNgNKWaS_PBlGO9Hr4Donh">here</a>!
</p>

## Ramping behaviour output:

![test](media/images/ramp.gif)


## Getting started

### Required library

- Simple Open EtherCAT Master (SOEM) - communication between robot and the actuators. [SOEM](https://github.com/OpenEtherCATsociety/SOEM)
- robif2b - robot control interface [robif2b](https://github.com/rosym-project/robif2b)
- GSL - GNU Scientific Library [GSL](https://github.com/ampl/gsl)
- WS21 SDP repository: Motion Control of the KELO 500 [Kelo 500 motion control](https://github.com/HBRS-SDP/ws21-kelo-500-motion-control)

### Building SOEM library

```bash
git clone https://github.com/OpenEtherCATsociety/SOEM
```

Replace `STATIC` with `SHARED` on line 72 in CMakeList.txt, it should look like the code block below,

```
add_library(soem SHARED
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})
```

Continue building steps:

```
mkdir install 
cd SOEM
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<path to created install folder>/install ..
make install
```

### Building WS21 SDP library

```bash
git clone https://github.com/HBRS-SDP/ws21-kelo-500-motion-control
cd ws21-kelo-500-motion-control/KELO_SDP
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<path to created install folder>/install ..
cmake -DCMAKE_C_FLAGS="-I<path to created install folder>/install/include" -DENABLE_PACKAGE_REGISTRY=on -DENABLE_ETHERCAT=ON ..
make install
```

### How to get active wheels (slaves) indexes

```bash
cd SOEM/build/test/linux/slaveinfo
sudo ./slaveinfo <your-ethernet-port-id>
```

[FYI: get the ethernet port id by running `ifconfig` or `ip a`]

## Building the code
```
git clone --recursive https://github.com/HBRS-SDP/ss22-motion-primitive-freddy.git
mkdir build && cd build
cmake ..
make
```

### Building robif2b library

```bash
cd robif2b
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<path to created install folder>/install ..
cmake -DCMAKE_C_FLAGS="-I<path to created install folder>/install/include" -DENABLE_PACKAGE_REGISTRY=on -DENABLE_ETHERCAT=ON -DENABLE_KELO=ON ..
make install
```

## Running the code
```bash
(from outside of cloned repo)
cd ss22-motion-primitive-freddy/build
sudo ./us5 <wheel_alignment_direction> # where wheel_alignment_direction is 0, 90, 180, or 270

# or run individual user stories, for example 
sudo ./us2 90 # to align wheels in 90 degrees (+Y direction)

```

## How to make any library as a package:

- Create a CMakeLists.txt file. Main features of the file are: 
  - **find_package** finds for the file or library mentioned in the closed parenthesis.<br> **Example:** `find_package(robif2b REQUIRED)`
  - **add_executable** adds the executable target <name> needs to be built from the listed source files. <br>
**Example:** `add_executable(us2 src/kelo_robile_example_us2.c)`
  - **target_link_libraries** takes the target and adds dependency. <br> **Example:** `target_link_libraries(us2 robif2b::kelo m)`


## User stories / TODO

- [x] Running previous semester SDP (Force distribution).
- [x] Orientation of wheel units to disired configuration.
- [x] Align the robot with ramp base.
- [x] Implement ramp-up behaviour.
- [x] Integrate sub-modules as a complete state machine.
