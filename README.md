<div align="center">
  <h1 align="center">Motion Primitive for Robile (Freddy)</h1>
  <h3 align="center">
    Ramping behaviour of the robile platform using motion primitives.
  </h3>
</div>

The aim of the project is performing a controlled movement over the ramp using motion primitives for the robile platform robot. Based on previous semester SDP (WS2021) [Kelo 500 motion control](https://github.com/HBRS-SDP/ws21-kelo-500-motion-control) which uses force control.

[<div align="center"><img width="460" src="media/images/Robile.png"></div>](https://griffig.xyz)

<p align="center">
  You can find many videos of the project <a href="[https://github.com/HBRS-SDP/ss22-motion-primitive-freddy/tree/main/media/video](https://drive.google.com/drive/folders/1rVMj4RdJPmCHwqase1NZHOby6o1TtAJh?usp=sharing)">here</a>!
</p>

## Getting started

### Required library

 - Simple Open EtherCAT Master (SOEM) - communication between robot and the actuators. [SOEM](https://github.com/OpenEtherCATsociety/SOEM)
 - robif2b - robot control interface3 [robif2b](https://github.com/rosym-project/robif2b)
 - GSL - GNU Scientific Library4 [GSL](https://github.com/ampl/gsl)
 - WS21 SDP repository: Motion Control of the KELO 500 [Kelo 500 motion control](https://github.com/HBRS-SDP/ws21-kelo-500-motion-control)


### Building SOEM library 
```bash
git clone https://github.com/OpenEtherCATsociety/SOEM

mkdir build
cd build
cmake ..
make
```

### Building robif2b library 
```bash
git clone https://github.com/OpenEtherCATsociety/SOEM

mkdir build
cd build
cmake ..
make
```

### How to get active wheels (slaves) indexes:
```bash
cd SOEM/build/test/linux/slaveinfo

sudo ./slaveinfo <your-ethernet-port-id>
```
[FYI: get the ethernet port id by running ifconfig or ip a]
```bash
cd SOEM/build/test/linux/slaveinfo

sudo ./slaveinfo <your-ethernet-port-id>
```


## Running the code.

```bash
cd robif2b/build
make


sudo ./src/example/kelo_robile_example
```


## User story / TODO 

- [x] Running previous semester SDP (Force distribution).
- [x] Orientation of wheel units to disired configuration.
- [x] Align the robot with ramp base.
- [ ] Implement ramp-up behaviour.
- [ ] Integrate sub-modules as a complete state machine.
