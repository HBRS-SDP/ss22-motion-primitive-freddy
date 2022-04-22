### Previous semester SDP git repo: 

Link: https://github.com/HBRS-SDP/ws21-kelo-500-motion-control

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

### NOTE: WIP To be updated 
