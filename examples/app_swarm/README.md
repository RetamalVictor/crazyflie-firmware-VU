# VU Swarm for Crazyflie
Application with Crayflie app-Layer.

------
#### This directory is directly related to the following paper:
### Onboard Controller Design for Nano UAV Swarm in Operator-Guided Collective Behaviors*

**Currently accepted to ICRA 2023**

### Citation:
```
Coming soon
```
---
REQUIREMENTS
------------

Installation of [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm).

Same requirements as Crazyflie firmware

## Installation
- clone the repository
```bash
git clone https://github.com/RetamalVictor/crazyflie-firmware-VU.git
```

## Build

Make sure that you are in the app_swarm (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```
