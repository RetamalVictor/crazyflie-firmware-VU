# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)
# *Onboard Controller Design for Nano UAV Swarm in Operator-Guided Collective Behaviours *
[Flight videos of the experiments can be found on this link](https://www.youtube.com/watch?v=RA3ePt5Dhoo)


The firmaware files needed for the experiments are located in ```crazyflie-firmware-VU/examples/app_swarm```

Please follow the official bitcraze documentation to build and flash new firmware in the crazyflie. 
More information about the app layer can be found [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/).

Once the new app is flashed in the crazyflies. Please refer to the main [mision control script](https://github.com/RetamalVictor/crazyswarm-VU) to initiate the experiments.

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag.

## Building and Flashing
See the [building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) in the github docs folder.


## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

## Generated documentation

The easiest way to generate the API documentation is to use the [toolbelt](https://github.com/bitcraze/toolbelt)

```tb build-docs```

and to view it in a web page

```tb docs```

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution

To run the tests please have a look at the [unit test documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/unit_testing/).

## License

The code is licensed under LGPL-3.0
