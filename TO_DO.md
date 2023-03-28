# Development of Relative localization with UWB deck

## Current Roadmap

1. Peer to Peer UWB communication with Bidirectional TWR [DONE-PHASE.1.0]
    [PHASE.1.0] - Ranging P2P and distance exchange only
    [PHASE.2.0] - Add message information to the communication.
        2.1 Include the structs needed to create the messages
        2.2 Extract information about itself from the Kalman estimations to populate the message.           (tag [TAG-2.1.2])
        2.3 Define ``twrGetSwarmInfo`` in the driver file                                                   (tag [TAG-2.1.1])
            The information is stored in a struct called ``swarmInfo_t``

    [EXTERNALS]

    [EXTRA-TASK]
        1.1 Outlier filtering. (Currently Median filter) (Ideas, Neural Network filter)
        1.2 Sender/Receiver Swapping method. (Now hardcoded for 3 drones)

2. Relative Localization task creation [IN-PROGRESS]
    2.1 Implement main file to create the task in system ``relative_localization.c`` & ``relative_localization.h``
        2.1.1 Define and create ``twrGetSwarmInfo`` in the driver o UWB                                     (tag [TAG-2.1.1])
            This will allow the Localization task to access the information shared with the UWB
        2.1.2 Define ``estimatorKalmanGetSwarmInfo`` in the ``estimator.c`` and ``estimator.h`` files.      (tag [TAG-2.1.2])
            This returns the information of the individual.
        2.1.3 Define ``RelativeEKF`` to compute the state estimation for peers                              (tag [TAG-2.1.3])
        

3. Kalman Filter [IN-PROGRESS]
    1.1 Define ``estimatorKalmanGetSwarmInfo`` in the ``estimator.c`` and ``estimator.h`` files.            (tag [TAG-2.1.2])
    1.2 Relative Estimation Kalman ``relativeEHF``                                                          (tag [TAG-2.1.3])
