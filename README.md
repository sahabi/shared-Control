ROS interface to OpenBCI board

* Defines message to hold EEG channel values
* publish_measurements.py connects to the board with the open_bci_v3.py and publishes the channel values
* detect_alpha.py plots the FFT of the signal, detects alpha waves, and publishes the detection