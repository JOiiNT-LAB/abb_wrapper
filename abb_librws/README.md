# abb_librws

## Important Notes

RobotWare versions `6.x` are currently incompatible with *abb_librws* (due to RWS `1.0` being replaced by RWS `2.0`). Check [this release note](https://robotapps.blob.core.windows.net/apps/ReleaseNotesRWS2019.3.zip) for more information about the different RWS versions.

Please note that this package has not been productized, it is provided "as-is" and only limited support can be expected.

## Overview

A C++ library for interfacing with ABB robot controllers supporting *Robot Web Services* (RWS). 
See the online [documentation](https://developercenter.robotstudio.com/api/RWS) for a detailed description of what RWS is and how to use it.

### Sketch

The following is a conceptual sketch of how this RWS library can be viewed, in relation to an ABB robot controller as well as the EGM companion library mentioned above. The optional *StateMachine Add-In* is related to the robot controller's RAPID program and system configuration.

![RWS sketch](docs/images/rws_sketch.png)

### Requirements

* RobotWare version `7.x`.

### Dependencies

* [POCO C++ Libraries](https://pocoproject.org) (`>= 1.4.3` due to WebSocket support)

### Limitations

RWS provides access to several services and resources in the robot controller, and this library currently support the following:

* Reading/writing of IO-signals.
* Reading/writing of RAPID data.
* Reading of RAPID data properties.
*	Starting/stopping/resetting the RAPID program.
*	Subscriptions (i.e. receiving notifications when resources are updated).
*	Uploading/downloading/removing files.
*	Checking controller state (e.g. motors on/off, auto/manual mode and RAPID execution running/stopped).
*	Reading the Joint/Cartesian values of a mechanical unit.
*	Register as a local/remote user (e.g. for interaction during manual mode).
*	Turning the motors on/off.
*	Reading of current RobotWare version and available tasks in the robot system.
*	Enable/disable lead-through.
*	Access to SmartGripper functionality.

### Recommendations

* This library has been verified to work with RobotWare `7.3.1`. Other versions are expected to work, but this cannot be guaranteed at the moment.
* It is a good idea to perform RobotStudio simulations before working with a real robot.
* It is prudent to familiarize oneself with general safety regulations (e.g. described in ABB manuals).
* Consider cyber security aspects, before connecting robot controllers to networks.

## Usage Hints

This is a generic library, which can be used together with any RAPID program and system configuration. The library's primary classes are:

* [POCOClient](include/abb_librws/rws_poco_client.h): Sets up and manages HTTP and WebSocket communication and is unaware of the RWS protocol.
* [RWSClient](include/abb_librws/rws_client.h): Inherits from `POCOClient` and provides interaction methods for using the RWS services and resources.
* [RWSInterface](include/abb_librws/rws_interface.h): Encapsulates an `RWSClient` instance and provides more user-friendly methods for using the RWS services and resources.
* [RWSStateMachineInterface](include/abb_librws/rws_state_machine_interface.h): Inherits from `RWSInterface` and has been designed to interact with the aforementioned *StateMachine Add-In*. The interface knows about the custom RAPID variables and routines, as well as system configurations, loaded by the RobotWare Add-In.

The optional *StateMachine Add-In* for RobotWare can be used in combination with any of the classes above, but it works especially well with the `RWSStateMachineInterface` class.

### StateMachine Add-In [Optional]

The purpose of the RobotWare Add-In is to *ease the setup* of ABB robot controllers. It is made for both *real controllers* and *virtual controllers* (simulated in RobotStudio). If the Add-In is selected during a RobotWare system installation, then the Add-In will load several RAPID modules and system configurations based on the system specifications (e.g. number of robots and present options).

The RAPID modules and configurations constitute a customizable, but ready to run, RAPID program which contains a state machine implementation. Each motion task in the robot system receives its own state machine instance, and the intention is to use this in combination with external systems that require interaction with the robot(s). The following is a conceptual sketch of the RAPID program's execution flow.

<p align="center">
  <img src="docs/images/statemachine_addin_sketch.png" width="500">
</p>

To install the Add-In:

1. Go to the *Add-Ins* tab in RobotStudio.
2. Search for *StateMachine Add-In* in the *RobotApps* window.
3. Select the Add-In and retrieve the Add-In by pressing the *Add* button.
4. Verify that the Add-In was added to the list *Installed Packages*.
5. The Add-In should appear as an option during the installation of a RobotWare system.

See the Add-In's [user manual](https://robotapps.blob.core.windows.net/appreferences/docs/2093c0e8-d469-4188-bdd2-ca42e27cba5cUserManual.pdf) for more details, as well as for install instructions for RobotWare systems. The manual can also be accessed by right-clicking on the Add-In in the *Installed Packages* list and selecting *Documentation*.

## Acknowledgements

This work is based on the [abb_librws](https://github.com/ros-industrial/abb_librws) classes developed by Jon Tjerngren for ABB IRC5 controllers (running RobotWare `6.x`)
