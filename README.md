# VeTeF - Vehicle Testing Framework
This is a framework (which is still in development phase) for testing autonomous vehicle control algorithms in simulation environments.

This project is maintained by Cumhur Erkan TUNCALI who is a PhD student in Computer Engineering at Arizona State University.

**Supported Platforms:**
* Webots (tested with 8.6.0) 
_While currently Webots is the only supported platform, with proper modifications, it is possible to extend the support to other simulation environments._

**Structure:**
* **controllers** folder contains the Webots controllers.
    * **webots_vehicle_testing** module in the webots_vehicle_testing folder is the **Simulation Controller**, i.e., the supervisor controller which controls the simulation execution. It is also responsible for communication with a Simulation Configurator.
    * **vehicle_controller** module is the main vehicle controller. It acts as a wrapper and loads the actual controller which is given to it as a parameter.
    * The other controllers can either be passed to vehicle_controller as an argument, or they can be used as standalone controllers (being standalone controller is not yet tested for most controllers, but it is easy to modify them to be standalone).
* **simulation_configurator** folder contains the Simulation Configurator which is running outside the simulator. Simulation Configurator is responsible for starting simulator, sending simulation environment, commands and simulation specific requests to the **Simulation Controller** over TCP/IP.
    * **examples** folder contains the example simulations. These are the entry points to run a simulation. They use an instance of simulation_configurator to set up a specific simulation environment and collect the requested data. 
* **protos** folder contains user modified vehicle PROTO files for Webots (or user defined vehicle/object PROTOs). These modifications can be added extra sensor locations etc.
* **worlds** folder contains example Webots worlds to load when starting a simulation. An empty world can be used here, and the requested vehicles and roads can be added by communication Simulation Configurator with Simulation Controller.
* **matlab_interfaces** folder is just some examples tailored to run directly from MATLAB. I use these scripts to simplify my personal work in MATLAB, and not essential for using the framework.
