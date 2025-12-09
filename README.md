# VaN3Twin

<div align="center">
<img src="img/VaN3Twin_logo_v5_blk.png" width="400">
</div>

<br/><br/>

[<img src="img/docs.png" height="40">](https://ms-van3ts-documentation.readthedocs.io/en/master/)

<br/><br/>

ns-3 modules to build and simulate ETSI-compliant VANET (V2X) applications using SUMO (v-1.6.0+) and ns-3 (ns-3-dev, version supporting the NR-V2X module by CTTC), with the possibility of easily switching stack and communication technology.

It has been tested with SUMO v1.6.0, v1.7.0, v1.8.0, v1.12.0, v1.18.0 on Ubuntu 20.04 and 22.04.
Back compatibility **is not** ensured with new versions of TraCI and Ubuntu 24.04 is **not** officially supported yet.

**Contacts, developers, project coordination**: Marco MALINVERNO [marco.malinverno1@gmail.com], Francesco RAVIGLIONE [francescorav.es483@gmail.com], Carlos Mateo RISMA CARLETTI [carlosrisma@gmail.com], Diego GASCO [diego.gasco@polito.it], Roberto PEGURRI [roberto.pegurri@mail.polimi.it], Alessandro GENOVESE [alessandro.genovese@icloud.com], Alessandro GIACCAGLINI [alessandro.giaccaglini@gmail.com], Marco RAPELLI [rapelli.m@libero.it], Francesco LINSALATA [francesco.linsalata@polimi.it], Eugenio MORO [eugenio.moro@polimi.it], Claudio CASETTI [claudio.casetti@polito.it], Carla-Fabiana CHIASSERINI [carla.chiasserini@polito.it]

This project is licensed under a GPL-2.0 License. Please see also the `LICENSE` file for more details.

## Looking for ms-van3t?
__Were you looking for <img src="img/MS-VAN3T_logo-V2_small.png" height="30"> but ended up here?__

VaN3Twin now integrates the whole ms-van3t framework, and much more, keeping the same functionalities, just under a new name, and with the addition of new functionalities like the integration with a full-fledged ray-tracer!

**Important:** you will still find the name __ms-van3t__ in many parts of the framework. We are in the process of renaming all the references and files, but this may take a bit of time.

## VaN3Twin installation

To build the project:
* Install SUMO following the guide at [https://sumo.dlr.de/wiki/Downloads](https://sumo.dlr.de/wiki/Downloads)
    * You can use 
    
    	`sudo add-apt-repository ppa:sumo/stable`  
    	`sudo apt update`  
    	`sudo apt install sumo sumo-tools sumo-doc`  
    * Be careful: in the future the previous commands will install updated version of SUMO which are not ensured to work with this scripts (that are tested with any version from **v-1.6.0** to **v-1.18.0** )
    * Test sumo by opening a terminal and running "sumo-gui".
	
    * **Possible problems**:
			
			You may get the following error when running SUMO:
			
        	"sumo-gui: symbol lookup error: /usr/lib/libgdal.so.26: undefined symbol: GEOSMakeValid_r"
    
        	To solve it, remove all the reference to GEOS inside /usr/local/lib/ (do NOT do it if you need the GEOS library):
    
        	"sudo rm /usr/local/lib/libgeos*"

* Clone this repository in your pc:

`git clone https://github.com/DriveX-devs/VaN3Twin.git`

* Run, from this repository either:

`./sandbox_builder.sh install-dependencies` -> if this is the first time you install ns-3 or VaN3Twin on your system

or

`./sandbox_builder.sh` -> if this is **not** the first time you install ns-3 

This script will download the proper version of ns-3-dev and install this framework. The folder `ns-3-dev` will remain linked to this GitHub repository (not to the vanilla ns-3-dev one), allowing you to more easily develop updates and possibile contributions to *VaN3Twin*.
    
* Configure `ns3` to build the framework with `<ns3-folder>./ns3 configure --build-profile=optimized --enable-examples --enable-tests --disable-python (add here what you want to enable)"` - The usage of the optimized profile allows to speed up the simulation time. This command should be launched from inside the `ns-3-dev` folder.

* **Important**: If you are compiling VaN3Twin on Ubuntu 22.04 LTS or later, you need to specify, when calling `./ns3 configure`, also the `--disable-werror` flag

* Build ns3:
`./ns3 build`

**Important**

`src/automotive/` contains all the application related files and all the source code implementing the ETSI ITS-G5 stack for vehicular communications. Inside `sumo_files_v2v_map` you can find the SUMO map and trace for the V2V sample application, while inside `sumo_files_v2i_map` you can find the SUMO map and trace for the V2I sample application. Similarly you can find the SUMO map and trace for the Traffic Manager sample application inside `sumo_files_v2i_TM_map` and the ones for the Emergency Vehicle Warning inside `sumo_files_v2i_EVW_map`

`src/traci/` and `src/traci-applications/` contain instead all the logic to link ns-3 and SUMO. 

`src/cv2x/` contains the model for C-V2X in transmission mode 4.

`src/sionna/` contains the integration files for the NVIDIA-SIONNA Ray Tracing module.

The user is also encouraged to use the `sumo_files_v2v_map` and `sumo_files_v2i_map` folders to save there the SUMO-related files for his/her own applications.

**The version of CAM and DENM messages (v1 or v2)** can be easily switched by relying on the `switch_ETSI_version.sh` script. This script relies on the `ns-3-dev/src/automotive/model/ASN1/currmode.txt` file. Please **never** modify it manually or delete it!

# Working with an IDE

Although not necessarily required, you can also configure an IDE in order to more comfortably work with VaN3Twin.
The suggested IDEs, which has also been used for the development of VaN3Twin, are _QtCreator_ and _CLion_.

## QtCreator

You can find all the instructions for setting up QtCreator with ns-3 (and the same applies to VaN3Twin, as it is based on ns-3) on the [official ns-3 Wiki](https://www.nsnam.org/wiki/HOWTO_configure_QtCreator_with_ns-3).

QtCreator can be installed on Debian/Ubuntu with:
`sudo apt install qtcreator`

You need also to install the `libclang-common-8-dev` package (the command for Debian/Ubuntu is reported below):
`sudo apt install libclang-common-8-dev`

Not installing `libclang-common-8-dev` may result in QtCreator wrongly highlighting several errors and not recognizing some types, when opening any source or header file, even if the code compiles correctly.

**Important**: if `libclang-common-8-dev` is not available, you can try installing a newer version. For example, on Ubuntu 22, we verified that `libclang-common-15-dev` works well too.

## CLion

CLion can be easily installed with the [JetBrains Toolbox App](https://www.jetbrains.com/toolbox-app/).

You can find all the instructions for setting up CLion with ns-3 (and the same applies to VaN3Twin, as it is based on ns-3) on our [documentation](https://ms-van3ts-documentation.readthedocs.io/en/master/IDE.html#clion).

# VaN3Twin-CARLA extension

In addition to SUMO and GPS traces VaN3Twin supports the use of CARLA for mobility and sensor perception simulation. This extension leverages the [OpenCDA framework](https://github.com/ucla-mobility/OpenCDA) to develop an LDM module and extend the gRPC adapter devised [here](https://github.com/veins/veins_carla) to be able to extract not only localization information from CARLA but also perception information from the LDM module. The developed client module on ns-3 queries the information to use it for the mobility of each of the ns-3 simulated nodes and to update the LDM module with all perception data sent over the simulated vehicular network. 

**System requirements**

We highly recommend running VaN3Twin-CARLA on Ubuntu 20.04 (used for developing the framework) or Ubuntu 18.04, Ubuntu 22.04 is not officially supported by CARLA. If both CARLA and OpenCDA need to be installed we recommend at least 35GB of free space on your system. For smooth execution of simulation (especially if AI/ML models are leveraged for the perception simulation) we recommend using a GPU with at least 8GB of memory.
The version of CARLA supported by VaN3Twin-CARLA is CARLA 0.9.12.

**Installing VaN3Twin-CARLA**

To be able to use VaN3Twin-CARLA, CARLA and OpenCDA need to be installed. After following the steps detailed above to build the project, from inside the `ns-3-dev` folder execute the `switch_ms-van3t-CARLA.sh` script to install all necessary dependencies. 
The script will try to find the path of your CARLA and OpenCDA installation to be defined in the `CARLA-OpenCDA.conf` file. If the user already has an installation of either CARLA or OpenCDA they should specify the path to the installation together with the path to their Python environment in the following way: 

`CARLA_HOME=/path/to/CARLA_0.9.12`

`OpenCDA_HOME=/path/to/OpenCDA`

`Python_Interpreter=/path/to/anaconda3/envs/msvan3t_carla/bin/python3` if using conda or `Python_Interpreter=python3.7` otherwise.

In case this is the first time using either CARLA or OpenCDA, the script will install them, prompting for confirmation in each case. It is highly recommended to install OpenCDA with conda.

Once the script finishes its execution, it is recommended to build only the simulation scripts for CARLA's usage instead of the whole project (i.e., run `./ns3 build "name-of-CARLA-example"` instead of `./ns3 build`).
This permits avoiding libraries and dependency issues coming from other parts of the simulator, so that the CARLA simulations can be properly run.

**Possible issues with CARLA and OpenCDA installation**

Below is a list of possible issues that can arise during the configuration and possible ways to prevent/solve them:
- In case OpenCDA was already present inside the VaN3Twin folder from previous runs of the script `switch_ms-van3t-CARLA.sh`, it is recommended to clean the cache of OpenCDA's folder before running again the installation script again, with the command: `sudo rm -r OpenCDA/cache/`.
- In case the anaconda virtual environment `msvan3t_carla` was already present in your system from previous runs of the script `switch_ms-van3t-CARLA.sh`, it is recommended to delete it before running again the installation script, with `conda remove -n msvan3t_carla --all` or `sudo rm -r ~/anaconda3/envs/environment_folder/`.
- The anaconda environment solution steps (i.e., the prints `Solving environment:` in the installation phase) might take a while to complete, especially if it is not the first time you try to install the packages. It is normal to wait for some minutes.
- There is an issue encountered in some edge cases, in which the configuration file to use OpenCDA's perception module is not working properly (`ms_van3t_example_ml`). We are still investigating the problem, since it happens only in some particular situations. To use CARLA and OpenCDA in any case, we suggest to adopt the default OpenCDA's configuration file (`ms_van3t_example`).

**VaN3Twin-CARLA examples**

Two examples leveraging CARLA are provided, showcasing how to use the extension with both IEEE 802.11p and NR-V2X as access technologies.
To run the provided examples: 

`./ns3 run "v2v-carla-80211p"` or  `./ns3 run "v2v-carla-nrv2x"` 

 For further description of the modules provided in this extension please refer to our paper [here](https://www.eurecom.fr/publication/7556/download/comsys-publi-7556.pdf).

# VaN3Twin NVIDIA-Sionna extension

To integrate `NVIDIA Sionna` with the `VaN3Twin` framework, you need to install the Python library for Sionna.

This can be done locally or on a remote server using the following command:

```sh
  pip install sionna
```

The supported and tested versions of `Sionna` are v0.19.0 and v1.0.

To create Sionna environments, make sure to use Blender version 3.6.22 with the mitsuba-blender addon available [here](https://github.com/mitsuba-renderer/mitsuba-blender).

The mitsuba-blender addon is used to export the scene in .xml format from Blender to Mitsuba.

Once SIONNA is installed, you need first to configure the ray tracer simulation through `sionna_server_script.py` or `sionna_v1_server_script.py` scripts.
The main difference between these two scripts is that the `sionna_server_script.py` simulates a simpler ray tracer simulation, without taking into account the vehicles speed and so the Doppler Effect.
On the other hand, `sionna_v1_server_script.py` integrates a more complete and rigorous simulation.

The arguments of both the two Python scripts are:
- `--file_name`, a string to identify the path of your Sionna scenario file. 
- `--local_machine`, include this argument if you want Sionna to run on the local machine. If not provided, the script will interpret that Sionna is running on a remote server.
- `--verbose`, include this argument if you want to print Sionna logs to the console.

The `sionna_server_script.py`/`sionna_v1_server_script.py` script automatically identifies the presence of a GPU and configures TensorFlow accordingly. It also sets up a UDP socket to communicate with the `VaN3Twin` framework, handling various types of messages such as location (and speed in the v1 script) updates, path loss requests, delay requests, and line-of-sight (LOS) checks.

Example usage:

```sh
  python3 sionna_v1_server_script.py --file_name src/sionna/scenarios/SionnaCircleScenario/scene.xml --local-machine --verbose
```

If you are running Sionna on a remote server, you need to specify the IP address of the SIONNA server in your simulation file inside VaN3Twin.

For instance, the `src/automotive/examples/v2v-cam-exchange-sionna-80211p.cc` simulation contains an example of how to configure the communication between VaN3Twin and Sionna on the VaN3Twin's side.
Refer to this example to configure the IP address for a remote SIONNA server in your simulation file.

By following these steps, you can successfully integrate `Sionna` with `VaN3Twin` framework and run simulations that leverage Sionna's ray tracing capabilities.

# VaN3Twin co-channel coexistence extension

`VaN3Twin` offers a special module to simulate the coexistence between different CAVS communication technologies.
Currently, the simulator can integrate scenarios where vehicles use either the `IEEE 802.11p` or `NR-V2X` communication stacks, even when their channels overlap.

The interference management is done by a special module called `TxTracker`.

To enable this specific feature of the simulator, it is important to **follow these steps precisely**:
1. Run the `./switch_ms-van3t-interference.sh` script with the `on` argument to configure the environment for interference simulation:
   ```sh
    ./switch_ms-van3t-interference.sh on
   ```

2. **Build just your example** rather than building all examples as in the classical simulator usage:
   ```sh
    ./ns3 build <your-example>
    # Example:
    ./ns3 build "v2v-coexistence-80211p-nrv2x"
   ```

3. Then run your example with:
    ```sh
    ./ns3 run <your-example>
    # Example:
    ./ns3 run "v2v-coexistence-80211p-nrv2x"
   ```
   
4. To come back to the **normal mode** of `VaN3Twin`, run the `./switch_ms-van3t-interference.sh` script again but with the `off` argument:
   ```sh
    ./switch_ms-van3t-interference.sh off
   ```
   
5. This will allow you to build and run all files as in the classical simulator usage:
    ```sh
    # Example
    ./ns3 build
    ./ns3 run "v2v-simple-cam-exchange-80211p"
    ```

For more details on how to manage the co-channel coexistence simulations, such as how to pass the necessary parameters to the `TxTracker` module, you can refer to the example `src/automotive/examples/v2v-coexistence-80211p-nrv2x.cc`.

# Supported ETSI C-ITS messages

*VaN3Twin* currently supports the following ETSI C-ITS messages:
- CAM
- DENM
- IVIM
- CPM
- VAM

For the transmission and reception of IVIMs (from an RSU to vehicles), you can refer to the `v2i-emergencyVehicleWarning-80211p` example.

# Introduction of ETSI DCC (DCC_NET, DCC_CROSS, DCC_ACC – Reactive & Adaptive)

*VaN3Twin* now includes a full implementation of the **ETSI Decentralized Congestion Control (DCC)** mechanism, enabling realistic modeling of channel access regulation and cooperative congestion management for ITS-G5 / ITS-G5-like stacks.
The implemented DCC framework follows the [ETSI TS 102 687](https://www.etsi.org/deliver/etsi_ts/102600_102699/102687/01.02.01_60/ts_102687v010201p.pdf) specification and supports all three control components:

* **DCC_NET (Network Layer DCC)**
  Handles the Channel Busy Ratio (CBR) sharing through the GeoNetworking header. Based on that, the receiver cna compute the Global CBR (CBR_G) to take into account not just the one sensed locally, but also the one received from neighbors.

* **DCC_CROSS (Cross-Layer DCC)**
  Acts as the coordination plane between layers, ensuring coherent transitions across application, networking, and MAC layers based on the observed channel load.

* **DCC_ACC (Access Layer DCC)**
  Implements the PHY/MAC-level congestion control. This includes both:

    * **Reactive DCC**: immediate adaptations to sudden changes in Channel Busy Ratio (CBR), following the fast-timescale state machine.
    * **Adaptive DCC**: long-term regulation based on smoothed CBR estimates, using timers and thresholds to prevent oscillations.

### Key Features of the DCC Integration

* **Standards-compliant state machines**
  The **Relaxed**, **Active**, and **Restrictive** DCC states are fully supported, driving transmission rate, Tx power, and CBR threshold adjustments as per ETSI requirements.

* **Cross-layer coordination hooks**
  Applications, BTP, and GeoNetworking modules transparently receive updated DCC parameters and do not need to manually manage state transitions.

* **Channel load monitoring**
  The PHY layers (IEEE 802.11p) export real-time channel occupancy metrics to the DCC engine, allowing accurate CBR-based congestion estimation.

* **Compatible with all VaN3Twin ITS-G5 scenarios**
  DCC logic can be automatically enabled for all ETSI ITS-G5 simulations (CAM, DENM, CPM, VAM, IVIM) with few lines of code in user applications.

### How DCC Is Enabled in VaN3Twin

DCC is **not implicitly enabled by default**. Instead, each vehicle that should run ETSI DCC explicitly receives its own DCC instance during the setup phase:

* The `MetricSupervisor` object should be configured to initiate the CBR checking for the nodes that will include DCC in their features:
    ```cpp
    metSup->startCheckCBR(dcc_nodes);
    metSup->setCBRWindowValue(100);
    metSup->setCBRAlphaValue(0.1);
    ```
* A `DCC` object is created for each vehicle that should use congestion control.
    ```cpp
    std::unordered_map<Ptr<Node>, Ptr<DCC>> dcc_per_node;
    for (uint8_t i = 0; i < dcc_nodes; i++)
    {
    dcc_per_node[c.Get(i)] = CreateObject<DCC>();
    }
    ```

* The instance is initialized inside the `setupNewWiFiNode` callback through:

  ```cpp
  dcc->SetupDCC(vehicleID, metSup, node, "adaptive", 200);
  dcc->setBitRate(6e6);
  gn->setDCC(dcc); // GeoNetworking binding
  dcc->StartDCC();
  ```

This design provides **per-vehicle control** instead of global automatic activation.

#### Main Parameters in the DCC Setup

The most relevant parameters configured during setup are:

* **DCC Modality** (`"adaptive"`):
  Selects the congestion control algorithm. `"adaptive"` enables the long-term ETSI Adaptive DCC logic, while other mode (i.e., `"reactive"`) can be selected by the user.

* **Initial Beaconing Rate (200 ms)**:
  The fourth argument of `SetupDCC()` sets the initial DCC check interval (in milliseconds).

* **Bit Rate (`6e6` bps)**:
  `setBitRate()` configures the PHY-layer nominal data rate used by the ITS-G5 station for the transmission.

* **Node Binding (`node` and `vehicleID`)**:
  Associates each DCC instance with the specific ns-3 node and its vehicle identifier.

Together, these parameters allow DCC to be configured with realistic, ETSI-compliant behavior, while still giving the user full control over **which vehicles**, **which algorithms**, and **which PHY parameters** are used.

### Supported DCC Modes at a Glance

| DCC Component          | Supported | Notes                                 |
| ---------------------- | --------- |---------------------------------------|
| **DCC_NET**            | ✔️        | GeoNetworking CBR sharing             |
| **DCC_CROSS**          | ✔️        | Coordinated state transitions across the ITS-G5 stack |
| **DCC_ACC – Reactive** | ✔️        | Fast-timescale state machine triggered by instant CBR |
| **DCC_ACC – Adaptive** | ✔️        | Slow-timescale smoothing + stable congestion mitigation |

This integration provides a realistic, standards-aligned congestion control behavior, enabling *VaN3Twin* to simulate dense ITS scenarios with high fidelity, including edge cases such as large-scale beaconing, clustered traffic, and multi-technology coexistence (e.g., when combined with the coexistence module).

# Sample V2I example and V2I/V2N applications

*VaN3Twin* currently supports two stacks/communication technologies for V2I/V2N:
- 802.11p, communicating, for instance, with a Road Side Unit (sample program name: `v2i-areaSpeedAdvisor-80211p`)
- LTE, for V2N communications (sample program name: `v2i-areaSpeedAdvisor-lte`)

To run the sample V2I program you can use:
`./ns3 run "v2i-areaSpeedAdvisor-lte"` or
`./ns3 run "v2i-areaSpeedAdvisor-80211p"`

*  Nodes are created in the ns3 simulation as vehicles enter the SUMO simulation
*  A full LTE or 802.11p stack is implemented at lower layers (depending on which example is run)

In this example, every vehicle that enters the scenario will start sending CAMs with a frequency between *1 Hz* and *10 Hz* (according to the ETSI standards). 

Then, the logic of the two sample applications for V2I/V2N is similar, but it differs slightly depending on whether 802.11p is used (V2I) or LTE is used (V2N).  

At a glance, in 802.11p vehicles broadcast periodic CAM messages and an RSU periodically broadcasts DENM messages to inform vehicles travelling in a low speed area to slow down. In this case CAMs and DENMs messages are encapsulated inside BTP and GeoNetworking.
In LTE, instead, the CAM messages are forwarded to a remote host (behind an eNB + EPC), which analyzes the position of the vehicles and sends unicast DENMs to vehicles entering a low speed area to change their maximum allowed speed. In this case, due to the absence of a MBMS module in the LTE framework of ns-3, all the messages are sent in unicast. 
CAM and DENM messages are encapsulated inside BTP, GeoNetworking, UDP and IP.

__802.11p application logic__

The map is divided into two areas: a circular area in the middle of the map, with a radius of 90 m, on which DENMs are broadcasted by the RSU (with their GeoArea set accordingly), where the maximum speed is 25km/h and an outer area, where the speed limit is set to 75km/h.

The RSU disseminates a DENM every second in the area mentioned before, and continues its transmissions until it receives CAMs from vehicles in the map. When no CAMs are received for more than 5 seconds, the DENM dissemination in paused, until new vehicles enter the scenario and new CAMs are received by the RSU.

![](img/v2i-80211p.png)

__LTE application logic__

The map is divided into two areas: the area in the middle, where the maximum speed is 25km/h and an outer area, where the speed limit is set to 75km/h. In this case, DENMs cannot be transmitted using merely BTP and GeoNetworking, but they have to rely on UDP and IPv4, since the server is located in a remote host behind the eNB and EPC. The server checks whenever a transition between the two areas is performed by a vehicle, and, when it happens, it sends it a _unicast_ DENM message to tell it to slow-down (or to let it speed-up again).

![](img/v2i-lte.png)

__Mobility Traces and Facilities Layer__

The mobility trace is contained in the file `ns-3-dev/src/automotive/example/sumo_files_v2i_map/cars.rou.xml`.
This SUMO map embeds some re-routers allowing the vehicles to continuously move in the map.

The CAMs and DENMs dissemination logics are in the modules inside the `automotive/Facilities` folder while the application logic resides on (areaSpeedAdvisorClient80211p.cc/.h, areaSpeedAdvisorClientLTE.cc/.h) and (areaSpeedAdvisorServer80211p.cc/.h, areaSpeedAdvisorServerLTE.cc/.h) inside `automotive/Applications`.

The user *IS NOT* expected to modify the code inside the "Facilities", "BTP" and "GeoNet" folders, but rather to use the ETSI Facilities Layer methods inside the application.

**Important**

If using the LTE version in this very simple toy case, it is possible to connect at most 23 UEs to the enB (due to the LENA framework currently implemented features). You can avoid this problem by using the option `--ns3::LteEnbRrc::SrsPeriodicity=[value]"` where [value]=0, 2, 5, 10, 20, 40, 80, 160, 320. In this way you can add more UEs. Example: `./waf --run "v2i-areaSpeedAdvisory-lte --ns3::LteEnbRrc::SrsPeriodicity=160"`

**List of the most important options:**
* `--realtime                  [bool] decide to run the simulation using the realtime scheduler or not`
* `--sim-time                  [double] simulation time`
* `--sumo-gui                  [bool] decide to show sumo-gui or not`
* `--server-aggregate-output   [bool] if true, the server will print every second a report on the number of DENM sent and CAM received correctly`
* `--sumo-updates              [double] frequency of SUMO updates`
* `--csv-log                   [string] prefix of the CSV log files where to save the disaggregated data coming from the CAMs received by the server and the DENMs received by the vehicles (the user can then use this sample application to build more complex logging mechanisms and/or log additional data coming from the server and/or the vehicles)`



# Sample V2V example and V2V applications

*VaN3Twin* currently supports three stacks/communication technologies for V2V:
- 802.11p (sample program name: `v2v-emergencyVehicleAlert-80211p`)
- LTE-V2X Mode 4 (sample program name: `v2v-emergencyVehicleAlert-cv2x`)
- NR-V2X Mode 2 (sample program name: `v2v-emergencyVehicleAlert-nrv2x`))

To run the program:

`./ns3 run "v2v-emergencyVehicleAlert-cv2x"` or
`./ns3 run "v2v-emergencyVehicleAlert-80211p"` or
`./ns3 run "v2v-emergencyVehicleAlert-nrv2x"`

*  Nodes are created in the ns3 simulation as vehicle enters the SUMO simulation
*  A full NR-V2X, LTE-V2X or 802.11p stack is implemented at lower layers

In this example, every vehicle that enters the scenario will start sending CAMs with a frequency between *1 Hz* and *10 Hz* (according to the ETSI standards). The vehicles are divided into "passenger" vehicles (i.e., normal vehicles) and "emergency" vehicles. 

A CAM generated by an emergency vehicle will have the "StationType" Data Element (i.e. a field of the message) set to "specialVehicles".
When normal vehicles receive these CAM messages from an emergency vehicle, they will check whether their heading is similar to the one of the emergency vehicle and which is their distance to the latter.

If the heading is similar and the distance is small enough, it means that the emergency vehicle is approaching. In this case, the receiving vehicles will either slow down (if on a different lane than the one the emergency vehicle is travelling on) or change lane as soon as possible (speeding up for a little while, if necessary, when they are on the same lane as the emergency vehicle).

When acting, in the SUMO GUI, vehicles will either turn orange (different lane --> slow down) or green (same lane --> clear path as soon as possible).

The CAMs and DENMs dissemination logic are in the modules inside the `automotive/Facilities` folder while the application logic is inside emergencyVehicleAlert.cc/.h (in `automotive/Applications`).
The user *IS NOT* expected to modify the code inside the "Facilities", "BTP" or "GeoNet" folders, but rather to use the ETSI Facilities Layer methods inside the application.

The SUMO scenario comprehends a ring-like topology, with two directions and two lanes for each direction (with a total of 4 lanes). 

![](img/v2v-road-topology.png)

The mobility trace is contained inside the file `automotive/example/sumo_files_v2v_map/cars.rou.xml`.

The SUMO map also embeds some re-routers allowing the vehicles to continuously travel on the available road segments.

![](img/v2v-logic.png)

**List of the most important options:**
* `--realtime                   [bool] decide to run the simulation using the realtime scheduler or not`
* `--sim-time                   [double] simulation time`
* `--sumo-gui                   [bool] decide to show sumo-gui or not`
* `--sumo-updates               [double] frequency of SUMO updates`
* `--csv-log:                   [string] prefix of the CSV log files where to save CAMs and DENMs disaggregated data and statistics`


# Sample V2X emulator application

*VaN3Twin* also includes an example of an emulation application, which is able to send the CAMs and DENMs generated by the vehicles, (virtually) travelling on the SUMO map, over a real network, by relying on a physical interface.

The same application should also be able to receive CAMs and DENMs coming from the external world (i.e. from a certain physical interface of the device running ns-3).

For the time being, this sample application is relying on the same map and mobility traces of the V2V application and it sends both CAM messages and periodic DENM messages, as an example on how both kinds of messages can be emulated and sent to the external world.

In order to properly work, the emulator application should always run in real time, and the device on which ns-3 is run should be able to handle the specified number of vehicles without delays and without slowing down. 

As it is communicating with the external world, it handles only ASN.1 standard-compliant messages.

More in details, this application emulates N vehicles, each with its own CA and DEN basic service, and make them send the CAM/DENM messages and receive the CAM/DENM messages through a physical interface (specified with the "interface" option), instead of using any ns-3 simulated model.
This should enable, in the future, hardware-in-the-loop testing and evaluation.

You can run it with:
`./ns3 run "v2x-emulator --interface=<interface name>"`

Where `<interface name>` is the name of the physical interface, on your PC, where CAMs will be sent.

**Please note that the interface, in order to work with ns-3, should be put in promiscuous mode.**

You can put an interface in promiscuous mode with:
`sudo ip link set <interface name> promisc on`

The promiscuous mode can then be disabled with:
`sudo ip link set <interface name> promisc off`

`sudo` may be needed to use the underlying ns-3 *FdNetDevice*: if you get a "permission denied" error, try adding `--enable-sudo` when doing `./ns3 configure` and then running without `sudo` (ns3 will ask for your password if needed).  

The logic of the application is contained inside model/Applications/v2x-helper.c/.h

**UDP mode** 

In the default emulation mode, messages will be sent, through the specified interface, as broadcast packets encapsulated inside BTP and GeoNetworking.

The user can also specify, however, a _UDP mode_, enabling the transmission of messages to an external UDP server. In this case, the ETSI V2X messages (i.e. CAM, DENM) will be encapsulated inside BTP --> GeoNetworking --> UDP --> IPv4, and sent to a host with a specified IPv4 and port.

*Any host is fine, but the following limitations apply:*

- The remote UDP server must be able to reply to the ARP requests sent by the vehicles, which will use their own source IP address and MAC and **not** the ones of the physical interface
- No loopback operations are possible so far, due to the limitation mentioned before
- The network at which the physical interface is connected shall be able to support the communication using spoofed MAC and IP addresses (otherwise, the ARP requests sent by ns-3 may not receive any reply). In general, we verified that any Ethernet link between ns-3 and the remote host receiving the UDP packets should be fine.

**Screenshots**

The following screenshot shows a Wireshark capture of the messages sent by the emulator application, when operating in normal mode and selecting the `ens33` interface (e.g. `./ns3 run "v2x-emulator --interface=ens33"`)

![](img/v2x-emulator-normal-mode.png)


The following screenshot shows a Wireshark capture of the messages sent by the emulator application, when operating in UDP mode, targeting a UDP server at 192.168.1.124/24, port 20000, and transmitting over the `ens33` interface (e.g. `./ns3 run "v2x-emulator --udp=192.168.1.124:20000 --interface=ens33 netmask=255.255.255.0 gateway=192.168.1.1"`)

![](img/v2x-emulator-udp-mode.png)


**List of the most important options:**
* `--sim-time                   [double] total emulation/simulation time`
* `--sumo-gui                   [bool] decide to show sumo-gui or not`
* `--sumo-updates               [double] frequency of SUMO updates`
* `--send-cam                   [bool] enable vehicles to send CAMs`
* `--send-denm                  [bool] enable vehicles to send DENMs`
* `--interface                  [string] Name of the physical interface to send(/receive) V2X messages to(/from)`
* `--udp                		   [string] To enable UDP mode and specify UDP port and IP address where the V2X messages are redirected (format: <IP>:<port>)`
* `--gateway                    [string] To specify the gateway at which the UDP/IP packets will be sent`
* `--subnet                     [string] To specify the subnet which will  be used to assign the IP addresses of emulated nodes (the .1 address is automatically excluded)`
*  `--netmask                     [string] To specify the netmask of the network`

# VaN3Twin web-based vehicle visualizer

**Requirement:** if you want to use this module, Node.js should be installed (on Ubuntu/Debian you can install it with `sudo apt install nodejs`).

*VaN3Twin* also comes with a web-based vehicle visualizer, able to display the vehicles travelling
during the simulation on a map, rendered directly inside the user's browser.

It can be particularly useful when working with GPS Traces (see the `gps-tc` module), which are not
coupled with a GUI (as opposed to SUMO).

If you want to fully exploit its potentiality, you need a Mapbox token, as the visualizer
relies on Mapbox to draw the street, hybrid and satellite map layers.

**Once you get it, the mapbox token shall be copied inside the file `mapbox_token` in `src/vehicle-visualizer/js`**

You can find more information on Mapbox [here](https://www.mapbox.com/). They currently have quite good free tier options (allowing, at least in March 2021, up to 200000 free tile requests/month),
which should be enough to use the VaN3Twin vehicle visualizer without the need of paying anything. Please check them before signing up to Mapbox
and getting a token to be used here. In general, we found out that a normal simulation, in which some sporadic zoom in/zoom out and
three layer changes are performed, may require around 150/200 tile requests (we advise you to check often the tile request count on the Mapbox website, when you use your token).

In general, you should disable the vehicle visualizer when doing long batches of simulations.

If you do not own a Mapbox token (i.e. the `mapbox_token` file is empty), the visualizer will work in any case, with the following limitations:
- You **must** make an **occasional** use of the visualizer (i.e. **no** heavy usage allowed, **no** derivative apps can be developed
  starting from the visualizer). Not making an occasional usage will conflict with the [Tile Usage Policy](https://operations.osmfoundation.org/policies/tiles/) of
  OpenStreetMap, which is not considered acceptable. So, you **must disable** the vehicle-visualizer
  when doing batches of simulations to gather some results. **You are responsible for this!**
- Only one map layer (standard streets view from OpenStreetMap) will be available for use  

In order to use the visualizer in your application, you need to add, in the main function (i.e. `int main()`),
the following code, which creates a new `vehicleVisualizer` object:

    vehicleVisualizer vehicleVisObj;
    Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
    vehicleVis->startServer();
    vehicleVis->connectToServer ();

Do **not** create a new `vehicleVisualizer` object with `CreateObject` or `new`, unless you plan to manually
call `delete` on it, as we rely on the object destructor in order to send a terminate message to the web visualizer
server and gracefully terminate it.

After creating a new vehicle visualizer object, you need to pass its pointer to the module which manages the vehicle's mobility.
If you are using SUMO and TraCI, you can call:

    sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));

Where sumoClient is a pointer to TraCI client object (`Ptr<TraciClient>`) you previously created.

If you are using, instead, GPS-tc, you can call:

    GPSTCHelper.setVehicleVisualizer(vehicleVis);

Where `GPSTCHelper` is a `GPSTraceClientHelper` object. In this case, `setVehicleVisualizer()`
must be called **before** `createTraceClientsFromCSV()`, otherwise the vehicle visualizer
reference will not be passed to the GPS Trace Client objects managing the mobility of the
vehicles and no moving vehicles will appear on the map.

The vehicles displayed by the visualizer can also be inserted and updated from user applications (e.g. in an emulator application, you
can use the CAMs received from the external world to add real vehicles to the map, together
with the simulated ones).

In order to add and update a moving object from an application, the latter should have, first of all,
a reference to the visualizer object (i.e. `Ptr<vehicleVisualizer>`). Then, it will be able to use
`<name of pointer to the vehicleVisualizer>->sendObjectUpdate()` to add and update any object in the map.
In this case it is important to specify, as first argument of the `sendObjectUpdate()` function, an object ID which must be different from the IDs of the simulated
vehicles.

You can also refer to the examples inside `src/automotive/examples`, which all (but the V2X emulator) include the possibility
of using the web-based vehicle visualizer via the `--vehicle-visualizer=true` option.

The visualizer, once a simulation has been started, can be opened inside any browser, as long as the simulation is running, at `localhost:8080` (if a different HTTP port is not specified with the `setHTTPPort()` method of the `vehicleVisualizer` object).

## Acknowledgements

To acknowledge us in your publication(s) please refer to the following publications:

### Journal publication @ Elsevier Computer Communications (2024)

```tex
@article{ms-van3t-journal-2024,
	title = {ms-van3t: An integrated multi-stack framework for virtual validation of V2X communication and services},
	journal = {Computer Communications},
	volume = {217},
	pages = {70-86},
	year = {2024},
	issn = {0140-3664},
	doi = {https://doi.org/10.1016/j.comcom.2024.01.022},
	url = {https://www.sciencedirect.com/science/article/pii/S0140366424000227},
	author = {F. Raviglione and C.M. Risma Carletti and M. Malinverno and C. Casetti and C.F. Chiasserini},
	keywords = {Connected vehicles, V2X, Virtual validation, Hardware in the loop},
}
```

Our manuscript is open access and it is available [here](https://doi.org/10.1016/j.comcom.2024.01.022).

### Main VaN3Twin publication (pre-print, 2025)

Our work has been submitted and a pre-print is available [on arXiv](https://arxiv.org/abs/2505.14184):
```tex
@misc{pegurri2025van3twinmultitechnologyv2xdigital,
      title={VaN3Twin: the Multi-Technology V2X Digital Twin with Ray-Tracing in the Loop}, 
      author={Roberto Pegurri and Diego Gasco and Francesco Linsalata and Marco Rapelli and Eugenio Moro and Francesco Raviglione and Claudio Casetti},
      year={2025},
      eprint={2505.14184},
      archivePrefix={arXiv},
      primaryClass={cs.NI},
      url={https://arxiv.org/abs/2505.14184}, 
}
```

### Project Acknowledgments

The development of the framework was also carried out within the **MOST – Sustainable Mobility National Research Center** (CN00000023, MOST Spoke 6), and supported by the European Union under the Italian National Recovery and Resilience Plan (NRRP) of NextGenerationEU, partnership on "Telecommunications of the Future" (PE00000001 - program **"RESTART"**).
