# Optilink
A tool to link Optitrack to Matlab/Simulink.

This tool is still in development.

## Installation

⚠️ This project only works on Windows and Linux (Ubuntu and Fedora)

1. Place the Optilink directory in your Matlab project 
2. Unzip the NatNetSDK archive, since it contains the .DLL file required for communicating with Optitrack
3. In Matlab, right click on the "Optilink" directory > Add to path > Selected folders and subfolders 
4. On first use of the Optilink tool, Matlab will open a dialog box to select the library file. Select the file in `NatNetSDK/lib/x64/NatNetML.dll`.

## How to use

### Calibration files

This project contains Optitrack calibration files, to be used in the ESTACA drone room, with the Motive Tracker software.

Those files can be found in the `motive_tracker` directory.

### Matlab Code

To use Optilink in your Matlab code, use the Optilink class.

Set the HostIP and ClientIP properties when instanciating the Optilink match the IP addresses of your Optitrack server and client, respectively.

The HostIP can be found on the Optitrack data streaming configuration.

Call the .connect() and .disconnect() at the beginning and at the end of your code.

**See code below**

```matlab
% Motive Tracker network configuration
% If the server is configured on its loopback address, set 127.0.0.1
% Otherwise, configure the address on the Optitrack streaming server
% public address
HOST_ADDR = '127.0.0.1';
CLIENT_ADDR = '127.0.0.1';

% Initialize the link between Optitrack
optilink = Optilink(HOST_ADDR, CLIENT_ADDR);

% Connect to the server
optilink.connect();

while true
    % Retrieve the drones
    drones = optilink.fetchOptitrackGroundTruth();
end
```

### Simulink

Add the OptilinkSystem block to your Simulink model and connect it as needed within your simulation.

Set the HostIP and ClientIP properties of the OptilinkSystem block to match the IP addresses of your Optitrack server and client, respectively. These properties specify the IP addresses for communication with the Optitrack system.

The HostIP can be found on the Optitrack data streaming configuration.

The simulink block will automatically connect to the server on startup and disconnect when the simulation stops.

## NatNetSDK

This project relies on the NatNetSDK : https://optitrack.com/software/natnet-sdk/

The SDK is packaged as an archive in this project and must be unzipped before using in your project.

⚠️ The SDK only works on Windows and Linux (Ubuntu and Fedora)