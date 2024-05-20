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

    % Print data
    if ~isempty(drones)
        % display(drones{1});
    end
end

