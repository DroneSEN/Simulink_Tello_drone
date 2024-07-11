classdef DroneState < Simulink.IntEnumType
    % DroneState: Énumération des états du drone
    %  Cette énumération définit les états possibles du drone
    enumeration
        % Le drone est en attente, caméra vers le bas activée
        Idle(0)
        
        % Initialisation du drone, caméra vers le bas activée
        Startup(1)
        
        % Décollage du drone
        Takeoff(2)
        
        % Régulation de la position initiale par rapport à la cible Aruco
        HomePosition(3)
        
        % Initialisation de la patrouille, activation de la caméra avant et SLAM
        InitPatrolling(4)
        
        % Patrouille en suivant les points de passage
        Patrolling(5)
        
        % Atterrissage du drone, désactivation des commandes RC
        Landing(6)
        
        % Drone a atterri et est inactif
        Landed(7)
        
        % Abandon de la mission, atterrissage immédiat et désactivation des caméras
        Aborting(8)
    end
end