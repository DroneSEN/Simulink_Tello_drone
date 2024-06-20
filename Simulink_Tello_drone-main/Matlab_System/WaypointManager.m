classdef WaypointManager < matlab.System
    % WaypointManager Gère les waypoints à suivre
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties

    end

    % Pre-computed constants or internal states
    properties (Access = private)
        % Index du waypoint (à partir de 1)
        CurrentWaypointIndex = 1;

        % Fin de la patrouille
        Finished = 0;

        % Flag utilisé pour détecter un front montant sur forceNextWaypoint
        PrvForceNextWaypoint = 0;
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [waypoint, finished] = stepImpl(obj,enable, lookaheadDistance, currentPos, waypoints, numWaypoints, forceNextWaypoint)

            if enable
                % Calcul de la distance entre le drone et le waypoint actuel
                errorPos = sqrt(sum((currentPos - waypoints(obj.CurrentWaypointIndex, 1:3)).^2));
    
                obj.Finished = 0;
                
                % Si la distance est acceptable ou qu'on force le prochain
                % waypoint
                if errorPos <= lookaheadDistance || (obj.PrvForceNextWaypoint ~= forceNextWaypoint && forceNextWaypoint == 1)
                    

                    % S'il ne s'âgit pas de la dernière position, on passe
                    % à la suivante
                    if obj.CurrentWaypointIndex ~= numWaypoints
                        obj.CurrentWaypointIndex = obj.CurrentWaypointIndex + 1;
                    else
                        obj.Finished = 1;
                    end

                end
            else
                % Si le WaypointManager n'est pas activé
                obj.Finished = 0;
                obj.CurrentWaypointIndex = 1;
            end

            % On actualise le flag pour détecter les front montants
            obj.PrvForceNextWaypoint = forceNextWaypoint;
            
            % On retourne le waypoint actuel
            finished = obj.Finished;
            waypoint = waypoints(obj.CurrentWaypointIndex, 1:3);
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end
    end
end
