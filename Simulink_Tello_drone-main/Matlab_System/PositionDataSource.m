classdef PositionDataSource < Simulink.IntEnumType
    enumeration
        % Aucune données (0,0,0)
        None(1)

        % Position à partir des données Optitrack
        Optitrack(2)
        
        % Code aruco
        Aruco(3)
        
        % Estimation de la position avec l'IMU
        IMU(4)
        
        % Estimation de la position avec le SLAM
        SLAM(5)
        
        % Estimation de la position avec le SLAM et l'IMU
        SensorFusion(6)
    end
end