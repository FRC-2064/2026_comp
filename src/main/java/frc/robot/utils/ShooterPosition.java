package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;


public record ShooterPosition(
    Angle turretAngle, 
    Angle hoodAngle, 
    AngularVelocity flywheelSpeed
) {}