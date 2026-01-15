package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;

public class Turret extends SubsystemBase{
    TalonFX turretmotor = new TalonFX(1);//id subject to change
    SmartMotorControllerConfig motorconfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))

}
