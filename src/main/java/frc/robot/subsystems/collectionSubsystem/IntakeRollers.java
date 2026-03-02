package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeRollerConstants;
import frc.robot.utils.RobotConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.ROLLER_ID);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withStatorCurrentLimit(IntakeRollerConstants.STATOR_LIMIT)
    .withMotorInverted(false)
    .withGearing(1)
    .withTelemetry("RollerMotor", RobotConstants.GetTelemetry());

    private final SmartMotorController roller = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX60Foc(1), motorConfig);

    public void intake() {
        roller.setDutyCycle(IntakeRollerConstants.INTAKE);
    }

    public void outtake() {
        roller.setDutyCycle(IntakeRollerConstants.OUTTAKE);
    }

    public void stop() {
        roller.setDutyCycle(0);
    }
}
