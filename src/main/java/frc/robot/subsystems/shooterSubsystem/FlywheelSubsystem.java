package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX flywheelMotor = new TalonFX(FlyWheelConstants.LEADER_ID);
    private final SmartMotorController motorController;
    private final FlyWheel flywheel;

    private AngularVelocity targetSpeed = RPM.of(0);

    public FlywheelSubsystem() {
        this.motorController = new TalonFXWrapper(
            flywheelMotor,
            FlyWheelConstants.MOTORS,
            FlyWheelConstants.MOTOR_CONFIG.withSubsystem(this) 
        );

        this.flywheel = new FlyWheel(
            FlyWheelConstants.FLYWHEEL_CONFIG.withSmartMotorController(motorController)
        );
    }

    public boolean isUpToSpeed() {
        return flywheel.getSpeed().isNear(targetSpeed, FlyWheelConstants.TOLERANCE);
    }

    public void setVelocity(AngularVelocity speed) {
        targetSpeed = speed;
        flywheel.setSpeed(speed);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }

    public AngularVelocity getVelocity() {
        return flywheel.getSpeed();
    }

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
