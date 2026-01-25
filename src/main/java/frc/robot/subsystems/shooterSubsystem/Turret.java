package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
    private final SmartMotorController motor;
    private final Pivot turret;
    
    private Angle targetAngle = Degrees.of(0);

    public Turret() {
        this.motor = new TalonFXWrapper(
            turretMotor,
            TurretConstants.MOTOR_TYPE,
            TurretConstants.MOTOR_CONFIG.withSubsystem(this)
        );

        this.turret = new Pivot(
            TurretConstants.TURRET_CONFIG.withSmartMotorController(motor)
        );
    }

    public void setTargetAngle(Angle angle) {
        this.targetAngle = angle;
        turret.setAngle(angle);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return turret.getAngle();
    }

    public boolean atPosition() {
        return targetAngle.isNear(turret.getAngle(), TurretConstants.TOLERANCE);
    }

    @Override
    public void periodic() {
        turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }
}