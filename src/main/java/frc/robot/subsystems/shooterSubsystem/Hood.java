package frc.robot.subsystems.shooterSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);
    private final SmartMotorController motor;
    private final Pivot hood;
    
    private Angle targetAngle = HoodConstants.STARTING_POS;

    public Hood() {
        this.motor = new TalonFXWrapper(
            hoodMotor,
            HoodConstants.MOTOR_TYPE,
            HoodConstants.MOTOR_CONFIG.withSubsystem(this)
        );

        this.hood = new Pivot(
            HoodConstants.HOOD_CONFIG.withSmartMotorController(motor)
        );
    }

    public void setTargetAngle(Angle angle) {
        this.targetAngle = angle;
        hood.setAngle(angle);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public boolean atPosition() {
        return targetAngle.isNear(hood.getAngle(), HoodConstants.TOLERANCE);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}