package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private AngularVelocity targetSpeed = RPM.zero();

    public Flywheel(FlywheelIO io) {
        this.io = io;
        SmartDashboard.putNumber("shooter/targetSpeedTuning", 0);
    }

    public void setTargetSpeed(AngularVelocity speed) {
        targetSpeed = RPM.of(
            MathUtil.clamp(
                speed.in(RPM),
                FlyWheelConstants.MIN_VELOCITY.in(RPM),
                FlyWheelConstants.MAX_VELOCITY.in(RPM)
            )
        );
        io.setTargetSpeed(targetSpeed);
    }

    public void stop() {
        targetSpeed = RPM.zero();
        io.stop();
    }

    public boolean isUpToSpeed() {
        return getVelocity().isNear(targetSpeed, FlyWheelConstants.TOLERANCE);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.velocityRpm);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        Logger.recordOutput("Flywheel/TargetSpeedRPM", targetSpeed.in(RPM));

        SmartDashboard.putNumber("shooter/velocity", inputs.velocityRpm);
        SmartDashboard.putNumber("shooter/target", targetSpeed.in(RPM));
    }
}
