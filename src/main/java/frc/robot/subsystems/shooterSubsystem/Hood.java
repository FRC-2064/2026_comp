package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private Angle targetAngle = HoodConstants.STARTING_POS;

    public Hood(HoodIO io) {
        this.io = io;
        setTargetAngle(HoodConstants.STARTING_POS);
    }

    public void setTargetAngle(Angle angle) {
        targetAngle = Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                HoodConstants.MIN_ANGLE.in(Degrees),
                HoodConstants.MAX_ANGLE.in(Degrees)
            )
        );
        io.setTargetAngle(targetAngle);
    }

    public void down() {
        setTargetAngle(HoodConstants.MIN_ANGLE);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return Degrees.of(inputs.positionDeg);
    }

    public void zero() {
        io.zeroPosition();
    }

    public boolean atPosition() {
        return getCurrentAngle().isNear(targetAngle, HoodConstants.TOLERANCE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        Logger.recordOutput("Hood/TargetAngleDeg", targetAngle.in(Degrees));

        SmartDashboard.putNumber("hood/desiredAngle", targetAngle.in(Degrees));
        SmartDashboard.putNumber("hood/current", inputs.positionDeg);
    }
}
