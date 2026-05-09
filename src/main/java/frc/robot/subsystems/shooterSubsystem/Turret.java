package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Angle targetAngle = TurretConstants.STARTING_POS;

    public Turret(TurretIO io) {
        this.io = io;
        setTargetAngle(TurretConstants.STARTING_POS);
    }

    public void setTargetAngle(Angle angle) {
        targetAngle = Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                TurretConstants.MIN_ANGLE.in(Degrees),
                TurretConstants.MAX_ANGLE.in(Degrees)
            )
        );
        io.setTargetAngle(targetAngle);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return Degrees.of(inputs.positionDeg);
    }

    public boolean atPosition() {
        return getCurrentAngle().isNear(targetAngle, TurretConstants.TOLERANCE);
    }

    public void zero() {
        io.zeroPosition();
    }

    public void setEncoderZero() {
        zero();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/TargetAngleDeg", targetAngle.in(Degrees));

        SmartDashboard.putNumber("turret/angle", inputs.positionDeg);
        SmartDashboard.putString("turret/status", inputs.absoluteSolveStatus);
    }
}
