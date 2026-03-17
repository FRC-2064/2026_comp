package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;

public class Hood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);

    private final MotionMagicTorqueCurrentFOC mmr = new MotionMagicTorqueCurrentFOC(HoodConstants.STARTING_POS);

    private Angle targetAngle = HoodConstants.STARTING_POS;

    private final DoublePublisher anglePub = NetworkTableInstance.getDefault().getDoubleTopic("turret/angle").publish();
    private final DoublePublisher targetPub = NetworkTableInstance.getDefault().getDoubleTopic("turret/target").publish();

    public Hood() {
        var c = new TalonFXConfiguration();

        c.Slot0.withKP(HoodConstants.kP)
               .withKI(HoodConstants.kI)
               .withKD(HoodConstants.kD);

        c.MotionMagic.withMotionMagicCruiseVelocity(HoodConstants.MM_VELOCITY)
                     .withMotionMagicAcceleration(HoodConstants.MM_ACCELERATION)
                     .withMotionMagicJerk(HoodConstants.JERK);

        c.Feedback.withSensorToMechanismRatio(HoodConstants.GEAR_RATIO);

        c.CurrentLimits.withStatorCurrentLimit(HoodConstants.STATOR_LIMIT)
                       .withStatorCurrentLimitEnable(true)
                       .withSupplyCurrentLimit(HoodConstants.SUPPLY_LIMIT)
                       .withSupplyCurrentLimitEnable(true);

        c.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        c.SoftwareLimitSwitch.withForwardSoftLimitThreshold(HoodConstants.MAX_ANGLE)
                             .withForwardSoftLimitEnable(true)
                             .withReverseSoftLimitThreshold(HoodConstants.MIN_ANGLE)
                             .withReverseSoftLimitEnable(true);

        hoodMotor.getConfigurator().apply(c);
        hoodMotor.setPosition(HoodConstants.STARTING_POS);
    }

    public void setTargetAngle(Angle angle) {
        var clamped = Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                HoodConstants.MIN_ANGLE.in(Degrees),
                HoodConstants.MAX_ANGLE.in(Degrees)
            )
        );

        this.targetAngle = clamped;
        hoodMotor.setControl(mmr.withPosition(clamped));
    }

    public void down() {
        setTargetAngle(HoodConstants.MIN_ANGLE);
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getCurrentAngle() {
        return hoodMotor.getPosition().getValue();
    }

    public void zero() {
        hoodMotor.setPosition(0);
    }

    public boolean atPosition() {
        return hoodMotor.getPosition().getValue()
        .isNear(targetAngle, HoodConstants.TOLERANCE);
    }

    @Override
    public void periodic() {
        anglePub.set(hoodMotor.getPosition().getValue().in(Degrees));
        targetPub.set(targetAngle.in(Degrees));
    }
}
