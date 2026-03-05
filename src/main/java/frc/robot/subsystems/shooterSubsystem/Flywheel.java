package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;

public class Flywheel extends SubsystemBase {
    private final TalonFX flywheelMotor = new TalonFX(FlyWheelConstants.LEADER_ID);
    private final TalonFX followerMotor = new TalonFX(FlyWheelConstants.FOLLOWER_ID);

    private final VelocityTorqueCurrentFOC fr = new VelocityTorqueCurrentFOC(RPM.zero());
    private AngularVelocity targetSpeed = RPM.zero();

    public Flywheel() {
        SmartDashboard.putNumber("shooter/targetSpeedTuning", 0);
        var c = new TalonFXConfiguration();
        var fc = new TalonFXConfiguration();

        c.Slot0
        .withKP(FlyWheelConstants.P)
        .withKI(FlyWheelConstants.I)
        .withKD(FlyWheelConstants.D)
        .withKS(FlyWheelConstants.S);

        c.CurrentLimits.withStatorCurrentLimit(FlyWheelConstants.STATOR_LIMIT)
        .withStatorCurrentLimitEnable(true);

        c.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

        fc.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);


        flywheelMotor.getConfigurator().apply(c);
        followerMotor.getConfigurator().apply(fc);

        // followerMotor.setControl(new StrictFollower(FlyWheelConstants.LEADER_ID));

        followerMotor.setControl(new Follower(FlyWheelConstants.LEADER_ID, MotorAlignmentValue.Opposed));
    }


    public void setTargetSpeed(AngularVelocity speed) {
        var s = RPM.of(
            MathUtil.clamp(
                speed.in(RPM),
                FlyWheelConstants.MIN_VELOCITY.in(RPM),
                FlyWheelConstants.MAX_VELOCITY.in(RPM)
            )
        );
        this.targetSpeed = s;
        flywheelMotor.setControl(fr.withVelocity(s));
        // followerMotor.setControl(new Follower(FlyWheelConstants.LEADER_ID, MotorAlignmentValue.Opposed));

    }

    public void stop() {
        this.targetSpeed = RPM.zero();
        flywheelMotor.stopMotor();
        // followerMotor.setControl(new Follower(FlyWheelConstants.LEADER_ID, MotorAlignmentValue.Opposed));

    }

    public boolean isUpToSpeed() {
        return flywheelMotor.getVelocity().getValue().isNear(targetSpeed, FlyWheelConstants.TOLERANCE);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }

    public AngularVelocity getVelocity() {
        return flywheelMotor.getVelocity().getValue();
    }

    private void telemetry() {
        SmartDashboard.putNumber("shooter/velocity", getVelocity().in(RPM));
        SmartDashboard.putNumber("shooter/target", targetSpeed.in(RPM));
    }

    @Override
    public void periodic() {
        telemetry();
    }
}
