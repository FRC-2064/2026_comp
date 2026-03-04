package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

        c.Slot0
        .withKP(FlyWheelConstants.P)
        .withKI(FlyWheelConstants.I)
        .withKD(FlyWheelConstants.D)
        .withKS(FlyWheelConstants.S);

        c.CurrentLimits.withStatorCurrentLimit(FlyWheelConstants.STATOR_LIMIT)
        .withStatorCurrentLimitEnable(true);

        c.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

        flywheelMotor.getConfigurator().apply(c);
        followerMotor.getConfigurator().apply(c);

        followerMotor.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        setDefaultCommand(buildFlywheelDefault());
    }

    private Command buildFlywheelDefault() {
        return new RunCommand(() ->flywheelMotor.setControl(fr), this).withName("FlywheelDefault");
    }

    public void setTargetSpeed(AngularVelocity speed) {
        this.targetSpeed = speed;
    }

    public void stop() {
        this.targetSpeed = RPM.zero();
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
        fr.withVelocity(targetSpeed);
        var speed = SmartDashboard.getNumber("shooter/targetSpeedTuning", 0);
        fr.withVelocity(RPM.of(speed));
        telemetry();
    }
}
