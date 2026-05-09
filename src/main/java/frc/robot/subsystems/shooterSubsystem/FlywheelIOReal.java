package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;

public class FlywheelIOReal implements FlywheelIO {
    private final TalonFX flywheelMotor = new TalonFX(FlyWheelConstants.LEADER_ID);
    private final TalonFX followerMotor = new TalonFX(FlyWheelConstants.FOLLOWER_ID);
    private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(RPM.zero());

    public FlywheelIOReal() {
        var config = new TalonFXConfiguration();

        config.Slot0
            .withKP(FlyWheelConstants.P)
            .withKI(FlyWheelConstants.I)
            .withKD(FlyWheelConstants.D)
            .withKS(FlyWheelConstants.S);

        config.CurrentLimits
            .withStatorCurrentLimit(FlyWheelConstants.STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(FlyWheelConstants.SUPPLY_LIMIT);

        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);

        flywheelMotor.getConfigurator().apply(config);

        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        followerMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRpm = flywheelMotor.getVelocity().getValue().in(RPM);
        inputs.leaderAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setTargetSpeed(edu.wpi.first.units.measure.AngularVelocity speed) {
        flywheelMotor.setControl(request.withVelocity(speed));
        followerMotor.setControl(request.withVelocity(speed));
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
        followerMotor.stopMotor();
    }
}
