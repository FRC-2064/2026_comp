package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;

public class HoodIOReal implements HoodIO {
    private final TalonFX hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);
    private final MotionMagicVoltage request =
        new MotionMagicVoltage(HoodConstants.STARTING_POS).withEnableFOC(true);

    public HoodIOReal() {
        var config = new TalonFXConfiguration();

        config.Slot0
            .withKP(HoodConstants.kP)
            .withKI(HoodConstants.kI)
            .withKD(HoodConstants.kD);

        config.MotionMagic
            .withMotionMagicCruiseVelocity(HoodConstants.MAX_VELOCITY)
            .withMotionMagicAcceleration(HoodConstants.MAX_ACCELERATION);

        config.Feedback.withSensorToMechanismRatio(HoodConstants.GEAR_RATIO);

        config.CurrentLimits
            .withStatorCurrentLimit(HoodConstants.STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true);

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setPosition(HoodConstants.STARTING_POS);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionDeg = hoodMotor.getPosition().getValue().in(Degrees);
        inputs.appliedVolts = hoodMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Angle angle) {
        hoodMotor.setControl(request.withPosition(angle));
    }

    @Override
    public void zeroPosition() {
        hoodMotor.setPosition(Degrees.zero());
    }
}
