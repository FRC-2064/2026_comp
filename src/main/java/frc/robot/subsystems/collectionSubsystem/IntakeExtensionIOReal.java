package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import frc.robot.utils.RobotConstants;

public class IntakeExtensionIOReal implements IntakeExtensionIO {
    private final TalonFX extendMotor = new TalonFX(
        IntakeConstants.EXTEND_ID,
        RobotConstants.CANIVORE
    );
    private final MotionMagicVoltage request =
        new MotionMagicVoltage(0).withEnableFOC(true);

    public IntakeExtensionIOReal() {
        var config = new TalonFXConfiguration();

        config.Slot0
            .withKP(IntakeConstants.P)
            .withKI(IntakeConstants.I)
            .withKD(IntakeConstants.D);

        config.MotionMagic
            .withMotionMagicCruiseVelocity(IntakeConstants.MM_CRUISE_VEL)
            .withMotionMagicAcceleration(IntakeConstants.MM_ACCEL);

        config.Feedback.withSensorToMechanismRatio(IntakeConstants.RACK_GEARING);

        config.CurrentLimits
            .withSupplyCurrentLimit(IntakeConstants.SUPPLY_LIMIT)
            .withStatorCurrentLimit(IntakeConstants.STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true);

        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        extendMotor.getConfigurator().apply(config);
        extendMotor.setPosition(IntakeConstants.STOW);
    }

    @Override
    public void updateInputs(IntakeExtensionIOInputs inputs) {
        inputs.positionRotations = extendMotor.getPosition().getValue().in(Rotations);
        inputs.appliedVolts = extendMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setTargetPosition(Angle position) {
        extendMotor.setControl(request.withPosition(position));
    }

    @Override
    public void zeroPosition() {
        extendMotor.setPosition(Rotations.zero());
    }
}
