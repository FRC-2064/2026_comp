package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeRollerConstants;

public class IntakeRollersIOReal implements IntakeRollersIO {
    private final TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.ROLLER_ID);
    private final DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(true);

    public IntakeRollersIOReal() {
        var config = new TalonFXConfiguration();

        config.CurrentLimits
            .withSupplyCurrentLimit(IntakeRollerConstants.SUPPLY_LIMIT)
            .withStatorCurrentLimit(IntakeRollerConstants.STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true);

        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);

        rollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.appliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setOutput(double output) {
        rollerMotor.setControl(request.withOutput(output));
    }
}
