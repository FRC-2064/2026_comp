package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeRollerConstants;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.ROLLER_ID);

    private final DutyCycleOut rr = new DutyCycleOut(0).withEnableFOC(true);

    public IntakeRollers() {
       var c = new TalonFXConfiguration();

       c.CurrentLimits.withStatorCurrentLimit(IntakeRollerConstants.STATOR_LIMIT)
       .withStatorCurrentLimitEnable(true);

       c.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
       .withInverted(InvertedValue.CounterClockwise_Positive);

       rollerMotor.getConfigurator().apply(c);
    }



    public void intake() {
        rollerMotor.setControl(rr.withOutput(IntakeRollerConstants.INTAKE));
    }

    public void outtake() {
        rollerMotor.setControl(rr.withOutput(IntakeRollerConstants.OUTTAKE));
    }

    public void stop() {
        rollerMotor.setControl(rr.withOutput(0));
    }
}
