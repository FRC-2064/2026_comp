package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import frc.robot.utils.RobotConstants;

public class IntakeExtension extends SubsystemBase {

    private final TalonFX extendMotor = new TalonFX(
        IntakeConstants.EXTEND_ID,
        RobotConstants.CANIVORE
    );

    private final MotionMagicVoltage mmr = new MotionMagicVoltage(
        0
    ).withEnableFOC(true);

    public IntakeExtension() {
        var c = new TalonFXConfiguration();

        c.Slot0.withKP(IntakeConstants.P)
            .withKI(IntakeConstants.I)
            .withKD(IntakeConstants.D);

        c.MotionMagic.withMotionMagicCruiseVelocity(IntakeConstants.MM_CRUISE_VEL)
        .withMotionMagicAcceleration(IntakeConstants.MM_ACCEL);

        c.Feedback.withSensorToMechanismRatio(IntakeConstants.RACK_GEARING);

        c.CurrentLimits.withStatorCurrentLimit(IntakeConstants.STATOR_LIMIT)
        .withStatorCurrentLimitEnable(true);

        c.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        extendMotor.getConfigurator().apply(c);
        extendMotor.setPosition(IntakeConstants.STOW);
    }

    public void extend() {
        extendMotor.setControl(mmr.withPosition(IntakeConstants.INTAKE));
    }

    public void stow() {
        extendMotor.setControl(mmr.withPosition(IntakeConstants.STOW));
    }
}
