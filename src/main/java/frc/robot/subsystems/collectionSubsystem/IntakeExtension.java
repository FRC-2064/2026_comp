package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeExtensionConstants;
import frc.robot.utils.RobotConstants;

public class IntakeExtension extends SubsystemBase {

    private final TalonFX extendMotor = new TalonFX(
        IntakeExtensionConstants.MOTOR_ID,
        RobotConstants.CANIVORE
    );

    private final MotionMagicVoltage mmr = new MotionMagicVoltage(
        0
    ).withEnableFOC(true);

    public IntakeExtension() {
        var c = new TalonFXConfiguration();

        c.Slot0.withKP(IntakeExtensionConstants.kP)
            .withKI(IntakeExtensionConstants.kI)
            .withKD(IntakeExtensionConstants.kD);

        c.MotionMagic.withMotionMagicCruiseVelocity(IntakeExtensionConstants.MM_VELOCITY)
        .withMotionMagicAcceleration(IntakeExtensionConstants.MM_ACCELERATION)
        .withMotionMagicJerk(IntakeExtensionConstants.JERK);

        c.Feedback.withSensorToMechanismRatio(IntakeExtensionConstants.GEAR_RATIO);

        c.CurrentLimits.withSupplyCurrentLimit(IntakeExtensionConstants.SUPPLY_LIMIT)
        .withStatorCurrentLimit(IntakeExtensionConstants.STATOR_LIMIT)
        .withStatorCurrentLimitEnable(true);

        c.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        extendMotor.getConfigurator().apply(c);
        extendMotor.setPosition(distanceToRotations(IntakeExtensionConstants.STOW_POS));
    }

    private Angle distanceToRotations(Distance distance) {
            return Rotations.of(distance.in(Inches) / IntakeExtensionConstants.INCHES_PER_ROT.in(Inches));
        }

    public void extend() {
        extendMotor.setControl(mmr.withPosition(distanceToRotations(IntakeExtensionConstants.INTAKE_POS)));
    }

    public void stow() {
        extendMotor.setControl(mmr.withPosition(distanceToRotations(IntakeExtensionConstants.STOW_POS)));
    }

    public void zero() {
        extendMotor.setPosition(0);
    }
}
