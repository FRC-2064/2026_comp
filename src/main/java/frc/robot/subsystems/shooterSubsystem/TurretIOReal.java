package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import java.util.Optional;

public class TurretIOReal implements TurretIO {
    private enum AbsoluteSolveStatus {
        MATCHED,
        NO_MATCH
    }

    private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
    private final CANcoder throughBoreSmall = new CANcoder(TurretConstants.ENCODER_13_ID);
    private final CANcoder throughBoreLarge = new CANcoder(TurretConstants.ENCODER_14_ID);
    private final MotionMagicVoltage request =
        new MotionMagicVoltage(TurretConstants.STARTING_POS).withEnableFOC(true);

    private AbsoluteSolveStatus absoluteSolveStatus = AbsoluteSolveStatus.NO_MATCH;

    public TurretIOReal() {
        var config = new TalonFXConfiguration();

        config.Slot0
            .withKP(TurretConstants.kP)
            .withKI(TurretConstants.kI)
            .withKD(TurretConstants.kD);

        config.MotionMagic
            .withMotionMagicCruiseVelocity(TurretConstants.MAX_VEL)
            .withMotionMagicAcceleration(TurretConstants.MAX_ACCEL);

        config.Feedback.withSensorToMechanismRatio(TurretConstants.GEAR_RATIO);

        config.CurrentLimits
            .withStatorCurrentLimit(TurretConstants.STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true);

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        turretMotor.getConfigurator().apply(config);
        calculateAbsoluteAngle().ifPresent(turretMotor::setPosition);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionDeg = turretMotor.getPosition().getValue().in(Degrees);
        inputs.appliedVolts = turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.absoluteSmallRotations =
            throughBoreSmall.getAbsolutePosition().getValue().in(Rotations);
        inputs.absoluteLargeRotations =
            throughBoreLarge.getAbsolutePosition().getValue().in(Rotations);
        inputs.absoluteSolveStatus = absoluteSolveStatus.name();
    }

    @Override
    public void setTargetAngle(Angle angle) {
        turretMotor.setControl(request.withPosition(angle));
    }

    @Override
    public void zeroPosition() {
        turretMotor.setPosition(Degrees.zero());
    }

    private Optional<Angle> calculateAbsoluteAngle() {
        double rawSmall = throughBoreSmall.getAbsolutePosition().getValue().in(Rotations);
        double rawLarge = throughBoreLarge.getAbsolutePosition().getValue().in(Rotations);

        double bestError = Double.POSITIVE_INFINITY;
        Double bestMechanismRotations = null;

        for (boolean subtractSmallOffset : new boolean[] {true, false}) {
            for (boolean subtractLargeOffset : new boolean[] {true, false}) {
                double correctedSmall = correctEncoder(
                    rawSmall,
                    TurretConstants.ENCODER_13_OFFSET.in(Rotations),
                    TurretConstants.ENCODER_13_INVERTED,
                    subtractSmallOffset
                );
                double correctedLarge = correctEncoder(
                    rawLarge,
                    TurretConstants.ENCODER_14_OFFSET.in(Rotations),
                    TurretConstants.ENCODER_14_INVERTED,
                    subtractLargeOffset
                );

                for (int wrap = -8; wrap <= 8; wrap++) {
                    double mechanismRotations =
                        (correctedSmall + wrap) / TurretConstants.ENCODER_13_RATIO;
                    if (mechanismRotations < TurretConstants.MIN_ANGLE.in(Rotations) - 0.1
                        || mechanismRotations > TurretConstants.MAX_ANGLE.in(Rotations) + 0.1) {
                        continue;
                    }

                    double expectedLarge = normalizeRotation(
                        mechanismRotations * TurretConstants.ENCODER_14_RATIO
                    );
                    double error = Math.abs(rotationError(expectedLarge, correctedLarge));

                    if (error < bestError) {
                        bestError = error;
                        bestMechanismRotations = mechanismRotations;
                    }
                }
            }
        }

        if (bestMechanismRotations != null
            && bestError <= TurretConstants.ENCODER_MATCH_TOLERANCE.in(Rotations)) {
            absoluteSolveStatus = AbsoluteSolveStatus.MATCHED;
            return Optional.of(Rotations.of(bestMechanismRotations));
        }

        absoluteSolveStatus = AbsoluteSolveStatus.NO_MATCH;
        return Optional.empty();
    }

    private static double correctEncoder(
        double rawRotations,
        double offsetRotations,
        boolean inverted,
        boolean subtractOffset
    ) {
        double corrected = subtractOffset
            ? rawRotations - offsetRotations
            : rawRotations + offsetRotations;
        if (inverted) {
            corrected = -corrected;
        }
        return normalizeRotation(corrected);
    }

    private static double normalizeRotation(double rotations) {
        double normalized = rotations % 1.0;
        return normalized < 0.0 ? normalized + 1.0 : normalized;
    }

    private static double rotationError(double expected, double measured) {
        double error = expected - measured;
        if (error > 0.5) {
            error -= 1.0;
        } else if (error < -0.5) {
            error += 1.0;
        }
        return error;
    }
}
