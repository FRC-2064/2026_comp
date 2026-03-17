package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.TurretConstants;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
    
    private final CANcoder tbSmall = new CANcoder(TurretConstants.ENCODER_13_ID);
    private final CANcoder tbBig = new CANcoder(TurretConstants.ENCODER_14_ID);
    
    private final EasyCRTConfig easyCRTConfig = new EasyCRTConfig(
        tbSmall.getAbsolutePosition().asSupplier(),
        tbBig.getAbsolutePosition().asSupplier()
    )
        .withCommonDriveGear(1.0, 80, 13, 14)
        .withMechanismRange(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
        .withAbsoluteEncoderOffsets(TurretConstants.ENCODER_13_OFFSET, TurretConstants.ENCODER_14_OFFSET)
        .withMatchTolerance(Rotations.of(0.06))
        .withAbsoluteEncoderInversions(true, true);
    
    private final EasyCRT solver = new EasyCRT(easyCRTConfig);
    
    private final MotionMagicTorqueCurrentFOC mmr = new MotionMagicTorqueCurrentFOC(TurretConstants.STARTING_POS);
    private Angle targetAngle = Degrees.zero();
    
    
    public Turret() {
        var c = new TalonFXConfiguration();
        
        c.Slot0.withKP(TurretConstants.kP)
               .withKI(TurretConstants.kI)
               .withKD(TurretConstants.kD)
               .withKS(TurretConstants.kS);
               
        c.MotionMagic.withMotionMagicCruiseVelocity(TurretConstants.MM_VELOCITY)
                     .withMotionMagicAcceleration(TurretConstants.MM_ACCELERATION);
                     
        c.Feedback.withSensorToMechanismRatio(TurretConstants.GEAR_RATIO);
        
        c.CurrentLimits.withStatorCurrentLimit(TurretConstants.STATOR_LIMIT)
                       .withStatorCurrentLimitEnable(true)
                       .withSupplyCurrentLimit(TurretConstants.SUPPLY_LIMIT)
                       .withSupplyCurrentLimitEnable(true);
                       
        c.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        
        c.SoftwareLimitSwitch.withForwardSoftLimitThreshold(TurretConstants.MAX_ANGLE)
                             .withForwardSoftLimitEnable(true)
                             .withReverseSoftLimitThreshold(TurretConstants.MIN_ANGLE)
                             .withForwardSoftLimitEnable(true);
                             
        turretMotor.getConfigurator().apply(c);
        
        solver.getAngleOptional().ifPresent(mechAngle -> {
            turretMotor.setPosition(mechAngle);
        });
    }
    
    public void setTargetAngle(Angle angle) {
        var clamped = Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), 
                TurretConstants.MIN_ANGLE.in(Degrees),
                TurretConstants.MAX_ANGLE.in(Degrees)
            )
        );
        
        this.targetAngle = clamped;
        turretMotor.setControl(mmr.withPosition(clamped));
    }
    
    public Angle getTargetAngle() {
        return targetAngle;
    }
    
    public Angle getCurrentAngle() {
        return turretMotor.getPosition().getValue();
    }
    
    public boolean atPosition() {
        return turretMotor.getPosition().getValue()
        .isNear(targetAngle, TurretConstants.TOLERANCE);
    }
    
    public void zero() {
        turretMotor.setPosition(Degrees.zero());
    }
    
    
	
}
