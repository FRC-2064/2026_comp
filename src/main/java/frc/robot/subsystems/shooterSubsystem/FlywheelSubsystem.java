package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.FlyWheelConstants;
import frc.robot.utils.RobotConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX flywheelMotor = new TalonFX(FlyWheelConstants.LEADER_ID);
    private final TalonFX followerMotor = new TalonFX(FlyWheelConstants.FOLLOWER_ID);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    FlyWheelConstants.kP,
                    FlyWheelConstants.kI,
                    FlyWheelConstants.kD)
            .withGearing(FlyWheelConstants.GEARING)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(FlyWheelConstants.STATOR_LIMIT)
            .withFollowers(new Pair<>(followerMotor, true))
            .withMotorInverted(true)
            .withTelemetry("ShooterMotor", RobotConstants.GetTelemetry());

    private final SmartMotorController motor = new TalonFXWrapper(
            flywheelMotor,
            FlyWheelConstants.MOTORS,
            motorConfig);

    private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motor)
            .withDiameter(FlyWheelConstants.WHEEL_DIAMETER)
            .withMass(FlyWheelConstants.WHEEL_MASS)
            .withLowerSoftLimit(FlyWheelConstants.MIN_VELOCITY)
            .withUpperSoftLimit(FlyWheelConstants.MAX_VELOCITY)
            .withTelemetry("Shooter", RobotConstants.GetTelemetry());

    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    private AngularVelocity targetSpeed = FlyWheelConstants.MIN_VELOCITY;

    private VelocityTorqueCurrentFOC flywheelCont = new VelocityTorqueCurrentFOC(RPM.zero());

    public FlywheelSubsystem() {
flywheelMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
            .withKP(4)
            .withKI(0)
            .withKD(0)
        ).withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        ));

        setDefaultCommand(buildFlywheelDefault());
    }

    private Command buildFlywheelDefault() {
        // return flywheel.setSpeed(() -> this.targetSpeed)
        // .onlyWhile(this::shouldSpin)
        // .andThen(Commands.runOnce(this::coastOut))
        // .andThen(Commands.waitUntil(this::shouldSpin))
        // .repeatedly()
        // .withName("FlywheelDefault");

        return new RunCommand(() -> flywheelMotor.setControl(flywheelCont), this);
    }

    private boolean shouldSpin() {
        return Math.abs(this.targetSpeed.in(RPM)) >= 100;
    }

    private void coastOut() {
        flywheelMotor.setControl(new DutyCycleOut(0));
    }

    public void setTargetSpeed(AngularVelocity speed) {
        this.targetSpeed = speed;
    }

    public void stop() {
        this.targetSpeed = FlyWheelConstants.MIN_VELOCITY;
    }

    public boolean isUpToSpeed() {
        return flywheelMotor.getVelocity().isNear(targetSpeed, FlyWheelConstants.TOLERANCE);
    }

    public AngularVelocity getTargetSpeed() {
        return targetSpeed;
    }

    public AngularVelocity getVelocity() {
        return flywheel.getSpeed();
    }

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
        flywheelCont.withVelocity(targetSpeed);
        SmartDashboard.putBoolean("shooter/isready", isUpToSpeed());

    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
