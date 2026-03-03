package frc.robot.subsystems.collectionSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import frc.robot.subsystems.shooterSubsystem.ShooterConstants.HoodConstants;
import frc.robot.utils.RobotConstants;

public class IntakeExtension extends SubsystemBase {
    private final TalonFX extendMotor = new TalonFX(IntakeConstants.EXTEND_ID, RobotConstants.CANIVORE);

    private final TalonFXConfiguration config = new TalonFXConfiguration()
    .withSlot0(
        new Slot0Configs()
        .withKP(50)
        .withKI(0)
        .withKD(0)
    )
    .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive)
    );

    private VelocityTorqueCurrentFOC pos = new VelocityTorqueCurrentFOC(0);

    private Debouncer debounce = new Debouncer(0.25);

    public IntakeExtension() {
        extendMotor.getConfigurator().apply(config);
    }

    public void extend() {
        extendMotor.set(0.5);
    }

    public void stow() {
   extendMotor.set(-0.5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("extension/number", extendMotor.getPosition().getValueAsDouble());
    }
}
