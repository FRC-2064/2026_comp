package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private TalonFX motor = new TalonFX(0);

    private TalonFXConfiguration config = new TalonFXConfiguration(); 
    
    public Indexer() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
    }

    public void feed() {
        motor.set(0.75);
    }

    public void stop() {
        motor.set(0.0);
    }

    public void outtake() {
        motor.set(-0.5);
    }

}
