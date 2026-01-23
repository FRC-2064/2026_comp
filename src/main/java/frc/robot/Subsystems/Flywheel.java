package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;

public class Flywheel extends SubsystemBase {

    
    private AngularVelocity targetSpeed = RPM.of(0);
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)

    .withControlMode(ControlMode.CLOSED_LOOP)
    // Feedback Constants (PID Constants)
    .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    // Feedforward Constants
    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    // Telemetry name and verbosity level
    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
    // Gearing from the motor rotor to final shaft.
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    // Motor properties to prevent over currenting.
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(60));

    // defining flywheel motor
    private final TalonFX flywheelMotor = new TalonFX(1);
 
    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController flywheelMotorSMC = new TalonFXWrapper(flywheelMotor, DCMotor.getKrakenX60Foc(1), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(flywheelMotorSMC)
    // Diameter of the flywheel.
    .withDiameter(Inches.of(4))
    // Mass of the flywheel.
    .withMass(Pounds.of(1))
    // Maximum speed of the shooter.
    .withUpperSoftLimit(RPM.of(1000))
    // Telemetry name and verbosity for the arm.
    .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
  
    // Shooter Mechanism
    private FlyWheel shooter = new FlyWheel(shooterConfig);

    // Congiguring the motor
    public Flywheel(){}

    // setting up methods and vars

    public boolean isUpTospeed(){
        return shooter.getSpeed().isNear(targetSpeed, RPM.of(10));   
    }

   public void setSpeed(AngularVelocity speed){
        targetSpeed = speed;
        shooter.setSpeed(targetSpeed);
   }

   public AngularVelocity getTargetSpeed(){
    return targetSpeed;
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }
}