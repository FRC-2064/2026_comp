package frc.robot.subsystems.shooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShooterPosition;

public class Shooter extends SubsystemBase {    
    private final Hood hood;
    private final FlywheelSubsystem flywheel;
    
    public Shooter(Hood hood, FlywheelSubsystem flywheel){
        this.hood = hood;
        this.flywheel = flywheel;
    }

    public Command shootCommand(ShooterCalc shooterCalc){
        Supplier<ShooterPosition> shooterPositionSupplier = shooterCalc.getShooterPositionSupplier();

        return this.run(() -> {
            hood.setTargetAngle(shooterPositionSupplier.get().hood());
            flywheel.setVelocity(shooterPositionSupplier.get().flywheel());
        });
    }

    public Command stowCommand(){
        return this.runOnce(() ->{
            flywheel.setVelocity(RPM.of(0));
            hood.setTargetAngle(Degrees.of(0));
        });
    }

    public Boolean atPosition(){
        return hood.atPosition() && flywheel.isUpToSpeed();
    }
}