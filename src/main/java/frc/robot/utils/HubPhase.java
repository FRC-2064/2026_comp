package frc.robot.utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Liana.LianaHelpers;

public class HubPhase extends SubsystemBase{
    String gameData = DriverStation.getGameSpecificMessage();


    @Override
    public void periodic() {
        if(gameData.length() > 0){
            switch (gameData.charAt(0)){
                case 'B' :
                    if(Timer.getFPGATimestamp() <= 30){
                        LianaHelpers.currentActiveHub("R");
                    }
                    if(Timer.getFPGATimestamp() <= 55){
                        LianaHelpers.currentActiveHub("B");
                    }
                    if(Timer.getFPGATimestamp() <= 80){
                        LianaHelpers.currentActiveHub("R");
                    }
                    if(Timer.getFPGATimestamp() <= 105){
                        LianaHelpers.currentActiveHub("B");
                    }
                    break;
                case 'R' :
                    if(Timer.getFPGATimestamp() <= 30){
                        LianaHelpers.currentActiveHub("B");
                    }
                    if(Timer.getFPGATimestamp() <= 55){
                        LianaHelpers.currentActiveHub("R");
                    }
                    if(Timer.getFPGATimestamp() <= 80){
                        LianaHelpers.currentActiveHub("B");
                    }
                    if(Timer.getFPGATimestamp() <= 105){
                        LianaHelpers.currentActiveHub("R");
                    }
                    break;
                default :
                    //This is corrupt data
                    break;
            }
        } else {
            //when the data has not been sent yet
        }
    }
}