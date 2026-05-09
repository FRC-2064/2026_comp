// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.HubShiftUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command auto;

    private final RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", "2026_comp");
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());

        Logger.addDataReceiver(new NT4Publisher());
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
        }
        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        HubShiftUtil.updateDashboardTelemetry();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        auto = robotContainer.getAutonomousCommand();

        if (auto != null) {
            CommandScheduler.getInstance().schedule(auto);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (auto != null) {
            CommandScheduler.getInstance().cancel(auto);
        }
        HubShiftUtil.initialize();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
