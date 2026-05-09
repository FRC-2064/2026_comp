package frc.robot.subsystems.collectionSubsystem;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collectionSubsystem.CollectionConstants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends SubsystemBase {
    private final IntakeExtensionIO io;
    private final IntakeExtensionIOInputsAutoLogged inputs =
        new IntakeExtensionIOInputsAutoLogged();

    public IntakeExtension(IntakeExtensionIO io) {
        this.io = io;
    }

    public void extend() {
        io.setTargetPosition(IntakeConstants.INTAKE);
        SmartDashboard.putNumber("extension/target", IntakeConstants.INTAKE.in(Rotations));
        Logger.recordOutput("IntakeExtension/TargetRotations", IntakeConstants.INTAKE.in(Rotations));
    }

    public void stow() {
        io.setTargetPosition(IntakeConstants.STOW);
        SmartDashboard.putNumber("extension/target", IntakeConstants.STOW.in(Rotations));
        Logger.recordOutput("IntakeExtension/TargetRotations", IntakeConstants.STOW.in(Rotations));
    }

    public void agitate() {
        io.setTargetPosition(IntakeConstants.AGITATE);
        SmartDashboard.putNumber("extension/target", IntakeConstants.AGITATE.in(Rotations));
        Logger.recordOutput("IntakeExtension/TargetRotations", IntakeConstants.AGITATE.in(Rotations));
    }

    public void zero() {
        io.zeroPosition();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeExtension", inputs);
        SmartDashboard.putNumber("extension/current", inputs.positionRotations);
    }
}
