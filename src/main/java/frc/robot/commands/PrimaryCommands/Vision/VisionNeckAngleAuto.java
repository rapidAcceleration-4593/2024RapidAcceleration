package frc.robot.commands.PrimaryCommands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class VisionNeckAngleAuto extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public VisionNeckAngleAuto(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.VisionNeckAngleAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}