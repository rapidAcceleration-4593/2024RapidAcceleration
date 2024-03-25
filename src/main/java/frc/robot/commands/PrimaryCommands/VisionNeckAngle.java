package frc.robot.commands.PrimaryCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class VisionNeckAngle extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public VisionNeckAngle(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.VisionNeckAngle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
