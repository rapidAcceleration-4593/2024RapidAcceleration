package frc.robot.commands.NeckCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class VisionNeckAngle extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public VisionNeckAngle(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.VisionNeckAngle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
