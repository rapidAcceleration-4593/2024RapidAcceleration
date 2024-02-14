package frc.robot.commands.ArmCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SubwooferPosition extends Command {
    
    private final ArmSubsystem arm;

    public SubwooferPosition(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.SubwooferPosition();
    }
}
