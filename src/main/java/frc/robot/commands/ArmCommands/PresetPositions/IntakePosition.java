package frc.robot.commands.ArmCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakePosition extends Command {

    private final ArmSubsystem arm;

    public IntakePosition(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }
    
    public void execute() {
        arm.IntakePosition();
    }
}
