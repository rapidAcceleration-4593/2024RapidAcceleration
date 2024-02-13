package frc.robot.commands.ArmCommands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ClimberStop extends Command {
    
    private final ArmSubsystem arm;

    public ClimberStop(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.ClimberStop();
    }
}
