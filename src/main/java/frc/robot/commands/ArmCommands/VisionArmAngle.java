package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class VisionArmAngle extends Command {
    
    private final ArmSubsystem arm;

    public VisionArmAngle(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    @Override
    public void execute() {
        arm.VisionArmAngle();
    }
}
