package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPIDSubsystem;

public class PIDHoldArmCommand extends Command {

    private final ArmPIDSubsystem armPIDSubsystem;
    
    public PIDHoldArmCommand(ArmPIDSubsystem armPIDSubsystem) {
        this.armPIDSubsystem = armPIDSubsystem;
        addRequirements(armPIDSubsystem);
    }

    @Override
    public void initialize() {
        armPIDSubsystem.setSetpoint(armPIDSubsystem.getMeasurement()); // Current position
    }

    @Override
    public void execute() {
        armPIDSubsystem.useOutput(armPIDSubsystem.getController().calculate(armPIDSubsystem.getMeasurement(), armPIDSubsystem.getSetpoint()), armPIDSubsystem.getSetpoint());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
