package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignWithAprilTag extends Command {
    
    private final NetworkTable table;
    private final SwerveSubsystem swerve;

    public AlignWithAprilTag(SwerveSubsystem swerve) {
        // m_vision = visionPassedIn;
        // addRequirements(visionPassedIn);
        
        this.swerve = swerve;

        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void execute() {
        NetworkTableEntry ty = table.getEntry("ty");
        double verticalOffset = ty.getDouble(0.0);

        System.out.println(verticalOffset);

        swerve.drive(new Translation2d(0.0, -verticalOffset * 0.1), -verticalOffset * 0.1, false);
    }
}
