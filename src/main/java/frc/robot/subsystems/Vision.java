package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Vision extends SubsystemBase {

    private final NetworkTable table;
    private final SwerveSubsystem swerveSubsystem;

    public Vision(SwerveSubsystem swerveSubsystem) {
       this.swerveSubsystem = swerveSubsystem;

       table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public Command alignWithAprilTag() {
        // NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");

        // double horizontalOffset = tx.getDouble(0.0);
        double verticalOffset = ty.getDouble(0.0);

        // SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
        SmartDashboard.putNumber("Vertical Offset", verticalOffset);

        
        DoubleSupplier translationX = () -> 0.0;
        DoubleSupplier translationY = () -> verticalOffset;
        DoubleSupplier angularRotationX = () -> 0.0;

        return swerveSubsystem.driveCommand(translationX, translationY, angularRotationX);
    }

    public void periodic() {
        alignWithAprilTag();
    }
}
