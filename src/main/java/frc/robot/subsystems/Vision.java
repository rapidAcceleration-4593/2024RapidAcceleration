package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public Vision() {
       
    }

    public void alignWithAprilTag() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");

        double horizontalOffset = tx.getDouble(0.0);
        double verticalOffset = ty.getDouble(0.0);

        System.out.println("Vertical Offset: " + horizontalOffset);
        System.out.println("Horizontal Offset: " + verticalOffset);
    }

    public void periodic() {
        alignWithAprilTag();
    }
}
