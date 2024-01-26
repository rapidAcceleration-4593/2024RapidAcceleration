package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    private final NetworkTable table;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void periodic() {
        int selectedPipeline = 2;

        table.getEntry("pipeline").getNumber(selectedPipeline);

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");

        double horizontalOffset = tx.getDouble(0);
        double verticalOffset = ty.getDouble(0);

        SmartDashboard.getNumber("Horizontal Offset", horizontalOffset);
        SmartDashboard.getNumber("Vertical Offset", verticalOffset);
    }
}
