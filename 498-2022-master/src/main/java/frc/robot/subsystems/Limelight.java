package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  public NetworkTableEntry camMode = table.getEntry("camMode");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  double x, y, area;
  public enum CamMode {
    VISION(0),
    DRIVER(1);
    public double val;
    private CamMode(double val) {
      this.val = val;
    }
  }
  public enum LEDMode {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);
    public double val;
    private LEDMode(double val) {
      this.val = val;
    }
  }
    public Limelight() {
    }
    @Override
    public void periodic() {
      x = tx.getDouble(0);
      y = ty.getDouble(0);
      area = ta.getDouble(0);
    }
    public void setCamMode(CamMode camMode){
      this.camMode.setNumber(camMode.val);
    }
    public void setLEDMode(LEDMode ledMode){
      this.ledMode.setNumber(ledMode.val);
    }
}