package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase{
    public DigitalInput m_topStorageSensor;
    public DigitalInput m_bottomStorageSensor;
    public Spark m_blinkin;
    public enum Color {
      RED(.61),//-.25 for breath
      BLUE(.87),//-.23 for breath
      GREEN(.77),
      WHITE(-.21),
      GRAY(-.13);
      public double val;
      private Color(double val) {
        this.val = val;
      }
    }
    public Blinkin() {
      m_topStorageSensor = new DigitalInput(6);
      m_bottomStorageSensor = new DigitalInput(7);
      m_blinkin = new Spark(0);
    }
    @Override
    public void periodic() {
      SmartDashboard.putBoolean("TopCargoSensor", m_topStorageSensor.get());
    }
    public void setColor(Color color){
      set(color.val);
    }
    public boolean getSensorData(DigitalInput sensor){
      return sensor.get();
    }
    public void set(double val) {
      if ((val >= -1.0) && (val <= 1.0)) {
        m_blinkin.set(val);
      }
    }
}