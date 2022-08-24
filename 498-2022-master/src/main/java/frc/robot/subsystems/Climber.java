package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public CANSparkMax m_left = new CANSparkMax(14, MotorType.kBrushless);
    public CANSparkMax m_right = new CANSparkMax(15, MotorType.kBrushless);
    public PneumaticHub m_pneumaticHub = new PneumaticHub(50);
    public Compressor m_compressor;
    public DoubleSolenoid m_solenoid;
    public RelativeEncoder m_leftEncoder = m_left.getEncoder();
    public RelativeEncoder m_rightEncoder = m_right.getEncoder();
    public Position position = Position.DOWN;
    public enum Position {
      MID(-120),
      LOW(-100),
      DOWN(0);
      public double setPoint;
      private Position(double setPoint) {
        this.setPoint = setPoint;
      }
    }
    public Climber() {
        configSpark(m_left);
        configSpark(m_right);
        m_right.follow(m_left, true);
        m_compressor = m_pneumaticHub.makeCompressor();
        m_compressor.enableDigital();
        m_solenoid = m_pneumaticHub.makeDoubleSolenoid(0, 1);
        m_solenoid.set(Value.kReverse);
      }
      private void configSpark(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.setClosedLoopRampRate(.05);
      }
      public double rightEncoderValue(){
        return -m_rightEncoder.getPosition();
      }
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("L Climber Value", m_leftEncoder.getPosition());
        //SmartDashboard.putNumber("R Climber Value", m_rightEncoder.getPosition());

      }
      public void setSpeed(double speed){
        m_left.set(speed);
      }
      public void setState(Position position){
        this.position=position;
      }
      public void setVoltage(double volts){
        m_left.setVoltage(volts);
      }
      public void stop(){
          m_left.setVoltage(0);
      }
}