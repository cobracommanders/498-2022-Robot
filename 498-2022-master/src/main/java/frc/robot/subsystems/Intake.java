package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    public CANSparkMax m_left = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax m_right = new CANSparkMax(60, MotorType.kBrushless);
    public RelativeEncoder m_leftEncoder = m_left.getEncoder();
    public RelativeEncoder m_rightEncoder = m_right.getEncoder();
    public State state = State.OFF;
    public enum State {
      FWD(.6),
      REV(-.6),
      OFF(0);
      public double speed;
      private State(double speed) {
        this.speed = speed;
      }
    }
    public Intake() {
      configSpark(m_left);
      configSpark(m_right);
      m_right.follow(m_left, true);
    }
      private void configSpark(CANSparkMax motor){
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(20);
        //motor.setOpenLoopRampRate(.05);
      }
      @Override
      public void periodic() {
        m_left.set(state.speed);

        // This method will be called once per scheduler run
      }
      public void setState(State state){
        this.state = state;
      }
      public void setSpeed(double speed){
        m_left.set(speed);
      }
      public void setVoltage(double volts){
        m_left.setVoltage(volts);
      }
      public void pleaseGo(double speed){
        m_left.set(speed);
      }
      public void stop(){
          m_left.setVoltage(0);
      }
}