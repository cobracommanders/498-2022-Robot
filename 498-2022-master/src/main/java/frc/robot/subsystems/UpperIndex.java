package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperIndex extends SubsystemBase{
    public CANSparkMax m_motor = new CANSparkMax(13, MotorType.kBrushless);

    public RelativeEncoder m_encoder = m_motor.getEncoder();
    private State state = State.OFF;
    public enum State {
      FWD(.9),
      REV(-.6),
      OFF(0);
      public double speed;
      private State(double speed) {
        this.speed = speed;
      }
    }
    public UpperIndex() {
        configSpark(m_motor);
      }
      private void configSpark(CANSparkMax motor){
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);
      }

      @Override
      public void periodic() {
        m_motor.set(state.speed);
        // This method will be called once per scheduler run
      }
      public void setState(State state){
        this.state = state;
      }
      public void setSpeed(double speed){
        m_motor.set(speed);
      }
      public void setVoltage(double volts){
        m_motor.setVoltage(volts);
      }
      public void stop(){
          m_motor.setVoltage(0);
      }
}