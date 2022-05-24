package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LowerIndex extends SubsystemBase{
    public CANSparkMax m_motor = new CANSparkMax(16, MotorType.kBrushless);
    public RelativeEncoder m_encoder = m_motor.getEncoder();
    private State state = State.OFF;

    public enum State {
      FWD(.8),
      REV(-.8),
      STORE(-.05),//-.025
      RESTORE(.0),//.1
      OFF(0);
      public double speed;
      private State(double speed) {
        this.speed = speed;
      }
    }
    public LowerIndex() {
        configSpark(m_motor);
      }
      private void configSpark(CANSparkMax motor){
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
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