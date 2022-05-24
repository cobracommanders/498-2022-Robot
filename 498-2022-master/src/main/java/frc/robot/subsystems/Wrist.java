package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Wrist extends SubsystemBase{
    public PIDController pidController = new PIDController(IntakeConstants.kP.get(), 0, IntakeConstants.kD.get());
    public ArmFeedforward feedForward = new ArmFeedforward(IntakeConstants.ks, IntakeConstants.kcos, IntakeConstants.kv, IntakeConstants.ka);
    public WPI_TalonFX m_left = new WPI_TalonFX(8);
    public WPI_TalonFX m_right = new WPI_TalonFX(61);
    public static DigitalInput downLimit = new DigitalInput(0);
    public static DigitalInput upLimit = new DigitalInput(1);
    public static DigitalInput rightLimit = new DigitalInput(4);

    public Position position = Position.IN;
    double outLimit = 2048;
    double inLimit = 0;
    double speed = .2;
    double error;
    //private Position m_position = Position.IN;
    public double setpoint = 0;
    public enum Position {
      IN(upLimit, 0),
      OUT(downLimit, 4);
      public DigitalInput limit;
      public double setpoint;
      private Position(DigitalInput limit, double setpoint) {
        this.limit = limit;
        this.setpoint = setpoint;
      }
    }
    public Wrist() {
        configPID();
        configSpark(m_left);
        configSpark(m_right);
        m_right.setInverted(true);
        //m_right.follow(m_left);
      }
      private void configSpark(WPI_TalonFX motor){
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configClosedloopRamp(0);
        motor.configPeakOutputForward(1);
        motor.configPeakOutputReverse(-1);
        motor.set(ControlMode.PercentOutput, 0);
      }
      public void setPosition(Position position){
        this.position = position;
      }
      private void configPID(){
            // // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
             pidController.setTolerance(1/48);
             pidController.setSetpoint(0);
      }
      @Override
      public void periodic() {
        SmartDashboard.putNumber("WristPose", getRadians());

        if(position == Position.IN && getUpSensor()==false){
          m_left.setVoltage(-speed*12);
          m_right.setVoltage(-speed*12);
        }
        if(position == Position.OUT && getDownSensor()==false){
          m_left.setVoltage(speed*12);
        }
        if(position == Position.OUT && !rightLimit.get() == false){
          m_right.setVoltage(speed*12);
        }
      }
      public double getRadians() {
        return 2*Math.PI*(m_left.getSelectedSensorPosition()/2048)/48;
      }
      public void set(double speed){
        m_left.setVoltage(speed*12);
        m_right.setVoltage(speed*12);
      }
      public void setPID(Position position){
        m_left.setVoltage((feedForward.calculate(pidController.calculate(getRadians(), position.setpoint), 0.01))*11);
      }
      public void stop(){
        m_left.set(0);
        m_right.set(0);
      }
      public boolean getUpSensor() {
        return !upLimit.get();
      }
      public boolean getDownSensor() {
          return !downLimit.get();
      }
      public boolean getLimit(Position position){
        return position.limit.get();
      }
      // public void setState(Position position){
      //   this.m_position = position;
      //   //setpoint = position.setPoint;
      // }
      public void resetEncoders(){
        m_left.setSelectedSensorPosition(0);
        m_right.setSelectedSensorPosition(0);
      }
}