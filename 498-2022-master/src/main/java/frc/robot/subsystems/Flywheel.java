package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{
    public WPI_TalonFX m_left = new WPI_TalonFX(12);
    public WPI_TalonFX m_right = new WPI_TalonFX(17);
    //public DigitalInput m_launchSensor = new DigitalInput(8);
    //public Counter counter = new Counter(m_launchSensor);//Maybe implement this later
    public PIDController pidController = new PIDController(.0000255, 0, 0);
    public double shootStall = SmartDashboard.getNumber("Shoot Stall", 1);
    public double driveStall = SmartDashboard.getNumber("Drive Stall", 0);
    public double spitStall = SmartDashboard.getNumber("Spit Stall", 0);
    public SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.5575125, 0.10782, 0.0070107);//55 is added
    //add pid for quicker shots
    public State state = State.OFF;
    public enum State {
      FULL(3100, "FULL"),//3150 rpm//54%//3190
      AUTO(4800, "AUTO"),//4400
      FWD(2300, "FWD"),//1920 rpm//2050
      REV(-2500, "REV"),
      OFF(0, "OFF");
      public double speed;
      public String string;
      private State(double speed, String string) {
        this.speed = speed;
        this.string = string;
      }
    }
    public Flywheel() {
        configTalon(m_left);
        configTalon(m_right);
        m_right.follow(m_left);
        m_right.setInverted(true);
        pidController.setTolerance(1000);
      }
      private void configTalon(WPI_TalonFX motor){
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configOpenloopRamp(.1);
        //motor.setSoftLimit
      }
     
      @Override
      public void periodic() {
        //THESE PID CONTROLLERS ARE NOT CORRECT
        //Causes improper units, but consistent results
        m_left.setVoltage(speedToVoltage(pidController.calculate(m_left.getSelectedSensorVelocity(), feedForward.calculate(rpmToRawUnits(state.speed)*32))));
        //SmartDashboard.putString("Flywheel State", state.string);
        SmartDashboard.putNumber("Flywheel Speed", getVelocity());
        //shootStall = SmartDashboard.getNumber("Shoot Stall", 1);
        //driveStall = SmartDashboard.getNumber("Drive Stall", 0);
        //spitStall = SmartDashboard.getNumber("Spit Stall", 0);

        //SmartDashboard.putNumber("Flywheel Setpoint", pidController.getPositionError());
        //SmartDashboard.putNumber("Cargo Count", counter.get());
      }
      public void setState(State State){
        this.state = State;
        //m_left.set(State.speed);
      }
      public double rpmToRawUnits(double rpm){
        return (rpm/600)*2048;
      }
      public double getVelocity(){
        double rawPer100ms = m_left.getSelectedSensorVelocity();
        double rawPerMin = rawPer100ms*10*60;
        double rotPerMin = rawPerMin/2048;
        return rotPerMin;
      }
      public void setVoltage(double volts){
        m_left.setVoltage(volts);
      }
      public double speedToVoltage(double speed){
        return speed*(10);
      }
      public void stop(){
          m_left.setVoltage(0);
      }
      public State getState(){
        return state;
      }
}