// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  //sensors
  final ADIS16448_IMU m_gyro = new ADIS16448_IMU(IMUAxis.kZ, Port.kMXP, CalibrationTime._128ms); //gyro/imu
  //motors
  public final WPI_TalonFX m_left_1 = new WPI_TalonFX(10);//front
  final WPI_TalonFX m_left_2 = new WPI_TalonFX(11);//back
  public final WPI_TalonFX m_right_1 = new WPI_TalonFX(19);//front
  final WPI_TalonFX m_right_2 = new WPI_TalonFX(18);//back
  //private SlewRateLimiter driveLimiter = new SlewRateLimiter(2);
  //public final DifferentialDrive m_drive = new DifferentialDrive(m_left_1, m_right_1);
  public SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
                    DriveConstants.ksVolts.get(),
                    DriveConstants.kvVoltSecondsPerMeter.get(),
                    DriveConstants.kaVoltSecondsSquaredPerMeter.get());
  public PIDController leftController = new PIDController(DriveConstants.kPDriveVel.get(), DriveConstants.kI, DriveConstants.kD);
  public PIDController rightController = new PIDController(DriveConstants.kPDriveVel.get(), DriveConstants.kI, DriveConstants.kD);
  public PIDController headingController = new PIDController(0.12, 0, 0);
  //odometry
  Field2d m_field = new Field2d();
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    configTalon(m_left_1, false);
    configTalon(m_left_2, false);
    configTalon(m_right_1, true);
    configTalon(m_right_2, true);
    m_left_1.setNeutralMode(NeutralMode.Coast);
    m_left_2.setNeutralMode(NeutralMode.Coast);
    m_right_1.setNeutralMode(NeutralMode.Coast);
    m_right_2.setNeutralMode(NeutralMode.Coast);
    m_left_2.follow(m_left_1);
    m_right_2.follow(m_right_1);
    leftController.setTolerance(3, 4);
    rightController.setTolerance(3, 4);
  }
  private StatorCurrentLimitConfiguration brakeConfiguration = new StatorCurrentLimitConfiguration(false, 5, 0, 0);//false is active during neutral mode only, true is always active
  private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 0, 0);//same here
  private void configTalon(TalonFX motor, boolean isRight){
    motor.configFactoryDefault();
    motor.configStatorCurrentLimit(brakeConfiguration);
    motor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    //motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(!isRight);//TalonFX also has the ability to set the Direction of rotation, but I didn't use that here
    motor.configOpenloopRamp(0);
   // motor.configClosedloopRamp(.05);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);
  }
  public void setNeutralMode(NeutralMode neutralMode){
    m_left_1.setNeutralMode(neutralMode);
    m_left_2.setNeutralMode(neutralMode);
    m_right_1.setNeutralMode(neutralMode);
    m_right_2.setNeutralMode(neutralMode);
  }
  public void configRamp(double secToFull){
    m_left_1.configOpenloopRamp(secToFull);
    m_left_2.configOpenloopRamp(secToFull);
    m_right_1.configOpenloopRamp(secToFull);
    m_right_2.configOpenloopRamp(secToFull);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      getHeading(), 
      nativeUnitsToDistanceMeters(averageLeftPosition()), 
      nativeUnitsToDistanceMeters(averageRightPosition()));
      //SmartDashboard.putData("Field", m_field);
  }
  public double getGyro(){
    return -m_gyro.getGyroAngleZ();
  }
  public double getAbsoluteVelocity(){
    return (leftWheelSpeedMetersPerSecond())+(rightWheelSpeedMetersPerSecond());
  }
  public void arcadeDrive(double fwd, double rot){
    //m_drive.arcadeDrive(fwd, rot);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_left_1.setVoltage(leftVolts);
    m_right_1.setVoltage(rightVolts);
  }
  public boolean stalling(){
    return m_left_1.getSupplyCurrent()>=30;
  }
  public void stop(){
    m_left_1.setVoltage(0);
    m_right_1.setVoltage(0);
  }
  public void setPID(double leftSpeed, double rightSpeed){
    //double angularAdjust = headingController.calculate(getHeading().getRadians());
    //THESE PID CONTROLLERS ARE NOT CORRECT
    //Causes improper units, but consistent result
    m_left_1.setVoltage(feedForward.calculate(leftSpeed)+leftController.calculate(leftWheelSpeedMetersPerSecond(), leftSpeed));
    m_right_1.setVoltage(feedForward.calculate(rightSpeed)+leftController.calculate(leftWheelSpeedMetersPerSecond(), rightSpeed));
  }
  public boolean halfSpeed = false;
  public void setPIDNoGyro(double leftSpeed, double rightSpeed){
    if(halfSpeed){
      leftSpeed = leftSpeed/3;
      rightSpeed = rightSpeed/3;
    }
    leftSpeed = leftSpeed * DriveConstants.maxSpeedMetersPerSec;
    rightSpeed = rightSpeed * DriveConstants.maxSpeedMetersPerSec;

    m_left_1.setVoltage(feedForward.calculate(leftSpeed)+leftController.calculate(leftWheelSpeedMetersPerSecond(), leftSpeed));
    m_right_1.setVoltage(feedForward.calculate(rightSpeed)+rightController.calculate(rightWheelSpeedMetersPerSecond(), rightSpeed));
    //m_left_1.setVoltage((leftController.calculate(m_left_1.getMotorOutputVoltage(), feedForward.calculate(leftSpeed* DriveConstants.maxSpeedMetersPerSec))));
    //m_right_1.setVoltage((rightController.calculate(m_right_1.getMotorOutputVoltage(), feedForward.calculate(rightSpeed* DriveConstants.maxSpeedMetersPerSec))));
  }
  public void setMaxOutput(double maxOutput) {
    //m_drive.setMaxOutput(maxOutput);
  }
  public void resetEncoders(){
    m_left_1.setSelectedSensorPosition(0);
    m_left_2.setSelectedSensorPosition(0);
    m_right_1.setSelectedSensorPosition(0);
    m_right_2.setSelectedSensorPosition(0);
  }
  public void zeroHeading(){
    m_gyro.reset();
  }
    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in radians, from -pi to pi
   */
  public Rotation2d getHeading(){
    double radians = -(Math.IEEEremainder(m_gyro.getGyroAngleZ(), 360)*Math.PI/180);
    return new Rotation2d(radians);
  }
    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate(){
    return m_gyro.getGyroRateZ();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftWheelSpeedMetersPerSecond(), rightWheelSpeedMetersPerSecond());
  }
  public double leftWheelSpeedMetersPerSecond(){
    double countsPer100ms = (m_left_1.getSelectedSensorVelocity()+m_left_2.getSelectedSensorVelocity())/2;
    double countsPerSecond = (countsPer100ms*10);
    return -nativeUnitsToDistanceMeters(countsPerSecond);
  }
  public double rightWheelSpeedMetersPerSecond(){
    double countsPer100ms = (m_right_1.getSelectedSensorVelocity()+m_right_2.getSelectedSensorVelocity())/2;
    double countsPerSecond = (countsPer100ms*10);
    return -nativeUnitsToDistanceMeters(countsPerSecond);
  }
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }
  private double averageLeftPosition(){
    return (m_left_1.getSelectedSensorPosition()+m_left_2.getSelectedSensorPosition())/2;
  }
  private double averageRightPosition(){
    return (m_right_1.getSelectedSensorPosition()+m_right_2.getSelectedSensorPosition())/2;
  }
  public double straightDistanceMeters(){
    return nativeUnitsToDistanceMeters((averageLeftPosition()+averageRightPosition())/2);
  }
  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveConstants.kCountsPerRev;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    return positionMeters;
  }







  CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(AutoConstants.maxCentripetalAcceleration);
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts.get(),
                DriveConstants.kvVoltSecondsPerMeter.get(),
                DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
            DriveConstants.kDriveKinematics,
            6);
    DifferentialDriveKinematicsConstraint differentialDriveKinematicsConstraint = new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, AutoConstants.kMaxSpeedMetersPerSecond);
    // Create config for trajectory
    
    TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .addConstraint(differentialDriveKinematicsConstraint)
                .addConstraint(centripetalAccelerationConstraint);

    public Command generate(Drivetrain drivetrain, Pose2d p0, Pose2d p1, Translation2d i0, Translation2d i1){
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at p0
                p0,
                // Pass through i0, i1
                List.of(i0, i1),
                // End at p1
                p1,
                // Pass config
                config);
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
                drivetrain::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                drivetrain.feedForward,
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                drivetrain.leftController,
                drivetrain.rightController,
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}