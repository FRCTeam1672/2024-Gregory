// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private final Timer timer = new Timer();
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }
  //No commands are being used currently
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();

  }
  @Override
  public void autonomousPeriodic() {
    
  }
}