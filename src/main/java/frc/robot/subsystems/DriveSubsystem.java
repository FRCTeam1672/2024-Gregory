// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSubsystem extends SubsystemBase {
  private WPI_VictorSPX backLeft = new WPI_VictorSPX(13);
  private WPI_TalonSRX frontLeft = new WPI_TalonSRX(11);
  private WPI_VictorSPX backRight = new WPI_VictorSPX(14);
  private WPI_TalonSRX frontRight = new WPI_TalonSRX(12);
  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  public DriveSubsystem() {
    frontRight.setInverted(true);
    backRight.setInverted(true);
  }

  private final CommandXboxController xboxController = new CommandXboxController(0);

  public void stop() {
    drive.stopMotor();
    frontLeft.stopMotor();
    frontRight.stopMotor();
    backLeft.stopMotor();
    backRight.stopMotor();
  }

  @Override
  public void periodic() {
    double xSpeed = /*-(0.65 + speed) */ -xboxController.getLeftY();
    double zRotation = /*-(0.75 + speed / 2) */ -xboxController.getRightX();
    if (Math.abs(xSpeed) <= 0.20 && Math.abs(zRotation) <= 0.2) {
      stop();
      return;
    }
    drive.arcadeDrive(MathUtil.clamp(xSpeed, -0.85, 0.85), MathUtil.clamp(zRotation, -1, 1));
  }

  public void drive(double xSpeed, double zSpeed) {
    drive.arcadeDrive(xSpeed, -zSpeed);
  }

  /**
   * Arcade drive
   * 
   * If both of the params are == 0, then the motors will stop
   * 
   * @param forwards [-1.0 - 1.0] 1.0 is forwards
   * @param turn     [-1.0, 1.0] 1.0 is clockwise
   */
  public void arcadeDrive(double forwards, double turn) {
    this.drive.arcadeDrive(-forwards, turn);
  }
}
