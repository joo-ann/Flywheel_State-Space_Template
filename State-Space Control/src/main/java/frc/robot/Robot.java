// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;
  private static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

  private static final double kFlyWheelMomentOfInertia = 0.00032; // Calculate by multiplying kg by m^2

  private static final double kFlywheelGearing = 1.0; // Reduction between motors and gears. Greater than one if flywheel spins slower than motors
  

  // State-space of the flywheel. 
  // Properties: States (velocities rad/s), Inputs (volts), Outputs (rad/s)
  private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), kFlyWheelMomentOfInertia, kFlywheelGearing);

  // Used to reject noise (unnecessary information that clouds our predictions)
  private final KalmanFilter<N1, N1, N1> m_observer = 
    new KalmanFilter<>(
      Nat.N1(), 
      Nat.N1(), 
      m_flywheelPlant, 
      VecBuilder.fill(3.0), // Accuracy of model
      VecBuilder.fill(0.01), // Accuracy of encoder data
      0.020);

  // Use feedback to create voltage commands
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller = 
    new LinearQuadraticRegulator<>(
        m_flywheelPlant, 
        VecBuilder.fill(8.0), // qelms. Velocity error tolerance. Decrease to heavily penalize state excursion and make the controller more aggressive
        VecBuilder.fill(12.0), // relms. Control error/voltage tolerance. Decrease to heavily penalize control effort, or make the controller less aggressive. Start with 12 (approximate battery max voltage)
        0.020); // Time between loops

  // State-space loop combines a controller, observer, feedforward, and plant for easy control
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  // Encoder to measure flywheel velocity in rad/s
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);

  private final Joystick m_joystick = new Joystick(kJoystickPort);


  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // 2π radians per 4096 clicks
    m_encoder.setDistancePerPulse(2.0 * Math.PI / 4096.0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Resets loop to base
    m_loop.reset(VecBuilder.fill(m_encoder.getRate()));
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Sets target speed for flywheel
    if(m_joystick.getTriggerPressed()) {
      // Set next reference
      m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    } else if(m_joystick.getTriggerReleased()) {
      // Spin down
      m_loop.setNextR(VecBuilder.fill(0.0));
    }

    // Correct Kalman filter's state vector estimate with encoder data
    m_loop.correct(VecBuilder.fill(m_encoder.getRate()));

    // Update LQR to generate new voltage commands and use the voltages to predict the next state without Kalman filter
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors
    // voltage = duty cycle * battery voltage -> duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_motor.setVoltage(nextVoltage);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
