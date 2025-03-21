// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Height;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIONeo550;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIONeo550;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIONeo;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Coral coral;
  private final Elevator elevator;
  private final Algae algae;

  // Controller
  private final CommandXboxController xDriver = new CommandXboxController(0);
  private final CommandXboxController coPilot = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        coral = 
            new Coral(new CoralIONeo550());
        elevator =
            new Elevator(new ElevatorIONeo());
        algae = 
            new Algae(new AlgaeIONeo550());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        coral =
            new Coral(new CoralIO() {});
        elevator = 
            new Elevator(new ElevatorIO() {});
        algae = 
            new Algae(new AlgaeIO() {});
        break;
    }

    //----- Set up Named Commands -----
    //Elevator Commands
    // NamedCommands.registerCommand("ElevatorHome", 
    //     Commands.runOnce(() -> elevator.setPosition(Height.HOME), elevator));
    // NamedCommands.registerCommand("ElevatorL2", 
    //     Commands.runOnce(() -> elevator.setPosition(Height.L2), elevator));
    // NamedCommands.registerCommand("ElevatorL3", 
    //     Commands.runOnce(() -> elevator.setPosition(Height.L3), elevator));
    // NamedCommands.registerCommand("ElevatorHighAlgae", 
    //     Commands.runOnce(() -> elevator.setPosition(Height.HIGH_ALGAE), elevator));
    // NamedCommands.registerCommand("ElevatorLowAlgae", 
    //     Commands.runOnce(() -> elevator.setPosition(Height.LOW_ALGAE), elevator));
    //Scoring Commands
    // NamedCommands.registerCommand("CoralTroughScore",
    //     Commands.run(() -> coral.scoreTrough()).withTimeout(0.25));
    // NamedCommands.registerCommand("CoralBranchScore", 
    //     Commands.run(() -> coral.scoreCoral()).withTimeout(0.25));
    //Algae Commands
    // NamedCommands.registerCommand("RemoveAlgae", 
    //     Commands.run(() -> algae.extakeAlgae()).withTimeout(2));
    // NamedCommands.registerCommand("GrabAlgae", 
    //     Commands.run(() -> algae.extakeAlgae()).withTimeout(1));  
    // NamedCommands.registerCommand("SpitAlgae", 
    //     Commands.run(() -> algae.extakeAlgae)).withTimeout(1);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -xDriver.getLeftY(),
            () -> -xDriver.getLeftX(),
            () -> -xDriver.getRightX()));

    //----- Driver Button Bindings -----
    // Lock to 0° when A button is held
    xDriver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -xDriver.getLeftY(),
                () -> -xDriver.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    xDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    xDriver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    xDriver
        .rightTrigger(0.75)
        .whileTrue(
            Commands.startEnd(
                () -> coral.run(), coral::stop, coral));

    //Extake Algea
    xDriver
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> algae.extakeAlgae(), algae::stop, algae));
                
    //Intake Algae
    xDriver
        .leftTrigger(0.75)
        .whileTrue(
            Commands.startEnd(
                () -> algae.intakeAlgae(), algae::stop, algae));

    //Run elevator to hight for L1 / Coral station
    // coPilot
    //     .b()
    //     .onTrue(
    //         Commands.run(
    //             () -> 
    //                 elevator.setPosition(Height.HOME), elevator));
    
    //Run elevator to height for L2
    coPilot
        .y()
        .onTrue(
            Commands.run(
                () ->
                    elevator.setPosition(Height.L2), elevator));

    //Run elevator to height for L3
    // coPilot
    //     .x()
    //     .onTrue(
    //         Commands.run(
    //             () -> 
    //                 elevator.setPosition(Height.L3), elevator));
    
    //Run elevator to height for high algae
    coPilot
        .rightBumper()
        .onTrue(
            Commands.run(
                () -> 
                    elevator.setPosition(Height.HIGH_ALGAE), elevator));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void initPreferences(){
    Preferences.initDouble("L2 Height", 10);
    Preferences.initDouble("L3 Height", 24.5);
    Preferences.initDouble("High Algae Height", 19);
    Preferences.initDouble("L2 & L3 Scoring Speed", 0.45);
    Preferences.initDouble("High Algae Height", 24.5);
    Preferences.initDouble("Trough Left Speed", 0.15);
    Preferences.initDouble("Trough Right Speed", 0.45);
    Preferences.initDouble("Algae Intake Speed", 1.0);
    Preferences.initDouble("Algae Extake Speed", -1.0);
  }
  public void updatePreferences(){
    Preferences.getDouble("L2 Height", 10);
    Preferences.getDouble("L3 Height", 24.5);
    Preferences.getDouble("High Algae Height", 19);
    Preferences.getDouble("Trough Left Speed", 0.15);
    Preferences.getDouble("Trough Right Speed", 0.45);
    Preferences.getDouble("Algae Intake Speed", 1.0);
    Preferences.getDouble("Algae Extake Speed", -1.0);

    Constants.elevatorHeight.L2 = Preferences.getDouble("L2 Height", 10);
    Constants.elevatorHeight.L3 = Preferences.getDouble("L3 Height", 24.5);
    Constants.elevatorHeight.highAlgae = Preferences.getDouble("High Algae Height", 19);

    Constants.coralSpeed.speed = Preferences.getDouble("L2 & L3 Scoring Speed", 0.45);
    Constants.coralSpeed.troughLeft = Preferences.getDouble("Trough Left Speed", 0.15);
    Constants.coralSpeed.troughRight = Preferences.getDouble("Trough Right Speed", 0.45);

    Constants.algaeSpeed.intakeSpeed = Preferences.getDouble("Algae Intake Speed", 1.0);
    Constants.algaeSpeed.extakeSpeed = Preferences.getDouble("Algae Extake Speed", -1.0);
  }
}