package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private final Drivetrain drivetrain = new Drivetrain();
    private final XboxController controller =
        new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.drive(
                    controller.getLeftY(),   // forward/back
                    -controller.getLeftX(),   // strafe
                    -controller.getRightX()   // rotation
                ),
                drivetrain
            )
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}