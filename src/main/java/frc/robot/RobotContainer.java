package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private final Drivetrain drivetrain = new Drivetrain();
    private final XboxController controller =
        new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    private final Shooter shooter = new Shooter();
    private final Indexer indexer = new Indexer(drivetrain);

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
        // A button → spin up shooter
        new JoystickButton(controller, XboxController.Button.kA.value)
            .whileTrue(new RunCommand(() -> shooter.setRPM(4000), shooter));

        // B button → stop shooter
        new JoystickButton(controller, XboxController.Button.kB.value)
            .onTrue(new RunCommand(() -> shooter.stop(), shooter));

        indexer.setDefaultCommand(
            new RunCommand(() -> {
                double trigger = controller.getRightTriggerAxis();
                boolean triggerPressed = trigger > 0.2;

                boolean shooterReady =
                    Math.abs(shooter.getRPM() - 4000) < 150;

                indexer.tryFireBall(shooterReady, triggerPressed);

                if (triggerPressed) {
                    indexer.feed();
                } else {
                    indexer.stop();
                }

            }, indexer)
        );
    }
    

    public Command getAutonomousCommand() {
        return null;
    }
}