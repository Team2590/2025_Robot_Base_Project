# GEMINI.md: Project Overview

This document provides a comprehensive overview of the 2025_Robot_Base_Project, designed to be used as a context for AI-powered development assistance.

## Project Overview

This is a Java-based robot control system for the FIRST Robotics Competition (FRC), built using the WPILib framework. The project is structured around the **Command-Based programming model**, which provides a robust framework for organizing robot code.

### Key Technologies:

*   **Programming Language:** Java 17
*   **Build Tool:** Gradle with the GradleRIO plugin
*   **FRC Framework:** WPILib
*   **Logging/Replay:** AdvantageKit for advanced data logging and simulation replay.
*   **Autonomous Pathing:** PathPlanner for creating and following autonomous paths.
*   **Code Quality:** Spotless for automated code formatting.

### Architecture:

The project follows a standard command-based architecture:

*   **`Main.java`**: The main entry point that initializes the robot.
*   **`Robot.java`**: The main robot class that contains the robot's lifecycle methods (`robotInit`, `autonomousInit`, `teleopInit`, etc.) and runs the `CommandScheduler`.
*   **`RobotContainer.java`**: The central class that instantiates subsystems, configures button bindings, and sets up autonomous routines. It uses a hardware abstraction layer to switch between real hardware and simulation.
*   **Subsystems (`src/main/java/frc/robot/subsystems/`)**: These classes represent the physical components of the robot (e.g., `Drive`, `Arm`, `Elevator`).
*   **Commands (`src/main/java/frc/robot/commands/` and `command_factories/`)**: These classes represent actions the robot can take. Command factories are used to create complex, reusable commands.
*   **`Constants.java`**: A centralized location for all robot constants, making tuning and configuration easier.

## Building and Running

### Building the Code:

To build the robot code, run the following command from the project's root directory:

```bash
./gradlew build
```

### Deploying to the RoboRIO:

To deploy the code to the RoboRIO, run:

```bash
./gradlew deploy
```

### Running the Simulation:

To run the robot code in a simulation environment, use:

```bash
./gradlew simulateJava
```

### Running the Controller App:

The project includes a separate controller application. To run it, use:

```bash
./gradlew controller-app:run
```

## Development Conventions

*   **Code Style:** The project uses the Google Java Format, enforced by the Spotless plugin. Run `./gradlew spotlessApply` to format the code.
*   **Hardware Abstraction:** Subsystems are designed with a hardware abstraction layer. The IO layer is separated into interfaces, with implementations for both real hardware (e.g., `ArmIOTalonFX`) and simulation (e.g., `ArmIOSim`). This allows for extensive testing in a simulated environment.
*   **Testing:** The project is set up with JUnit and Mockito for unit testing, located in the `src/test/java` directory.
*   **Constants:** All constants are stored in the `Constants.java` file.
*   **Command-Based Model:** The project strictly adheres to the command-based model. Subsystems are self-contained, and their methods are controlled by Commands. Operator actions are bound to commands in `RobotContainer.java`.
