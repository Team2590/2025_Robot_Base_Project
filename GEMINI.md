# GEMINI.md - FRC 2590 2025 Robot Base Project

This document provides a comprehensive overview of the FRC 2590 2025 Robot Base Project, intended as a guide for AI-assisted development.

## Project Overview

This is a Java-based robot project for the FIRST Robotics Competition (FRC), built using the WPILib framework. The project is configured for the 2025 season. It features a sophisticated, multi-robot architecture with distinct configurations for different robots (named "Leonidas", "Kronos", "Larry"), a physics simulation mode, and a log replay mode for debugging.

The project leverages several key libraries and tools:

*   **AdvantageKit:** For advanced logging, data replay, and a robust hardware abstraction layer (IO interfaces).
*   **PathPlannerLib:** For creating and following complex autonomous paths.
*   **Phoenix 6 (CTRE):** For controlling TalonFX motors, likely for the swerve drive and other mechanisms.
*   **PhotonVision:** For vision processing and AprilTag detection.
*   **QuestNav:** Another vision or localization library.
*   **Spotless & Google Java Format:** To enforce code style and formatting.
*   **Lombok:** To reduce boilerplate code.

The robot code is organized into subsystems (Drive, Arm, Elevator, etc.), commands, and command factories. It uses a command-based programming model. A notable feature is the use of different "IO" implementations for real hardware versus simulation, allowing for high-fidelity testing.

## Building and Running

This is a Gradle project. The primary way to interact with the build system is through the `gradlew` wrapper script.

*   **Build the code:**
    ```bash
    ./gradlew build
    ```

*   **Run tests:**
    ```bash
    ./gradlew test
    ```

*   **Deploy to the RoboRIO:**
    ```bash
    ./gradlew deploy
    ```

*   **Run the physics simulator:**
    The `wpi.sim.addGui().defaultEnabled` property in `build.gradle` is set to `false`, which is non-standard. This is likely to support AdvantageKit's log replay feature. To run the simulator with a GUI, you may need to change this to `true` or run a specific Gradle task.

*   **Run AdvantageKit Log Replay:**
    ```bash
    ./gradlew replayWatch
    ```

*   **Run the Controller App:**
    A separate Java application, `controller-app`, is included. It can be run with:
    ```bash
    ./gradlew controller-app:run
    ```

## Development Conventions

*   **Code Style:** The project uses the Google Java Format, enforced by the Spotless plugin. Run `./gradlew spotlessApply` to format the code.
*   **Hardware Abstraction:** A key convention is the use of an `IO` layer for each subsystem (e.g., `ArmIO`, `GyroIO`). This separates the logic of the subsystem from the specific hardware implementation. For example, `ArmIOTalonFX` is used on the real robot, while `ArmIOSim` is used in simulation.
*   **Constants:** The `Constants.java` file is central to the project. It defines different sets of constants for each robot (`ArmConstantsLeonidas`, `ArmConstantsKronos`, etc.) and for different modes (real, sim, replay). It also makes heavy use of `LoggedTunableNumber`, allowing for live tuning of values from a dashboard.
*   **Command Factories:** Instead of instantiating commands directly in the `RobotContainer`, the project uses "factory" classes (e.g., `DriveFactory`, `ScoringFactory`). This helps to organize command creation and keep the `RobotContainer` cleaner.
*   **Autonomous:** Autonomous routines are defined using PathPlanner. The project registers "named commands" (e.g., `PrimeL4`, `ScoreL4`) which can be called from within the PathPlanner paths, allowing for complex, event-driven autonomous sequences.
*   **Robot Identification:** The robot's current operational mode is determined by `Constants.currentMode`. This is set to a specific robot name (like `Mode.Leonidas`) when running on hardware, or to `Mode.SIM` or `Mode.REPLAY` for development. This allows the code to adapt to the environment it's running in.

## Key Files

*   `build.gradle`: The main Gradle build file. Defines dependencies, plugins, and build tasks.
*   `settings.gradle`: Defines the multi-project structure (including `controller-app`).
*   `src/main/java/frc/robot/Robot.java`: The main entry point of the robot program. It extends AdvantageKit's `LoggedRobot`.
*   `src/main/java/frc/robot/RobotContainer.java`: The core of the robot's structure. It instantiates subsystems, configures button bindings, and sets up autonomous routines.
*   `src/main/java/frc/robot/Constants.java`: Contains all the constants for the robot, with different values for different robots and modes. This is a critical file for understanding the robot's configuration.
*   `src/main/java/frc/robot/subsystems/`: This package contains the robot's subsystems. Each subsystem has a main class and one or more `IO` interfaces and implementations.
*   `src/main/java/frc/robot/command_factories/`: This package contains the command factory classes.
*   `src/main/deploy/pathplanner/`: This directory contains the PathPlanner autonomous paths (`.auto` files) and paths (`.path` files).
*   `vendordeps/`: Contains JSON files defining third-party vendor dependencies.
