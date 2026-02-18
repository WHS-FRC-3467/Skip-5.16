# SmartDashboard Command Annotation

This document explains how to use the `@SmartDashboardCommand` annotation to automatically register commands on the SmartDashboard.

## Overview

The `@SmartDashboardCommand` annotation automatically generates code that registers command methods on the SmartDashboard during build time. This eliminates the need to manually add `SmartDashboard.putData()` calls in `RobotContainer`.

## How It Works

1. **Annotate Commands**: Add `@SmartDashboardCommand(key = "...")` to public command methods in subsystems
2. **Build Time Processing**: The annotation processor runs during compilation and generates a `SmartDashboardCommands` class
3. **Registration**: The generated class contains a `register()` method that calls `SmartDashboard.putData()` for each annotated command
4. **Initialization**: `RobotContainer` calls `SmartDashboardCommands.register(this)` to register all commands

## Usage Example

### Step 1: Annotate Command Methods

In your subsystem class, add the `@SmartDashboardCommand` annotation to command factory methods:

```java
import frc.lib.util.SmartDashboardCommand;

public class IntakeSuperstructure extends SubsystemBase {
    
    @SmartDashboardCommand(key = "Intake Linear/Extend")
    public Command extendLinear() {
        return this.runOnce(() -> intakeLinearIO.runCurrent(Amps.of(LINEAR_CURRENT.get())))
            .withName("Extend Linear");
    }

    @SmartDashboardCommand(key = "Intake Linear/Retract")
    public Command retractIntake() {
        return Commands.sequence(
            runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())),
            retractLinear(),
            Commands.waitUntil(isRetracted),
            stopRoller()).withName("Retract Intake");
    }

    @SmartDashboardCommand(key = "Intake Linear/Cycle")
    public Command cycle() {
        return Commands.repeatingSequence(
            extendLinear(),
            Commands.waitUntil(isExtended).withTimeout(1.25),
            retractLinear(),
            Commands.waitUntil(isRetracted).withTimeout(1.25)).withName("Cycle");
    }
}
```

### Step 2: Build the Project

Run the Gradle build to trigger annotation processing:

```bash
./gradlew build
```

This generates the `SmartDashboardCommands` class in the `frc.robot` package.

### Step 3: Call the Generated Registration Method

In `RobotContainer.initializeDashboard()` (or constructor), call the generated registration method:

```java
private void initializeDashboard() {
    // Register all @SmartDashboardCommand annotated commands
    SmartDashboardCommands.register(this);
    
    // Add other manual SmartDashboard entries as needed
    SmartDashboard.putData("Indexer/Feed", indexer.feed());
}
```

### Step 4: Make Subsystems Accessible

Ensure that subsystems used in annotated commands are accessible from the generated code. They should be at least package-private (default access) in `RobotContainer`:

```java
public class RobotContainer {
    // Package-private to allow SmartDashboardCommands access
    final IntakeSuperstructure intake;
    final IndexerSuperstructure indexer;
}
```

## Generated Code Example

For the annotated methods above, the processor generates:

```java
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class SmartDashboardCommands {

    private SmartDashboardCommands() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    public static void register(RobotContainer container) {
        // Commands from frc.robot.subsystems.intake.IntakeSuperstructure
        SmartDashboard.putData("Intake Linear/Extend", container.intake.extendLinear());
        SmartDashboard.putData("Intake Linear/Retract", container.intake.retractIntake());
        SmartDashboard.putData("Intake Linear/Cycle", container.intake.cycle());
    }
}
```

## Requirements

The annotated method must meet these requirements:

1. **Public**: The method must be `public`
2. **Non-static**: The method cannot be `static`
3. **Returns Command**: The method must return `edu.wpi.first.wpilibj2.command.Command` or a subtype
4. **No parameters**: The method should not have parameters (commands with parameters cannot be easily registered)
5. **Non-empty key**: The annotation's `key` parameter must be a non-empty string

## Key Naming Conventions

- Use forward slashes (`/`) to create dashboard groups: `"Subsystem/Command Name"`
- Consider using subsystem NAME constants for consistency: `IntakeLinearConstants.NAME + "/Extend"`
- Keep names descriptive but concise

## Benefits

1. **Reduces Boilerplate**: No need to manually write `SmartDashboard.putData()` calls
2. **Prevents Errors**: Eliminates typos in SmartDashboard keys and command method names
3. **Maintainability**: When you add/remove/rename commands, the dashboard is automatically updated
4. **Self-Documenting**: The annotation makes it clear which commands are exposed to SmartDashboard
5. **Type Safety**: Compile-time validation ensures methods return `Command` and meet requirements

## Troubleshooting

### Compilation Errors

If you get compile errors about `SmartDashboardCommands` not being found:
1. Ensure you've run a full build: `./gradlew clean build`
2. Check that the annotation processor is registered in `META-INF/services/javax.annotation.processing.Processor`
3. Verify that the annotated methods meet all requirements (see above)

### Commands Not Appearing on SmartDashboard

1. Verify that `SmartDashboardCommands.register(this)` is called in `RobotContainer`
2. Check that the subsystem field in `RobotContainer` is accessible (not `private`)
3. Ensure the command method name matches the field name heuristic (e.g., `IntakeSuperstructure` → `intake`)

### Custom Subsystem Field Names

If your subsystem field name doesn't follow the convention (lowercase class name), you may need to manually edit the generated `SmartDashboardCommands` class after each build, or update the annotation processor logic to handle your naming scheme.

## Implementation Details

- **Annotation**: `frc.lib.util.SmartDashboardCommand`
- **Processor**: `frc.lib.processors.SmartDashboardCommandProcessor`
- **Generated Class**: `frc.robot.SmartDashboardCommands` (auto-generated in `build/generated/sources/annotationProcessor/java/main/`)
- **Processing**: Runs at compile time via Java annotation processing API

**Note**: The generated `SmartDashboardCommands.java` file is created during the build process in the `build/generated/sources/annotationProcessor/` directory. It is not checked into source control. After building, you can view it there to see what was generated.

## Future Enhancements

Possible improvements for future versions:

1. Support for parameterized commands with dashboard number inputs
2. Custom grouping and ordering of commands
3. Conditional registration based on robot mode (REAL/SIM/REPLAY)
4. Integration with Shuffleboard layouts
