# Quick Start: @SmartDashboardCommand Annotation

## TL;DR

Replace manual `SmartDashboard.putData()` calls with annotations:

### Before:
```java
// In RobotContainer.java
SmartDashboard.putData("Intake Linear/Extend", intake.extendLinear());
SmartDashboard.putData("Intake Linear/Retract", intake.retractIntake());
```

### After:
```java
// In your subsystem
@SmartDashboardCommand(key = "Intake Linear/Extend")
public Command extendLinear() { ... }

@SmartDashboardCommand(key = "Intake Linear/Retract")
public Command retractIntake() { ... }

// In RobotContainer.initializeDashboard()
SmartDashboardCommands.register(this);
```

## 3 Simple Steps

### 1. Annotate Your Commands

Add `@SmartDashboardCommand(key = "...")` to command methods:

```java
import frc.lib.util.SmartDashboardCommand;

public class YourSubsystem extends SubsystemBase {
    
    @SmartDashboardCommand(key = "Subsystem Name/Command Name")
    public Command yourCommand() {
        return Commands.run(() -> { /* ... */ })
            .withName("Your Command");
    }
}
```

### 2. Build Your Code

```bash
./gradlew build
```

The annotation processor automatically generates `SmartDashboardCommands.java`.

### 3. Register in RobotContainer

```java
private void initializeDashboard() {
    SmartDashboardCommands.register(this);
    // Other SmartDashboard entries...
}
```

Make sure your subsystem field is accessible (not `private`):
```java
final YourSubsystem yourSubsystem;  // Package-private or public
```

## Rules

✅ Method must be `public`  
✅ Method must return `Command`  
✅ Method must have no parameters  
✅ Method must not be `static`  
✅ Key must not be empty  

## Full Documentation

See [SmartDashboardCommandAnnotation.md](./SmartDashboardCommandAnnotation.md) for complete documentation, examples, and troubleshooting.

## Example from IntakeSuperstructure

```java
@SmartDashboardCommand(key = "Intake Linear/Extend")
public Command extendLinear() {
    return this.runOnce(() -> intakeLinearIO.runCurrent(Amps.of(LINEAR_CURRENT.get())))
        .withName("Extend Linear");
}

@SmartDashboardCommand(key = "Intake Linear/Cycle")
public Command cycle() {
    return Commands.repeatingSequence(
        extendLinear(),
        Commands.waitUntil(isExtended).withTimeout(1.25),
        retractLinear(),
        Commands.waitUntil(isRetracted).withTimeout(1.25)).withName("Cycle");
}
```

This automatically generates:
```java
SmartDashboard.putData("Intake Linear/Extend", container.intake.extendLinear());
SmartDashboard.putData("Intake Linear/Cycle", container.intake.cycle());
```
