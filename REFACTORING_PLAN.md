# DC Robot Code Refactoring Plan

## Current Issues Identified

### 1. **Static State Anti-Pattern**
- Subsystems use static fields for state machines (`private static DCShooterStateMachine`)
- This prevents multiple instances and causes thread-safety issues
- Example: `DCShooterSubsystem`, `DCIntakeSubsystem`, `DCLiftSubsystem`

### 2. **Code Duplication**
- All subsystems follow similar patterns but duplicate code
- State machine initialization is repeated
- Update/stop methods are nearly identical

### 3. **Naming Inconsistencies**
- Mix of camelCase and PascalCase: `DCliftSubsystem` vs `DCShooterSubsystem`
- Inconsistent field naming: `DCintakeStateMachine` vs `DCShooterStateMachine`
- Method names inconsistent: `getDCLiftSubsystem()` vs `getDCIntakeSubsystem()`

### 4. **Tight Coupling**
- `DCAutoRobotLIS` directly creates hardware and subsystems
- Hard to test or mock dependencies
- No dependency injection pattern

### 5. **Configuration Scattered**
- Magic numbers throughout code (0.55d, -0.65d, etc.)
- No central configuration class
- Hard-coded values in multiple places

### 6. **Mixed Responsibilities**
- `DCBaseLIS` mixes drive logic with robot initialization
- Too many responsibilities in single classes

### 7. **Commented Code**
- Large blocks of commented code that should be removed
- Makes code harder to read

### 8. **Inconsistent Patterns**
- Some subsystems use static getters/setters, others use instance
- No consistent pattern for state management

## Proposed Improvements

### 1. **Remove Static State**
- Convert all static fields to instance fields
- Each robot instance should have its own subsystem instances

### 2. **Create Base Subsystem Class**
- Abstract base class for common subsystem functionality
- Reduces code duplication by ~60%

### 3. **Standardize Naming**
- Use consistent camelCase for all fields and methods
- Follow Java naming conventions strictly

### 4. **Configuration Class**
- Create `DCRobotConfig` class for all constants
- Motor powers, lift positions, PID values, etc.

### 5. **Dependency Injection**
- Use constructor injection for dependencies
- Make classes more testable

### 6. **Better Separation of Concerns**
- Separate robot initialization from drive logic
- Create a `RobotBuilder` pattern for initialization

### 7. **Remove Dead Code**
- Delete all commented code blocks
- Clean up unused imports

### 8. **Subsystem Registry**
- Create a `SubsystemManager` to manage all subsystems
- Centralized update/start/stop calls

## Implementation Plan

### Phase 1: Configuration & Constants
1. Create `DCRobotConfig.java` with all constants
2. Create `DCMotorConfig.java` for motor-specific configs

### Phase 2: Base Classes
1. Create `BaseSubsystem<TStateMachine, TState>` abstract class
2. Refactor all subsystems to extend base class

### Phase 3: Remove Static State
1. Convert static fields to instance fields
2. Update all references

### Phase 4: Standardize Naming
1. Fix all naming inconsistencies
2. Ensure consistent camelCase throughout

### Phase 5: Dependency Injection
1. Refactor `DCAutoRobotLIS` to use DI
2. Create `RobotBuilder` for initialization

### Phase 6: Cleanup
1. Remove all commented code
2. Clean up unused imports
3. Add JavaDoc comments

## Benefits

- **Maintainability**: Easier to understand and modify
- **Testability**: Can mock dependencies for unit tests
- **Scalability**: Easy to add new subsystems
- **Consistency**: Uniform patterns throughout codebase
- **Readability**: Cleaner, more professional code
- **Reliability**: No static state issues, thread-safe

