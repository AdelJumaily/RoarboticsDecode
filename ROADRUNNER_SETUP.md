# Road Runner Setup Instructions

## Dependencies Added

The following dependencies have been added to `TeamCode/build.gradle`:

1. **Road Runner FTC**: `com.acmerobotics.roadrunner:ftc:0.1.25`
2. **Road Runner Core**: `com.acmerobotics.roadrunner:core:0.5.5`
3. **Road Runner Actions**: `com.acmerobotics.roadrunner:actions:0.5.5`
4. **Dashboard**: `com.acmerobotics.dashboard:dashboard:0.5.0`
5. **Apache Commons Math**: `org.apache.commons:commons-math3:3.6.1` (for RegressionUtil)
6. **EJML**: `org.ejml:ejml-all:0.41`

## Repository Configuration

The Road Runner repository has been added to:
- `build.gradle` (allprojects section)
- `build.dependencies.gradle`

## Next Steps

1. **Configure Android SDK** (if not already done):
   - Set `ANDROID_HOME` environment variable, OR
   - Create `local.properties` in project root with:
     ```
     sdk.dir=/path/to/Android/sdk
     ```

2. **Sync Gradle in Android Studio**:
   - Click "Sync Now" when prompted, OR
   - Go to File → Sync Project with Gradle Files

3. **Download Dependencies**:
   - Run: `./gradlew build --refresh-dependencies`
   - Or let Android Studio sync automatically

## Files Fixed

- ✅ `DashboardUtil.java` - Restored Road Runner imports
- ✅ `AssetsTrajectoryManager.java` - Uses Road Runner trajectory classes
- ✅ `RegressionUtil.java` - Added Apache Commons Math dependency
- ✅ `Encoder.java` - Uses Road Runner NanoClock
- ✅ `MovingAverage.java` - Fixed division by zero
- ✅ `MovingAverageTwist2d.java` - Fixed division by zero
- ✅ `LoggingUtil.java` - Fixed null pointer exception

## Note

Dependencies may show as unresolved until:
1. Android SDK is configured
2. Gradle sync completes
3. Dependencies are downloaded from repositories

Once the Android SDK is configured and Gradle syncs, all Road Runner classes should be available.
