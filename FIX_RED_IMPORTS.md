# Fix Red Imports in Android Studio

## Why Imports Are Still Red

The red imports in `DashboardUtil.java` are showing because:
1. **Gradle hasn't synced** - The IDE needs to sync with Gradle to download dependencies
2. **Dependencies haven't been downloaded** - They're declared but not yet fetched
3. **IDE cache is stale** - Android Studio's index might be out of date

## Solution Steps (Do in Android Studio)

### Step 1: Sync Gradle
- Look for a notification bar at the top saying "Gradle files have changed"
- Click **"Sync Now"**
- OR go to: **File → Sync Project with Gradle Files**

### Step 2: Invalidate Caches
- Go to: **File → Invalidate Caches / Restart**
- Select **"Invalidate and Restart"**
- Wait for Android Studio to restart

### Step 3: Sync Again
- After restart, sync again: **File → Sync Project with Gradle Files**
- Wait for the sync to complete (check the bottom status bar)

### Step 4: Rebuild
- Go to: **Build → Rebuild Project**
- This will download all dependencies and compile the project

## Expected Result

After these steps:
- ✅ Red imports should turn white/black (resolved)
- ✅ No more "Cannot resolve symbol" errors
- ✅ Dependencies will be downloaded to `~/.gradle/caches`

## If Still Red After These Steps

1. Check the **Build** tab at the bottom for any error messages
2. Verify `local.properties` has correct SDK path
3. Check internet connection (dependencies need to be downloaded)
4. Try: **Build → Clean Project** then **Build → Rebuild Project**

## Dependencies That Should Download

- `com.acmerobotics.roadrunner:ftc:0.1.25`
- `com.acmerobotics.roadrunner:core:0.5.5`
- `com.acmerobotics.roadrunner:actions:0.5.5`
- `com.acmerobotics.dashboard:dashboard:0.5.0`
- `org.apache.commons:commons-math3:3.6.1`
