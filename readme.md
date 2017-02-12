# Pi OpenCV Vision
This project utilizes OpenCV and the Raspberry Pi to process image feeds to publish values to a network table.

## Build Instructions
**NOTE: If you are building this on the pi, you may need to `chmod -x gradlew`.**

1. Run `gradlew build` on the command line.
2. Enable SSH on the pi (if it is not already)
3. Configure the SSH remote in the `build.gradle` file to match the pi's IP.
4. Run `gradlew deploy`.

## Run Instructions
If the project is on the pi, simply running `gradlew run` should do just fine. Otherwise...

1. `cd` into the output directory, e.g.: `cd output/`.
2. If this is the first time you have deployed this to the pi, you may need to `chmod +x ./runCameraVision`.
3. Run `./runCameraVision`

## Usage
- All network table values are stored in a sub table called `Vision`.
- You need some way to exit the program. Right now, a way to do it is to create a key in the `Vision` network table called `shutdown`. Set this to `true` to shut down the program.
	- The shutdown call has a 5-second grace period, perfectly enough time to finish up the work with the pi or even cancel the shutdown call.
- The HSL thresholds are editable. You can use the SFX Dashboard or edit the values directly from an Outline Viewer/Network Tables viewer. These values can be found under `/Vision/hslThreshold`. They are:
	- `hueMin`/`hueMax` : Hue threshold
	- `satMin`/`satMax` : Saturation threshold
	- `lumMin`/`lumMax` : Luminance threshold
- This program uses two different USB cameras to find targets. One looks for the gear vision targets, the other finds the high goal targets. Each target is published into `/Vision/gearVision` or `/Vision/highGoal`, respectively. Every target's center coordinate, width, and height as perceived in the feed are published onto the sub tables.