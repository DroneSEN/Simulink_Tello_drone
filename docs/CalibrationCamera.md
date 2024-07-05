### Step-by-Step Process

1. **Capture Calibration Images:**

   Use the provided Matlab script to capture images of a checkerboard pattern. Ensure that the checkerboard is visible in different positions and angles to cover the entire field of view of the camera.

   ```matlab
   % Run the script to capture images
   run('/DRONE_edutello/Simulink_Tello_drone-main/camera_calibration/camera_calibration_photo_extract.m')


2. **Open the Camera Calibrator App:**

   In Matlab, open the Camera Calibrator app using the following command:

   `run cameraCalibrator`

   ![Camera calibration](docs/imagesreadme/Cameracalibration.jpg)

3. **Import Calibration Images:**

   Click on Add Images and select the images captured in step 1.
   Ensure that the images contain a clear view of the checkerboard pattern.

4. **Set Checkerboard Parameters:**

   Enter the square size and the number of squares in the checkerboard.
   For this example, use the following settings:

   - Square Size: 34 (in millimeters)
   - Number of Squares: 6 rows by 8 columns

5. **Perform Calibration:**

   Click on Calibrate to start the calibration process.
   Matlab will process the images and compute the camera parameters.

6. **Review Calibration Results:**

   After the calibration is complete, review the reprojection errors and ensure they are within an acceptable range.
   Save the calibration session and export the camera parameters for future use.
