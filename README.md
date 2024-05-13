# Single Object Tracking with Template Matching & Kalman Filter
This repo demonstrates single object tracking using two different algorithms: template matching and Kalman filter. Template matching is a simple yet effective method for locating a template (object) within an image. Kalman filter is a recursive algorithm that estimates the state of a dynamic system from a series of noisy measurements. By combining these two algorithms, accurate and robust object tracking can be achieved.

Object tracking is a fundamental task in computer vision and robotics. It involves locating and following a specific object within a sequence of frames or images. This repo explores two different approaches to object tracking: template matching and Kalman filter. Template matching is based on comparing a template (a small patch of the image containing the object) as a ground truth with different parts of the image to find the best match. Kalman filter, on the other hand, is a state estimation algorithm that predicts the future state of an object based on past observations and a dynamic model.

## Feature
1. Kalman Filter Algorithm for Single Object Tracking

![tracked_object_kalman_filter-ezgif com-crop](https://github.com/KAN201197/Single_Object_Tracking/assets/128454220/b2bfc00e-c589-4d52-83a0-f16333413eb6)

2. Template Matching Algorithm for Single Object Tracking

![tracked_object_template_matching-ezgif com-crop](https://github.com/KAN201197/Single_Object_Tracking/assets/128454220/e4433805-7c06-40d4-8261-5520753e2f4a)

3. Integration with ROS for real-time tracking with Kalman Filter Algorithm in Robotics Application

## Prerequisites
1. python==3.9.0
2. numpy==1.24.3
3. matplotlib==3.8.2
4. opencv-python==4.9.0.80
5. ipywidgets==8.1.1
6. ipython==8.18.1

## Usage
1. Clone the repository:
   
       git clone https://github.com/KAN201197/Single_Object_Tracking.git
  
2. Navigate to the project directory:
   
       cd Single_Object_tracking
   
3. Install dependencies:
   
       pip install -r requirements.txt
   
4. Run the script 'single_object_tracking.ipynb' inside of jupyter notebook

## Detail instruction how to run the code

1. First, run cell 1 in jupyter notebook to load the sequence of images, firsttrack, and ground truth data. It also provides a slider in which the value can be changed according to the chosen sequence of images and will update the new path of the sequence of images, firsttrack, and ground truth data. 

2. Second, run cell 2 to load and run the function of template matching algorithm.
   
3. Third, run cell 3 to load and run the function of kalman filter algorithm.

4. Fourth, run cell 4 to evaluate the performance of both algorithms based on selected sequence of images. If you want to test the other sequence of images, go to cell 1 and on the ouput of cell 1 change the value on the slider to desire sequence and run cell 4 again to evaluate the performance for this sequence. For example, want to run sequence 5 then in the slider drag to value 5 and run again the code on cell 4 to evaluate the performance.

5. Fifth, run cell 5 to visualize the result of object tracking algorithm based on selected sequence of images. The same thing happen if you want to visualize other sequence, go to cell 1 and change the value on the slider to desire sequence and run cell 5 again to visualize the result. For example, want to run sequence 5 then in the slider drag to value 5 and run again the code on cell 5 to visualize the result.

## ROS Implementation

The ROS implementation of the Kalman filter tracking algorithm involves subscribing to image topics from a raw image performing object tracking in real-time, and publishing the tracked object's position (the coordinate of bounding box).

To use the Kalman Filter tracking with ROS:

1. set up your ROS environment and create a ROS package for the project.
2. Modify the code to subscribe to the appropriate image topics and publish the object's position.
3. Run the ROS nodes:

       roslaunch object_tracking_ros tracked_object.launch
   
