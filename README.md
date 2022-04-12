# Search and Rescue simulation
This project aims to simulate the Urban Search and Rescue operation in ROS. 
Here, we work in environment where the map is available, which can also be created using `gmapping` package of ROS. One turtlebot explores the environment while searching for 'trapped victims', modeled with ArUcO markers. These markers have IDs which can represent 'severity' of injury. Once the environment is explored and all markers found, the explorer turtlebot returns to start location and the 'rescuer' turtlebot goes to each of the markers in ID order. 

![world](https://github.com/niteshjha08/simplified)

### 1. Explorer robot reads approximate locations of victims
We populate the parameter server with approximate coordinates around which targets may be found. The only requirement for this location is that the ArUco marker must be visible from this location in the explorer's camera.

<INSERT CAMERA IMAGE------

The explorer reads these from the parameter server and creates a MoveBase goal, and sets the goal pose as the approximate victim location. Then this goal is sent and the motion begins.

<INSERT GIF of motion------

### 2. Search is executed
When the explorer reaches the vicinity of victim, it rotates slowly and continuously searches for the ArUco marker, until it is found. When this happens, it calculates the 3D position of the marker.

<INSERT GIF of rotating ----------

#### How is 3D pose found?
This is achieved by using the camera images of the explorer, and running a node 'aruco_detect'. This subscribes to
- camera images
- camera info : the intrinsic matrix of the camera
This node searches for the markers in every frame. When a marker is found, the image coordinates of the corners of the marker is saved. We leverage the fact that the marker lies on a plane, and calculate the homography matrix that will transform the marker (assuming z=0) to the current image coordinates. With the homography matrix H and the camera intrinsic matrix K, we can find the Projection matrix P. <SHOW equations ----------

With this, using the 2D image points, we find the 3D world coordinates.

The aruco_detect also decodes the tag ID which is used to determine order of visiting by the rescuer turtlebot.
### 3. Determine offset for marker location and broadcast position
<INSERT LOCATION OF MARKER IMG -------------------

### 4. Listen to TF topic and determine marker location in map frame
If TF lookup succeeds, push the position to an array

### 5. Go to next target location, repeat search until all targets have been found.

### 6. Rescuer bot initiates motion, and visits all positions of the marker that was stored in the array
<INSERT A GIF OF RESCUER-------

