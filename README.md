# Search and Rescue simulation
This project aims to simulate the Urban Search and Rescue operation in ROS. 
Here, we work in environment where the map is available, which can also be created using `gmapping` package of ROS. One turtlebot explores the environment while searching for 'trapped victims', modeled with ArUcO markers. These markers have IDs which can represent 'severity' of injury. Once the environment is explored and all markers found, the explorer turtlebot returns to start location and the 'rescuer' turtlebot goes to each of the markers in ID order. 

![world](https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/world.png)

### 1. Explorer robot reads approximate locations of victims
We populate the parameter server with approximate coordinates around which targets may be found. The only requirement for this location is that the ArUco marker must be visible from this location in the explorer's camera.
![camera_view](https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/camera_view.png)

The explorer reads these from the parameter server and creates a MoveBase goal, and sets the goal pose as the approximate victim location. Then this goal is sent and the motion begins.
<!-- ![camera_view](https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/move_to_target.gif) -->
<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/move_to_target.gif" width="1000px" align="center">

### 2. Search is executed
When the explorer reaches the vicinity of victim, it rotates slowly and continuously searches for the ArUco marker, until it is found. When this happens, it calculates the 3D position of the marker.
<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/search_at_target.gif" width="1000px" align="center">
<!-- ![camera_view](https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/search_at_target.gif) -->

#### How is 3D pose found?
This is achieved by using the camera images of the explorer, and running a node 'aruco_detect'. This subscribes to
- camera images
- camera info : the intrinsic matrix of the camera
This node searches for the markers in every frame. When a marker is found, the image coordinates of the corners of the marker is saved. We leverage the fact that the marker lies on a plane, and calculate the homography matrix that will transform the marker (assuming z=0) to the current image coordinates. With the homography matrix H and the camera intrinsic matrix K, we can find the Projection matrix P. 
<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/eq1.png" width="200px" align="center">

<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/eq2.png" width="200px" align="center">

With this, using the 2D image points, we find the 3D world coordinates.

The aruco_detect also decodes the tag ID which is used to determine order of visiting by the rescuer turtlebot.
### 3. Determine offset for marker location and broadcast position

Once marker is detected, the TFbroadcaster sends the transform between the explorer's camera frame and the marker coordinate. In doing so, there has to be an offset added to the marker as the rescuer bot will not be able to reach exactly at the marker location (as it will collide with the wall). Thus, the z-direction of the marker, which is always outwards from the marker plane is given an offset and the marker position is then broadcasted.

<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/tolerance.png" width="700px" align="center">

The broadcast function flow is shown.


<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/broadcast.png" width="500px" align="center">

### 4. Listen to TF topic and determine marker location in map frame
The listener is then called, and it attempts to look up transform between map frame and marker frame. If the TF lookup succeeds, the position is pushed to an array.

<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/listener.png" width="500px" align="center">

### 5. Continue search until all victims have been found

<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/next_search.gif" width="1000px" align="center">

### 6. Rescuer bot initiates motion, and visits all positions of the marker that was stored in the array
<img src="https://github.com/niteshjha08/simplified-search-rescue/blob/main/media/rescuer_start.gif" width="1000px" align="center">

### 7. 

