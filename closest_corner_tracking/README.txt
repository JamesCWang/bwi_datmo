How to use:
  - Create a yaml/pgm file in the same format as 3netest.yaml/3netest.pgm
  - Change yaml source in datmo.launch 
  - Run "roslaunch datmo_pkg datmo.launch"
  - Tracked objects' position, velocity, and width are published to "datmo"
  
Tunable parameters:
  - Segmentation distance in range_segmentation.py
  - Rectangle fitting criterion in rectangle_fitting.py
  - Rectangle fitting angular resolution in rectangle_fitting.py
  - Association threshold in association.py
  - Kalman filter covariances in kalman.py
  
Notes:
	The published rectangles are encoded by [x, y, l, w, theta] where each corner is (x+-l/2, y+-w/2) rotated counter-clockwise by theta degrees.
	