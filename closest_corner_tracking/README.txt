How to use:
  - Create a yaml/pgm file in the same format as 3netest.yaml/3netest.pgm
  - Change yaml source in datmo.launch 
  - Run "roslaunch datmo_pkg datmo.launch"
  - Tracked objects' position, velocity, and width are published to "datmo"
  
Tunable parameters:
  - Segmentation distance in range_segmentation.py
  - Rectangle fitting criterion in rectangle_fitting.py
  - Rectangle fitting angular resolution in rectangle_fitting.py
  - Association threshold in datmo.py
  - Kalman filter covariances in kalman.py
  
Notes:
	If there are lag issues consider lowering the rectangle fitting angular resolution or switching features from center of fitted rectangle to average of x,y coordinates. Also note that for the rectangle fitting criterions, area is relatively inaccurate and variance is the slowest.
	