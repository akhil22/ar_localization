This package uses AR markers to localize the robot in an environment, to Run the localization follow these steps:

1. Launch ar_kinect.launch file from ar_kinect package and specify the patterns present in the environment using the parameter "marker pattern list" see the file in data/objects_ils in ar_kinect package for format 

2. Run the ar_localization node from ar_localization package, specify the marker location file using the parameter "marker_location_list" the default value of this parameter is "/data/marker_location". This is the file in which we define the marker poses correspoding to each of the marker mentioned in the file given in the parameter "marker_pattern_list" in the first step.

3. Run the amcl node.

 
