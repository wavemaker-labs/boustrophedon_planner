# Boustrophedon Example
This is an example usage that loads a map file into a field_polygon param, which gets read by the script and then converted and passed on to boustrophedon_server. All other dummy dependencies are pre-loaded into the launch file, just run `start_sim.launch` as below:

`roslaunch boustrophedon_example start_sim.launch`

This will start the boustrophedon_server and the waypoint planner script provided here. The waypoint planner script contains also some logic used for cleaning the output points.

How the example script works is basically just sending the points read from the input file into a polygon that is sent to the server which then creates the waypoints coverage. After retrieving the waypoints from the server, the plan is parsed, headings are added, points are cleaned, and dumped to csv. Finally, the path is visually shown by graph and published to `"/visualization_marker"` topic.

The query is looped in order to show increasing angles of incline for the given map. This is also a good test for robustness of the current planning logic.
