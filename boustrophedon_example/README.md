# Boustrophedon Example
This is an example usage that loads a map file into a field_polygon param, which gets read by the script and then converted and passed on to boustrophedon_server. All other dummy dependencies are pre-loaded into the launch file, just run `start_sim.launch`.

After retrieving the waypoints from the server, the plan is parsed, headings are added, points are cleaned, and dumped to csv. Finally, the path is visually shown by graph and published to `"/visualization_marker"` topic.
