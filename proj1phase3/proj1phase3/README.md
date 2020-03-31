# Report PDF
The report pdf is named `proj1phase3 Report`
# Testing code
The main function is run in the file `..\code\test_trajectory`
There are two methods and four paths to choose:
## Maps and Methods
There are four maps for path plannin. Two are in 2D space(map1,map2) and the others are in 3D space (map3, map4). To chose a map  to test, modify **Line 69** in the `test_trajectory.m`
```matlab
test_map = map4;  % Choices: 2D: map1,map2 3D: map3, map4
```
Then You can decide using 2D path planning or 3D path planning, the command of which locate in **Line 73** in the `test_trajectory.m`
```matlab
%Optimal_path = path_from_A_star_2D(test_map);
Optimal_path = path_from_A_star_3D(test_map);
```
`path_from_A_star_3D` can be applied to either 3D maps or 2D maps but `path_from_A_star_2D` can only cope with 2D maps.

# Source figures
Due to the page limit of report, the included figures may not be clear. The .fig files are stored in `..\code` directory and named by correspoding number, e.g., `map1.fig` is the result of the simulation of map1.
