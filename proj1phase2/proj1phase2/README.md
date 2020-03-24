# Report PDF
The report pdf is named `Proj1phase2 Report`
# Testing code
The main function is run in the file `..\code\test_trajectory`
There are two methods and four paths to choose:
## Methods
You can modify the function in the line 80 to apply the two methods.
```matlab
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9,@trajectory_generator_polynomial);
```
- `@trajectory_generator_polynomial`
- `@trajectory_generator_snap`
## Paths
To choose which path to follow, modify line 58 and set the global variavble `path` to the index of the desired path:
```matlab
path = path*     % *can be 1,2,3,4
```
# Source figures
Due to the page limit of report, the included figures may not be clear. The .fig files are stored in `..\code` directory and named by correspoding number, e.g., `path1.fig` is the result of the simulation following the path 1.
