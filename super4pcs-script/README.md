# Super4PCS scripts

Scripts to automate registering multiple point clouds through Super4PCS, and finding the error between registered point clouds using PCL.

Super4PCS source: https://github.com/nmellado/Super4PCS

These scripts are used for [UC San Diego Engineers for Exploration](http://e4e.ucsd.edu/) research program under the [Maya Archaeology project](http://e4e.ucsd.edu/maya-archaeology).

For instructions on how to run these scripts in Docker, refer to the [E4E project wiki](https://github.com/UCSD-E4E/maya-archaeology/wiki/3.-Registration-algorithms#super4pcs).


## Scripts included

### `registration/s4p-register.sh`

Takes in an input text file including all point clouds to register, runs Super4PCS on these point clouds with the parameters specified in this text file, and outputs error between the point clouds in specified output folders.

For usage, refer to the [E4E project wiki: "Registration algorithms: Super4PCS"](https://github.com/UCSD-E4E/maya-archaeology/wiki/3.-Registration-algorithms#super4pcs) or run `./s4p-register.sh` without any parameters.

```
$ ./s4p-register.sh 
No input txt file specified.
USAGE: ./s4p-register.sh <input txt file> [-i input root filepath] [-o output root filepath] [-s = scripts filepath] [-p = Super4PCS filepath] [-d = create root folder with today's date] [-v = verbose] [-k = keep output]
where options are:
<input txt file>: .txt file specifying each Super4PCS registration run.
  Run ./s4p-register.sh -h for more details on how to format this file.
[-i input root filepath]: root directory where reading/reference point clouds will be read.
  (default: S4P_INPUT_ROOT)
[-o output root filepath]: root directory where outputted logs
  and aligned reading point clouds will be read.
  (default: S4P_OUTPUT_ROOT)
[-s scripts filepath]: directory where get_pcl_error.sh and pcl_convert.sh are stored
  (default: REG_SCRIPTS_PATH)
[-p Super4PCS filepath]: directory where the Super4PCS binary is stored.
  (default: S4P_PATH)
[-d]: create a folder in the output root filepath with today's date and store all output in this folder
  (default: do not create folder)
[-v]: verbose mode. Prints info during runtime
  (default: not verbose)
[-k]: keep outputted point clouds from registration and computing error, i.e. aligned reading models
  and reference models with error intensity colors, in pcd and ply. Note that this might take up a lot
  of memory if your models are huge!
  (default: don't keep)
```

#### Input text file

For info on the input text file, refer to the same [E4E project wiki page](https://github.com/UCSD-E4E/maya-archaeology/wiki/3.-Registration-algorithms#super4pcs), or run `./s4p-register.sh -h`:
```
$ ./s4p-register.sh -h
  Each line in the input .txt file is formatted like so:
<referenceRelativeFilepath1> <readingRelativeFilepath1> [outputRelativeFilepath1] [overlap] [delta] [time] [# of samples]
<referenceRelativeFilepath2> <readingRelativeFilepath2> [outputRelativeFilepath2] [overlap] [delta] [time] [# of samples]
...
  where <referenceFilepath> is the location of the .ply reference/source point cloud
    relative to the input root filepath,
  <readingFilepath> is the location of the .ply reading/target point cloud
    relative to the input root filepath,
  [outputFilepath] is the directory where the log file and aligned reading point cloud
    will be stored relative to the output root filepath,
    (default: outputRootFilepath/<currentTime>)
  and the rest are Super4PCS options.
  (default: overlap = 0.6, delta = 0.03, time = 10000, samples = 10000)
To omit an optional option and use defaults, enter '-' without quotes.
```

#### Testing

A sample input text file is in `registration/inputs.txt`. To test the script, run this test input text file like so:
```
# Set S4P_PATH to wherever your Super4PCS binary is located.
$ export S4P_PATH=~/workspace/Super4PCS/build/

$ ./s4p-register.sh inputs.txt -i ../testModels/ -o ../testOutput -s ../general/ -dvk
```

The last 4 runs in inputs.txt should be skipped as the reading/reference files do not exist in the testModels folder.


### `general/get_pcl_error.sh`

Takes in two point clouds in `.vtk`, `.pcd`, or `.ply`, calculates the error between them using PCL with user given parameters, and outputs the target/reference point cloud in `.ply` colored with error intensity. Used by `s4p-register.sh`.  Instructions on how to view the error intensity file in the [E4E project wiki: "Viewing point clouds"](https://github.com/UCSD-E4E/maya-archaeology/wiki/5.-Viewing-point-clouds#pcl_viewer).

### `general/pcl_convert.sh`

Takes in any point cloud in `.vtk`, `.pcd`, or `.ply` and converts it to `.vtk`, `.pcd`, or `.ply` using PCL.  Used by `get_pcl_error.sh`.
