# maya-archaeology
various pipelines for slam, registration, error comparison algorithms for maya tunnel scanning

## go-icp script
	install GoICP
	install PCL
	
To install:
```
cd go-icp-script
mkdir build
cd build
cmake ..
make
```

To run (config.txt - adjust parameters for registration)
```
cd go-icp-script/scripts
python go-icp.py lidar.ply slam.ply
```
where lidar.ply and slam.ply are the reference and reading models, respectively

Model Files will be located in 
```go-icp-script/models/```
	-> lidar.ply (original reference file)
	-> mat.txt (transformation matrix file)
	-> slam-t.ply (transformed slam file)
	-> error-p2p.pcd (error file of point to point error)
	-> error-p2pl.pcd (error file of point to plane error)

