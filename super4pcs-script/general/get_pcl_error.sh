#!/bin/bash

# Functions---------------
printUsage() {
	echo "USAGE: $0 <source> <target> [-l logFilename] [-o outputFilename] [-s scriptsPath] [-c {nn|nnplane|index}]" 1>&2
	echo "where options are:"
	echo "<source>: reference point cloud in .vtk, .ply, or .pcd"
	echo "<target>: reading (aligned) point cloud in .vtk, .ply, or .pcd"
	echo "[-l logFilename] = name of log file to append the error output"
	echo "  (default: log.txt)"
	echo "[-o outputFilename] = name of output file which is a reference point cloud"
	echo "	to be generated with error intensity respective to target."
	echo "	(default: [targetName]-out.ply)"
	echo "[-s scriptsPath] = directory where pcl_convert.sh is stored"
	echo "  (default: REG_SCRIPTS_PATH)"
	echo "[-c X] = correspondence for error, which is"
	echo "	the way of selecting the corresponding pair in the target cloud"
	echo "	for the current point in the source cloud."
	echo "		(default: nn)"
	echo "	options are:"
	echo "		index = points with identical indices are paired together."
	echo "		  Note: both clouds need to have the same number of points."
	echo "		nn = source point is paired with its nearest neighbor"
	echo "			in the target cloud."
	echo "		nnplane = source point is paired with its projection"
	echo "			on the plane determined by the nearest neighbor"
	echo "			in the target cloud."
	echo "		  Note: target cloud needs to contain normals."
	exit -1
}

printFormatError() {
	echo "Unknown input file format for" $1 1>&2
	echo "Needs to be .vtk, .ply, or .pcd" 1>&2
	exit -1
}

checkInputFiles() {
	if [[ -z ${inputFiles[0]} ]]; then
		echo "Source filename not specified." 1>&2
		printUsage
	elif [[ -z ${inputFiles[1]} ]]; then
		echo "Target filename not specified." 1>&2
		printUsage
	else
		for file in ${inputFiles[@]}; do
			if [[ ! -e $file ]]; then
				echo "Input file" $file "not found." 1>&2
				exit -1
			fi
			case ${file: -4} in
				".pcd"|".ply"|".vtk") continue;;
				*) printFormatError $file;;
			esac
		done
	fi
}

setLog() {
	if [[ -z $logFilename ]]; then
		echo "Log filename not specified."
		logFilename="log.txt"
		echo "Setting log filename to" $logFilename
	fi
}

setOutput() {
	# if output is .pcd or .ply file, accept it
	if [[ -z $output ]]; then
		echo "Output filename not specified."
		output=${inputFiles[1]%.*}-out.ply
		echo "Setting output filename to" $output
	elif [[ ${output: -4} != ".pcd" ]] && [[ ${output: -4} != ".ply" ]]; then
		echo "Output filename must be .pcd or .ply." 1>&2
		output=${inputFiles[1]%.*}-out.ply
		echo "Setting output filename to" $output
	fi
}

setScriptsPath() {
	# Default for scripts path: Set to REG_SCRIPTS_PATH
	if [[ -z $scriptsPath ]]; then
		if [[ ! -z $REG_SCRIPTS_PATH ]]; then
			if [[ $verbose ]]; then
				echo "Setting scripts filepath to" $REG_SCRIPTS_PATH
			fi
			scriptsPath=$REG_SCRIPTS_PATH
		else
			echo "No scripts filepath. REG_SCRIPTS_PATH not set." 1>&2
			exit -1
		fi
	fi

	# Remove ending / in scripts filepath if present
	if [[ ${scriptsPath: -1 } == "/" ]]; then
		scriptsPath=${scriptsPath%/}
	fi
}

setCorrespondence() {
	if [[ $correspondence != "nn" ]] && [[ $correspondence != "nnplane" ]] && [[ $correspondence != "index" ]]; then
		echo "Unknown correspondence:" $correspondence 1>&2
		echo "Setting correspondence to nn"
		correspondence="nn"
	fi
}

#----------------------------
# Handle source and target
inputFiles[0]=$1
inputFiles[1]=$2
# Shift arguments to be able to use getopts with flags
shift
shift
checkInputFiles

# Fill in other flags
while getopts 'l:o:s:c:' flag; do
	case "${flag}" in
		l) logFilename=${OPTARG};;
		o) output=${OPTARG};;
		s) scriptsPath=${OPTARG};;
		c) correspondence=${OPTARG};;
	esac
done

# Process other flags
setLog
setOutput
setScriptsPath
setCorrespondence

#Convert input files to pcd if necessary
for fileIndex in ${!inputFiles[@]}; do
	file=${inputFiles[$fileIndex]}

	pcdFile=${file%.*}.pcd
	$scriptsPath/./pcl_convert.sh $file $pcdFile -v
	if [[ $? == 2 ]]; then
		echo "Using" $pcdFile "to compute cloud error."
	fi
	inputFiles[$fileIndex]=$pcdFile
done

# Get error
if [ "${output: -4}" == ".ply" ]; then
	outputPCD=${output%.ply}.pcd
	# Record command used in log file
	echo "Running: $ pcl_compute_cloud_error ${inputFiles[0]} ${inputFiles[1]} $outputPCD -correspondence $correspondence" | tee -a $logFilename

	echo "Saving error to" $logFilename
	pcl_compute_cloud_error ${inputFiles[0]} ${inputFiles[1]} $outputPCD -correspondence $correspondence 2>&1 | tee -a $logFilename

	# Convert output to ply if computing cloud error succeeded
	if [[ $? == 0 ]]; then
		$scriptsPath/./pcl_convert.sh $outputPCD $output -v -f

	else
		echo "Computing cloud error failed." 1>&2
		exit -1
	fi
else
	# Record command used in log file
	echo "Running: $ pcl_compute_cloud_error ${inputFiles[0]} ${inputFiles[1]} $output -correspondence $correspondence" | tee -a $logFilename

	echo "Saving error to" $logFilename
	pcl_compute_cloud_error ${inputFiles[0]} ${inputFiles[1]} $output -correspondence $correspondence 2>&1 | tee -a $logFilename
fi

exit 0
