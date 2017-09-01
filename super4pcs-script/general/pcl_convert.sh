#!/bin/bash

OUTPUT_EXISTS_RET=2 # return code if output already exists

# Functions ------------------
printUsage() {
	echo "USAGE: $0 <input> <output> [-v verbose] [-f force]" 1>&2
	echo "where options are:"
	echo "<input>: point cloud in .vtk, .ply, or .pcd"
	echo "<output>: point cloud in .vtk, .ply, or .pcd"
	echo "[-v verbose] = print more info during runtime"
	echo "[-f force] = overwrite output file if it already exists"
	exit -1
}

printFormatError() {
	echo "Unknown file format for" $1 1>&2
	echo "Needs to be .vtk, .ply, or .pcd" 1>&2
	exit -1
}

toPCD() {
	file=$1
	output=$2
	retCode=0
	# Check if output file already exists
	if [[ -e $output ]] && [[ ! $force ]]; then
		echo "pcd file" $output "already exists." 1>&2
		retCode=$OUTPUT_EXISTS_RET
		exit $retCode
	# convert .vtk to .pcd
	elif [[ "${file: -4}" == ".vtk" ]]; then
		pcl_vtk2pcd $file $output
		retCode=$?
		# If PCL 1.7 doesn't work, try PCL 1.8
		if [[ $retCode != 0 ]]; then
			if [[ $verbose ]]; then
				echo "Using PCL 1.8"
			fi
			pcl1.8_vtk2pcd $file $output
			retCode=$?
			if [[ $retCode != 0 ]]; then # Error with PCL 1.8
				echo "Error" $retCode "converting" $file "to" $output 1>&2
				exit $retCode
			fi
		fi
	# convert .ply to .pcd
	elif [[ "${file: -4}" == ".ply" ]]; then
		pcl_ply2pcd $file $output
		retCode=$?
		# If PCL 1.7 doesn't work, try PCL 1.8
		if [[ $retCode != 0 ]]; then
			if [[ $verbose ]]; then
				echo "Using PCL 1.8"
			fi
			pcl1.8_ply2pcd $file $output
			retCode=$?
			if [[ $retCode != 0 ]]; then # Error with PCL 1.8
				echo "Error" $retCode "converting" $file "to" $output 1>&2
				exit $retCode
			fi
		fi
	# If input was in pcd, then just copy it over to get output
	elif [[ "${file: -4}" == ".pcd" ]]; then
		if [[ $verbose ]]; then
			echo "Copying" $file "to" $output
		fi
		cp $file $output
		exit $retCode
	fi
	echo "Converted" $file "to" $output
}

toPLY() {
	file=$1
	output=$2
	retCode=0
	# Check if output file already exists
	if [[ -e $output ]] && [[ ! $force ]]; then
		echo "ply file" $output "already exists." 1>&2
		retCode=$OUTPUT_EXISTS_RET
		exit $retCode
	# convert .vtk to .ply
	elif [[ "${file: -4}" == ".vtk" ]]; then
		pcl_vtk2ply $file $output
		retCode=$?
		# If PCL 1.7 doesn't work, try PCL 1.8
		if [[ $retCode != 0 ]]; then
			if [[ $verbose ]]; then
				echo "Using PCL 1.8"
			fi
			pcl1.8_vtk2ply $file $output
			retCode=$?
			if [[ $retCode != 0 ]]; then # Error with PCL 1.8
				echo "Error" $retCode "converting" $file "to" $output 1>&2
				exit $retCode
			fi
		fi
	# convert .pcd to .ply
	elif [[ "${file: -4}" == ".pcd" ]]; then
		pcl_pcd2ply $file $output
		retCode=$?
		# If PCL 1.7 doesn't work, try PCL 1.8
		if [[ $retCode != 0 ]]; then
			if [[ $verbose ]]; then
				echo "Using PCL 1.8"
			fi
			pcl1.8_pcd2ply $file $output
			retCode=$?
			if [[ $retCode != 0 ]]; then # Error with PCL 1.8
				echo "Error" $retCode "converting" $file "to" $output 1>&2
				exit $retCode
			fi
		fi
	# If input was in ply, then just copy it over to get output
	elif [[ "${file: -4}" == ".ply" ]]; then
		if [[ $verbose ]]; then
			echo "Copying" $file "to" $output
		fi
		cp $file $output
		exit $retCode
	fi
	echo "Converted" $file "to" $output
}

toVTK() {
	file=$1
	output=$2
	retCode=0
	# Check if output file already exists
	if [[ -e $output ]] && [[ ! $force ]]; then
		echo "vtk file" $output "already exists." 1>&2
		retCode=$OUTPUT_EXISTS_RET
		exit $retCode
	# convert .pcd to .vtk
	elif [[ "${file: -4}" == ".pcd" ]]; then
		pcl_pcd2vtk $file $output
		retCode=$?
		# If PCL 1.7 doesn't work, try PCL 1.8
		if [[ $retCode != 0 ]]; then
			if [[ $verbose ]]; then
				echo "Using PCL 1.8"
			fi
			pcl1.8_pcd2vtk $file $output
			retCode=$?
			if [[ $retCode != 0 ]]; then # Error with PCL 1.8
				echo "Error" $retCode "converting" $file "to" $output 1>&2
				exit $retCode
			fi
		fi
	# convert .ply to .vtk
	elif [[ "${file: -4}" == ".ply" ]]; then
		pcl_ply2vtk $file $output
		retCode=$?
		# If PCL 1.7 doesn't work, try PCL 1.8
		if [[ $retCode != 0 ]]; then
			if [[ $verbose ]]; then
				echo "Using PCL 1.8"
			fi
			pcl1.8_ply2vtk $file $output
			retCode=$?
			if [[ $retCode != 0 ]]; then # Error with PCL 1.8
				echo "Error" $retCode "converting" $file "to" $output 1>&2
				exit $retCode
			fi
		fi
	# If input was in vtk, then just copy it over to get output
	elif [[ "${file: -4}" == ".vtk" ]]; then
		if [[ $verbose ]]; then
			echo "Copying" $file "to" $output
		fi
		cp $file $output
		exit $retCode
	fi
	echo "Converted" $file "to" $output
}

#--------------------------------
# Fill in args
input=$1
output=$2
# Shift so getopts can be used for other flags
shift
shift

# Fill in other flags
while getopts 'vf' flag; do
	case "${flag}" in
		v) verbose=true;;
		f) force=true;;
	esac
done

if [[ -z $input ]]; then
	echo "Input file not specified for conversion." 1>&2
	printUsage
elif [[ -z $output ]]; then
	echo "Output file not specified for conversion." 1>&2
	printUsage
elif [[ ! -e $input ]]; then
	echo "Input file" $input "not found for conversion." 1>&2
	exit -1
fi

# Check if file formats are in PCD, PLY, or VTK
case ${input: -4} in
	".pcd"|".ply"|".vtk") ;;
	*) printFormatError $input;;
esac

# Convert output
case ${output: -4} in
	".pcd") toPCD $input $output;;
	".ply") toPLY $input $output;;
	".vtk") toVTK $input $output;;
	*) printFormatError $output;;
esac


exit 0
