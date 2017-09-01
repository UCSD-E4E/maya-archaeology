#!/bin/bash

# default Super4PCS options
OVERLAP_DEF=0.6
DELTA_DEF=0.03
TIME_DEF=10000
SAMPLES_DEF=10000

# default output file names
MAT_FILE="matrix.txt"
LOG_FILE="log.txt"
ALIGNED_READING_PCD="aligned-reading.pcd"
ALIGNED_READING_PLY=${ALIGNED_READING_PCD%.pcd}.ply
REF_ERROR_PCD_PT="ref-error-ptToPt.pcd"
REF_ERROR_PLY_PT=${REF_ERROR_PCD_PT%.pcd}.ply
REF_ERROR_PCD_PLANE="ref-error-ptToPlane.pcd"
REF_ERROR_PLY_PLANE=${REF_ERROR_PCD_PLANE%.pcd}.ply

# default params
needDateFolder=false
verbose=false
keepOutput=false


# Functions for preparing args ---------------
printUsage() {
	echo "USAGE: $0 <input txt file> [-i input root filepath] [-o output root filepath] [-s = scripts filepath] [-p = Super4PCS filepath] [-d = create root folder with today's date] [-v = verbose] [-k = keep output]" 1>&2
	echo "where options are:"
	echo "<input txt file>: .txt file specifying each Super4PCS registration run."
	echo "  Run $0 -h for more details on how to format this file."
	echo "[-i input root filepath]: root directory where reading/reference point clouds will be read."
	echo "  (default: S4P_INPUT_ROOT)"
	echo "[-o output root filepath]: root directory where outputted logs"
	echo "  and aligned reading point clouds will be read."
	echo "  (default: S4P_OUTPUT_ROOT)"
	echo "[-s scripts filepath]: directory where get_pcl_error.sh and pcl_convert.sh are stored"
	echo "  (default: REG_SCRIPTS_PATH)"
	echo "[-p Super4PCS filepath]: directory where the Super4PCS binary is stored."
	echo "  (default: S4P_PATH)"
	echo "[-d]: create a folder in the output root filepath with today's date and store all output in this folder"
	echo "  (default: do not create folder)"
	echo "[-v]: verbose mode. Prints info during runtime"
	echo "  (default: not verbose)"
	echo "[-k]: keep outputted point clouds from registration and computing error, i.e. aligned reading models"
	echo "  and reference models with error intensity colors, in pcd and ply. Note that this might take up a lot"
	echo "  of memory if your models are huge!"
	echo "  (default: don't keep)"
	exit -1
}

printTxtFileFormat() {
	echo "  Each line in the input .txt file is formatted like so:"
	echo "<referenceRelativeFilepath1> <readingRelativeFilepath1> [outputRelativeFilepath1] [overlap] [delta] [time] [# of samples]"
	echo "<referenceRelativeFilepath2> <readingRelativeFilepath2> [outputRelativeFilepath2] [overlap] [delta] [time] [# of samples]"
	echo "..."
	echo "  where <referenceFilepath> is the location of the .ply reference/source point cloud"
	echo "    relative to the input root filepath,"
	echo "  <readingFilepath> is the location of the .ply reading/target point cloud"
	echo "    relative to the input root filepath,"
	echo "  [outputFilepath] is the directory where the log file and aligned reading point cloud"
	echo "    will be stored relative to the output root filepath,"
	echo "    (default: outputRootFilepath/<currentTime>)"
	echo "  and the rest are Super4PCS options."
	echo "  (default: overlap = 0.6, delta = 0.03, time = 10000, samples = 10000)"
	echo "To omit an optional option and use defaults, enter '-' without quotes."
	exit -1
}

# Create folder with today's date in output root directory
createDateFolder() {
	folderName=`date +'%m-%d-%y'`
	if [[ ! -d $outputRoot/$folderName ]]; then
		if [[ "$verbose" == "true" ]]; then
			echo "Creating folder with today's date in output root directory" $outputRoot/$folderName
		fi
		mkdir $outputRoot/$folderName
	fi
	outputRoot=$outputRoot/$folderName
}

# Set input and output roots to defaults if needed
setRoots() {
	# Default for input root directory: Set to S4P_INPUT_ROOT
	if [[ -z $inputRoot ]]; then
		if [[ ! -z $S4P_INPUT_ROOT ]]; then
			if [[ "$verbose" == "true" ]]; then
				echo "Setting input root filepath to" $S4P_INPUT_ROOT
			fi
			inputRoot=$S4P_INPUT_ROOT
		else
			echo "No input root filepath. S4P_INPUT_ROOT not set." 1>&2
			exit -1
		fi
	fi

	# Default for output root directory: Set to S4P_OUTPUT_ROOT
	if [[ -z $outputRoot ]] && [[ ! -z $S4P_OUTPUT_ROOT ]]; then
		if [[ "$verbose" == "true" ]]; then
			echo "Setting output root filepath to" $S4P_OUTPUT_ROOT
		fi
		outputRoot=$S4P_OUTPUT_ROOT
	elif [[ -z $outputRoot ]]; then
		echo "No output root filepath. S4P_OUTPUT_ROOT not set." 1>&2
		exit -1
	fi

	# Remove ending / in root filepath if present
	if [[ ${inputRoot: -1 } == "/" ]]; then
		inputRoot=${inputRoot%/}
	fi
	if [[ ${outputRoot: -1 } == "/" ]]; then
		outputRoot=${outputRoot%/}
	fi

	# Make directories for user inputted input and output roots if they don't exist
	if [[ ! -d $inputRoot ]]; then
		if [[ "$verbose" == "true" ]]; then
			echo "Creating input root directory" $inputRoot
		fi
		mkdir -p $inputRoot
	fi
	if [[ ! -d $outputRoot ]]; then
		if [[ "$verbose" == "true" ]]; then
			echo "Creating output root directory" $outputRoot
		fi
		mkdir -p $outputRoot
	fi
}

setScriptsPath() {
	# Default for scripts path: Set to REG_SCRIPTS_PATH
	if [[ -z $scriptsPath ]]; then
		if [[ ! -z $REG_SCRIPTS_PATH ]]; then
			if [[ "$verbose" == "true" ]]; then
				echo "Setting scripts filepath to" $REG_SCRIPTS_PATH
			fi
			scriptsPath=$REG_SCRIPTS_PATH
		else
			echo "No scripts filepath. REG_SCRIPTS_PATH not set." 1>&2
			exit -1
		fi
	fi

	# Default for Super4PCS path: Set to S4P_PATH
	if [[ -z $s4pPath ]]; then
		if [[ ! -z $S4P_PATH ]]; then
			if [[ "$verbose" == "true" ]]; then
				echo "Setting Super4PCS filepath to" $S4P_PATH
			fi
			s4pPath=$S4P_PATH
		else
			echo "No Super4PCS binary filepath. S4P_PATH not set." 1>&2
			exit -1
		fi
	fi

	# Remove ending / in scripts filepath if present
	if [[ ${scriptsPath: -1 } == "/" ]]; then
		scriptsPath=${scriptsPath%/}
	fi

	# Remove ending / in Super4PCS filepath if present
	if [[ ${s4pPath: -1 } == "/" ]]; then
		s4pPath=${s4pPath%/}
	fi
}

# Sanity check input text file
checkTxtFile() {
	txtFile=$1
	if [[ -z $txtFile ]]; then
		echo "No input txt file specified." 1>&2
		printUsage
	elif [[ ! -e $txtFile ]]; then
		echo "Input txt file does not exist." 1>&2
		exit -1
	fi
}

# Functions for running Super4PCS ----------------
setFilepaths() {
	ref=$inputRoot/$ref
	reading=$inputRoot/$reading
	if [[ ! -z $output ]] || [[ $output == "-" ]]; then
		output=$outputRoot/$output
		if [[ ! -d $output ]]; then
			mkdir -p $output
		fi
	else
		# if no subfolder in output root directory specified, create subfolder with current time.
		currTime=`date +'%H-%M-%S'`
		mkdir $outputRoot/$currTime
		output=$outputRoot/$currTime
	fi

	if [[ ! -e $ref ]]; then
		echo "Reference file" $ref "does not exist." 1>&2
		return -1
	elif [[ ! -e $reading ]]; then
		echo "Reading file" $reading "does not exist." 1>&2
		return -1
	fi

	# input to Super4PCS must be ply
	if [[ ${ref: -4} != ".ply" ]]; then
		$scriptsPath/./pcl_convert.sh $ref ${ref%.*}.ply
		ref=${ref%.*}.ply
	fi
	if [[ ${reading: -4} != ".ply" ]]; then
		$scriptsPath/./pcl_convert.sh $reading ${reading%.*}.ply
		reading=${reading%.*}.ply
	fi

	return 0
}

# Set default Super4PCS options if necessary
setS4Popts() {
	if [[ -z $deltaOpt ]]; then
		deltaOpt=$DELTA_DEF
		timeOpt=$TIME_DEF
		samplesOpt=$SAMPLES_DEF
		if [[ -z $overlapOpt ]]; then
			overlapOpt=$OVERLAP_DEF
		fi
		return
	fi

	if [[ $overlapOpt == "-" ]]; then overlapOpt=$OVERLAP_DEF; fi
	if [[ $deltaOpt == "-" ]]; then deltaOpt=$DELTA_DEF; fi
	if [[ $timeOpt == "-" ]]; then timeOpt=$TIME_DEF; fi
	if [[ $samplesOpt == "-" ]]; then samplesOpt=$SAMPLES_DEF; fi
}

# Run Super4PCS with arguments passed in.
# format: "<referenceRelativeFilepath1> <readingRelativeFilepath1> [outputRelativeFilepath1] [overlap] [delta] [time] [# of samples]"
# Time is outputted in seconds
# Output is stored in a log file in the specified outputRelativeFilepath
runS4P() {
	ref=$1
	reading=$2
	output=$3
	overlapOpt=$4
	deltaOpt=$5
	timeOpt=$6
	samplesOpt=$7

	setS4Popts
	setFilepaths
	retCode=$?

	# Run Super4PCS here
	if [[ $retCode == 0 ]]; then
		# record command used in log file
		echo "Running: $ time -p $s4pPath/./Super4PCS -i $ref $reading -o $overlapOpt -d $deltaOpt -t $timeOpt -n $samplesOpt -m $output/$MAT_FILE" >> $output/$LOG_FILE

		if [[ "$verbose" == "true" ]]; then
			echo "Running: $ time -p $s4pPath/./Super4PCS -i $ref $reading -o $overlapOpt -d $deltaOpt -t $timeOpt -n $samplesOpt -m $output/$MAT_FILE"
		fi

		(time -p $s4pPath/./Super4PCS -i $ref $reading -o $overlapOpt -d $deltaOpt -t $timeOpt -n $samplesOpt -m $output/$MAT_FILE) 2>&1 | tee -a $output/$LOG_FILE
	else
		echo "Skipping this run." 1>&2
	fi

	return $retCode
}

# -- Aligning reading point cloud with matrix --
alignReading() {
	# Convert reading point cloud from ply to pcd if necessary
	readingPCD=${reading%.ply}.pcd
	if [[ ! -e $readingPCD ]]; then
		if [[ "$verbose" == "true" ]]; then
			echo "Converting" $reading "to" $readingPCD "for alignment"
		fi
		$scriptsPath/./pcl_convert.sh $reading $readingPCD -f
	fi

	# Parse transformation matrix outputted by Super4PCS
	# and format it so it can be used with PCL
	# matrix starts on line 3 of outputted -m txt file from S4P
	transformMat=`tail -n +3 $output/$MAT_FILE` # get transformation matrix from Super4PCS output
	transformMat=${transformMat//  /,} # convert spaces to commas
	transformMat=${transformMat//$'\n'/,} # convert newlines to commas
	transformMat=${transformMat// /} # strip trailing spaces from beginning of line 

	if [[ "$verbose" == "true" ]]; then
		echo "Transforming reading point cloud with Super4PCS alignment" $transformMat
	fi
	pcl_transform_point_cloud $readingPCD $output/$ALIGNED_READING_PCD -matrix $transformMat
	# Convert aligned reading point cloud from pcd to ply
	if [[ "$verbose" == "true" ]]; then
		echo "Converting aligned reading point cloud" $output/$ALIGNED_READING_PCD "to" $output/$ALIGNED_READING_PLY
	fi
	$scriptsPath/./pcl_convert.sh $output/$ALIGNED_READING_PCD $output/$ALIGNED_READING_PLY -f

	# -- Get error --
	if [[ "$verbose" == "true" ]]; then
		echo "Getting point-to-point error and saving in" $output/$LOG_FILE
	fi
	$scriptsPath/./get_pcl_error.sh $ref $output/$ALIGNED_READING_PCD -l $output/$LOG_FILE -o $output/$REF_ERROR_PLY_PT -c nn -s $scriptsPath
	if [[ "$verbose" == "true" ]]; then
		echo "Getting point-to-plane error and saving in" $output/$LOG_FILE
	fi
	$scriptsPath/./get_pcl_error.sh $ref $output/$ALIGNED_READING_PCD -l $output/$LOG_FILE -o $output/$REF_ERROR_PLY_PLANE -c nnplane -s $scriptsPath
}

# if not keeping outputted point clouds, remove them after each S4P run.
removeOutputPointClouds() {
	if [[ "$verbose" == "true" ]]; then
		echo "Removing outputted point clouds."
	fi
	rm $output/$ALIGNED_READING_PCD
	rm $output/$ALIGNED_READING_PLY
	rm $output/$REF_ERROR_PCD_PT
	rm $output/$REF_ERROR_PCD_PLANE
	rm $output/$REF_ERROR_PLY_PT
	rm $output/$REF_ERROR_PLY_PLANE
}

# Go through the input text file and run the arguments in each line through Super4PCS.
runTxtFile() {
	while read line; do
		runS4P $line
		if [[ $? == 0 ]]; then
			alignReading
			if [[ "$keepOutput" != "true" ]]; then
				removeOutputPointClouds
			fi
		fi
	done < $1
}


# What actually gets run in the script ----------------------------
# --Preparing arguments--
# Check if need to print the input text file format
if [[ $1 == "-h" ]]; then
	printTxtFileFormat
fi

# Fill in txt file name
checkTxtFile $1
shift # shift to be able to use getopts with the rest of the flags

# Fill in flags
while getopts 'i:o:s:p:dvhk' flag; do
	case "${flag}" in
		i) inputRoot=${OPTARG};;
		o) outputRoot=${OPTARG};;
		s) scriptsPath=${OPTARG};;
		p) s4pPath=${OPTARG};;
		d) needDateFolder=true;;
		v) verbose=true;;
		h) printTxtFileFormat;;
		k) keepOutput=true;;
	esac
done

# Process params and fill in defaults as necessary
setRoots
setScriptsPath

if [[ "$needDateFolder" == "true" ]]; then
	createDateFolder
fi

# -- Running Super4PCS --
runTxtFile $txtFile


exit 0
