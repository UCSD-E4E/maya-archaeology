function rotMatrixToVector(in_file_name, out_file_path)

	file = fopen(in_file_name, 'r');
	formatSpec = '%f';
	sizeMatrix = [3 3];

	matrix = fscanf(file, formatSpec, sizeMatrix);
	matrix = matrix';

	vector = rotationMatrixToVector(matrix);

	fclose(file);

	out_file_name = out_file_path + "rotation_vector.txt";
	file = fopen(out_file_name, 'w');
	fprintf(file, '%f\n', vector);

	fclose(file);

end
