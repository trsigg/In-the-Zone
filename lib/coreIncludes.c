enum angleType { DEGREES, RADIANS, RAW };

int limit(float input, float min, float max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

float convertAngle(float angle, angleType output, angleType input=RAW) {
	if (input != output) {
		//convert input to RAW
		if (input == DEGREES) {
			angle *= 10;
		} else if (input == RADIANS) {
			angle *= 1800 / PI;
		}

		if (output == DEGREES) {
			angle /= 10;
		} else if (output == RADIANS) {
			angle *= PI / 1800;
		}
	}

	return angle;
}

void arrayCopy(int* source, int* destination, int elements) {
	for (int i=0; i<elements; i++)
		destination[i] = source[i];
}

float tan(float x) {
	return sin(x)/cos(x);
}
