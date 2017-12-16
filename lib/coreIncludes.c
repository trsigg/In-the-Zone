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

void arrayCopy(void* source, void* destination, int elements) {
	for (int i=0; i<elements; i++)
		destination[i] = source[i];
}

float copysign(float sign, float magnitude) {
	return sgn(sign) * fabs(magnitude);
}

float min(float val1, float val2) {
	return (val1>val2 ? val2 : val1);
}

float max(float val1, float val2) {
	return (val1>val2 ? val1 : val2);
}

float tan(float x) {
	return sin(x)/cos(x);
}
