int soundID = -1;

task main() {
	while (true) {
		if (soundID != -1) {
			playSound(soundID);

			while (bSoundActive);

			soundID = -1;
		}
	}
}
