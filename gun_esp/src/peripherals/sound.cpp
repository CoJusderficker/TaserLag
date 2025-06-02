#include "sound.h"
#include "config.h"
#include <DFRobotDFPlayerMini.h> // has to be 1.0.5, newer will get stuck on play()!!!
#include <SoftwareSerial.h>

DFRobotDFPlayerMini audio;
SoftwareSerial AudioSerial(PIN_AUDIO_RX, PIN_AUDIO_TX);

void sound::setup() {
  AudioSerial.begin(9600);

  for (int i = 0; i < 10; i++) {
    if (audio.begin(AudioSerial, true, true)) {
      Serial.println("DFPlayer Mini online.");

      audio.volume(15);
      audio.play(SOUND_PEW);
      break;
    } else {
      Serial.println("Audio Init Error.");
      delay(500);
    }
  }
}

void sound::play(int index) { audio.play(index); }