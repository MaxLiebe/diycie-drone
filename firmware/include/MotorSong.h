#ifndef MOTOR_SONG_H
#define MOTOR_SONG_H

/*********************************************************************
 *  MotorSong.h  –  Play MIDI-derived songs through brushed motors
 *
 *  Usage:
 *    1. Convert a MIDI file to a MotorSong::Note array using the
 *       Python helper script (midi_to_cpp.py) shipped alongside.
 *    2. Instantiate MotorSong, attach up to 4 motor pins.
 *    3. Call play() from loop() – it is NON-BLOCKING.
 *
 *  Platform:  ESP32 (Arduino core 3.x)  –  uses analogWrite()
 *********************************************************************/

#include <Arduino.h>
#include <math.h>

class MotorSong
{
public:
    // ----------------------------------------------------------------
    // One note in a song.
    //   midiNote  : MIDI note number 0-127 (0 = REST / silence)
    //   durationMs: how long to hold the note, in milliseconds
    // ----------------------------------------------------------------
    struct Note
    {
        uint8_t midiNote; // 0 = REST
        uint16_t durationMs;
    };

    // ----------------------------------------------------------------
    // One 'frame' in a polyphonic song.
    //   midiNotes[4] : MIDI note for each motor (0 = REST)
    //   durationMs   : how long to hold this chord
    // ----------------------------------------------------------------
    struct PolyNote
    {
        uint8_t midiNotes[4];
        uint16_t durationMs;
    };

    // ----------------------------------------------------------------
    // Constructor
    //   motorPins[]  : array of up to 4 GPIO pins
    //   numPins      : 1 – 4
    //   volume       : analogWrite duty (0-255) applied while playing
    //                  Default = 12  (~5 %, enough to buzz, not spin)
    // ----------------------------------------------------------------
    MotorSong(const int motorPins[], uint8_t numPins, uint8_t volume = 12)
        : _numPins(numPins > 4 ? 4 : numPins),
          _volume(volume),
          _song(nullptr),
          _songLen(0),
          _noteIdx(0),
          _noteStart(0),
          _playing(false),
          _restoreFreq(16000)
    {
        for (uint8_t i = 0; i < _numPins; ++i)
            _pins[i] = motorPins[i];
    }

    // ----------------------------------------------------------------
    // Setters (call before play())
    // ----------------------------------------------------------------

    // Override the volume / duty cycle (0-255)
    void setVolume(uint8_t v) { _volume = v; }

    // Frequency to restore to after playback ends (your flight freq.)
    void setRestoreFrequency(uint32_t hz) { _restoreFreq = hz; }

    // ----------------------------------------------------------------
    // Begin a song.
    //   song[]  : pointer to a Note array in PROGMEM or RAM
    //   len     : number of notes in the array
    // ----------------------------------------------------------------
    void begin(const Note *song, uint16_t len)
    {
        _song = song;
        _songLen = len;
        _noteIdx = 0;
        _playing = true;
        _noteStart = 0; // force first note to start immediately
        _startNote(0);
    }

    // ----------------------------------------------------------------
    // Non-blocking update – call every loop() iteration.
    // Returns true while the song is still playing.
    // ----------------------------------------------------------------
    bool play()
    {
        if (!_playing)
            return false;
        if (!_song)
            return false;

        uint32_t now = millis();

        // Check whether the current note has expired
        if (now - _noteStart >= _song[_noteIdx].durationMs)
        {
            // Silence between notes (small 5 ms gap prevents clicks)
            _silence();
            delay(5);

            ++_noteIdx;
            if (_noteIdx >= _songLen)
            {
                // Song finished
                _playing = false;
                _restore();
                return false;
            }
            _startNote(_noteIdx);
        }
        return true;
    }

    // Stop playback immediately and restore flight frequency
    void stop()
    {
        _playing = false;
        _silence();
        _restore();
    }

    bool isPlaying() const { return _playing; }

    // ----------------------------------------------------------------
    // Static helper: MIDI note number → frequency in Hz
    //   f = 440 * 2^((N - 69) / 12)
    // ----------------------------------------------------------------
    static float midiToHz(uint8_t midiNote)
    {
        return 440.0f * powf(2.0f, (midiNote - 69) / 12.0f);
    }

private:
    int _pins[4];
    uint8_t _numPins;
    uint8_t _volume;

    const Note *_song;
    uint16_t _songLen;
    uint16_t _noteIdx;
    uint32_t _noteStart;
    bool _playing;
    uint32_t _restoreFreq;

    void _startNote(uint16_t idx)
    {
        _noteStart = millis();
        const Note &n = _song[idx];

        if (n.midiNote == 0)
        {
            // REST  keep motors silent
            _silence();
        }
        else
        {
            float freq = midiToHz(n.midiNote);
            analogWriteFrequency((uint32_t)freq);
            for (uint8_t i = 0; i < _numPins; ++i)
            {
                // Re-enable PWM output just in case digitalWrite killed the routing
                pinMode(_pins[i], OUTPUT);
                analogWrite(_pins[i], _volume);
            }
        }
    }

        void _silence()
    {
        for (uint8_t i = 0; i < _numPins; ++i)
        {
            analogWrite(_pins[i], 0);
            digitalWrite(_pins[i], LOW); // Force the pin totally low
        }
    }

    void _restore()
    {
        analogWriteFrequency(_restoreFreq);
        analogWriteResolution(8);
    }
};

#endif // MOTOR_SONG_H
