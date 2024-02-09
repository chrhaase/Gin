/*
 ==============================================================================

 This file is part of the GIN library.
 Copyright (c) 2019 - Roland Rabien.

 ==============================================================================
 */


#pragma once

//==============================================================================
/** Track audio levels with decay in dB
*/
class LevelTracker
{
public:
    LevelTracker (float decayPerSecond = 30.0f);

    void trackBuffer (const float* buffer, int numSamples);
    void trackBuffer (juce::AudioSampleBuffer& buffer);
    void trackSample (float f);

    float getLevel() const;
    bool getClip() const    { return clip;  }
    void clearClip()        { clip = false; }

protected:
    std::atomic<float> peakTime  = 0.0f;
    std::atomic<float> peakLevel = -100.0f;
    std::atomic<float> decayRate = 0.0f;
    std::atomic<bool> clip       = false;
};
