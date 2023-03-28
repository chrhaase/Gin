
//================================================================================
const float DIGITAL_TC = -2.0f; // log(1%)
const float ANALOG_TC = -0.43533393574791066201247090699309f; // (log(36.7%)

//================================================================================
void EnvelopeDetector::reset() { envelope = 0.0; }

void EnvelopeDetector::setParams (float attackS_, float holdS_, float releaseS_, bool analogTC_,
                                  Mode detect_, bool logDetector_)
{
    analogTC = analogTC_;
    mode = detect_;
    logDetector = logDetector_;

    setAttackTime (attackS_);
    setHoldTime (holdS_);
    setReleaseTime (releaseS_);
}

void EnvelopeDetector::setHoldTime (float holdS) { holdTime = holdS; }

void EnvelopeDetector::setAttackTime (float attackS)
{
    if (analogTC)
        attackTime = (float) std::exp (ANALOG_TC / (attackS * sampleRate));
    else
        attackTime = (float) std::exp (DIGITAL_TC / (attackS * sampleRate));
}

void EnvelopeDetector::setReleaseTime (float releaseS)
{
    if (analogTC)
        releaseTime = (float) std::exp (ANALOG_TC / (releaseS * sampleRate));
    else
        releaseTime = (float) std::exp (DIGITAL_TC / (releaseS * sampleRate));
}

float EnvelopeDetector::process (float input)
{
    switch (mode)
    {
        case peak: input = std::fabs (input); break;
        case ms: input = std::fabs (input) * std::fabs (input); break;
        case rms: input = std::pow (std::fabs (input) * std::fabs (input), 0.5f); break;
    }

    if (input > envelope)
    {
        envelope = attackTime * (envelope - input) + input;
        holdRemaining = holdTime;
    }
    else if (holdTime > 0.0f && holdRemaining > 0.0f)
    {
        holdRemaining -= 1.0f / float (sampleRate);
    }
    else
    {
        envelope = releaseTime * (envelope - input) + input;
    }

    envelope = std::max (0.0f, envelope);

    if (logDetector)
    {
        if (envelope == 0.0f)
            return -100.0f;

        return juce::Decibels::gainToDecibels (envelope);
    }

    return envelope;
}

//================================================================================
void Dynamics::setSampleRate (double sampleRate_)
{
    sampleRate = sampleRate_;
    reset();
}

void Dynamics::setNumChannels (int ch)
{
    if (channels != ch)
    {
        channels = ch;
        reset();
    }
}

void Dynamics::setParams (float attackS, float holdS, float releaseS, float threshold_,
                          float ratio_, float kneeWidth_)
{
    envelope.setAttackTime (attackS * 1000.0f);
    envelope.setReleaseTime (releaseS * 1000.0f);

    threshold = threshold_;
    ratio = ratio_;
    kneeWidth = kneeWidth_;
}

void Dynamics::reset()
{
    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.numChannels = channels;
    spec.maximumBlockSize = 2048; // not expected to be used
    envelope.prepare (spec);
    envelope.setLevelCalculationType (juce::dsp::BallisticsFilterLevelCalculationType::RMS);
}

void Dynamics::process (juce::AudioSampleBuffer& buffer, juce::AudioSampleBuffer* envelopeOut)
{
    buffer.applyGain (inputGain);
    inputTracker.trackBuffer (buffer);

    int numSamples = buffer.getNumSamples();

    auto input = buffer.getArrayOfReadPointers();
    auto output = buffer.getArrayOfWritePointers();
    auto env = envelopeOut != nullptr ? envelopeOut->getArrayOfWritePointers() : nullptr;

    for (int i = 0; i < numSamples; i++)
    {
        float peakReduction = 1.0f;
        if (channelsLinked)
        {
            float linked = 0.0f;
            for (int c = 0; c < channels; c++)
            {
                float in = input[c][i];
                linked += envelope.processSample (c, in);
            }

            linked /= float (channels);

            if (env != nullptr)
                env[0][i] = linked;

            linked = juce::Decibels::gainToDecibels (linked);

            auto gain = juce::Decibels::decibelsToGain (calcCurve (linked) - linked);
            peakReduction = std::min (peakReduction, gain);

            for (int c = 0; c < channels; c++)
                output[c][i] = gain * input[c][i] * outputGain;
        }
        else
        {
            for (int c = 0; c < channels; c++)
            {
                float in = inputGain * input[c][i];
                in = envelope.processSample (c, in);

                if (env != nullptr)
                    env[c][i] = juce::Decibels::decibelsToGain (in);

                auto gain = juce::Decibels::decibelsToGain (calcCurve (in) - in);
                peakReduction = std::min (peakReduction, gain);

                output[c][i] = inputGain * gain * input[c][i] * outputGain;
            }
        }
        envelope.snapToZero();
        reductionTracker.trackSample (peakReduction);
    }

    outputTracker.trackBuffer (buffer);
}

float Dynamics::calcCurve (float dbIn)
{
    if (type == compressor)
    {
        float dbOut = dbIn;

        if (kneeWidth > 0 && dbIn >= (threshold - kneeWidth / 2.0f)
            && dbIn <= threshold + kneeWidth / 2.0f)
            dbOut = dbIn
                    + ((1.0f / ratio - 1.0f) * std::pow (dbIn - threshold + kneeWidth / 2.0f, 2.0f)
                       / (2.0f * kneeWidth));
        else if (dbIn > threshold + kneeWidth / 2.0)
            dbOut = threshold + (dbIn - threshold) / ratio;

        return dbOut;
    }
    else if (type == limiter)
    {
        float dbOut = dbIn;

        if (kneeWidth > 0 && dbIn >= (threshold - kneeWidth / 2.0f)
            && dbIn <= threshold + kneeWidth / 2.0f)
            dbOut = dbIn
                    + (1.0f * std::pow (dbIn - threshold + kneeWidth / 2.0f, 2.0f)
                       / (2.0f * kneeWidth));
        else if (dbIn > threshold + kneeWidth / 2.0f)
            dbOut = threshold;

        return dbOut;
    }
    else if (type == expander)
    {
        float dbOut = dbIn;

        // soft-knee with detection value in range?
        if (kneeWidth > 0 && dbIn >= (threshold - kneeWidth / 2.0f)
            && dbIn <= threshold + kneeWidth / 2.0f)
            dbOut = dbIn
                    - ((ratio - 1.0f) * std::pow ((dbIn - threshold - (kneeWidth / 2.0f)), 2.0f))
                          / (2.0f * kneeWidth);
        else if (dbIn < threshold + kneeWidth / 2.0)
            dbOut = threshold + (dbIn - threshold) * ratio;

        return dbOut;
    }
    else if (type == gate)
    {
        float dbOut = dbIn;

        if (kneeWidth > 0 && dbIn >= (threshold - kneeWidth / 2.0f)
            && dbIn <= threshold + kneeWidth / 2.0f)
            dbOut = dbIn
                    - ((100.0f - 1.0f) * std::pow ((dbIn - threshold - (kneeWidth / 2.0f)), 2.0f))
                          / (2.0f * kneeWidth);
        else if (dbIn < threshold - kneeWidth / 2.0f)
            dbOut = -1000.0f;

        return dbOut;
    }

    jassertfalse;
    return dbIn;
}
