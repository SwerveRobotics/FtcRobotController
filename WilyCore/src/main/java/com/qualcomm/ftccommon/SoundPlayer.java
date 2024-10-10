package com.qualcomm.ftccommon;

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import android.content.Context;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.function.Consumer;

public class SoundPlayer
{
    protected static class InstanceHolder
    {
        public static SoundPlayer theInstance = new SoundPlayer();
    }
    public static SoundPlayer getInstance()
    {
        return InstanceHolder.theInstance;
    }
    public void startPlaying(final Context context, final int resId, final PlaySoundParams params, @Nullable final Consumer<Integer> runWhenStarted, @Nullable final Runnable runWhenFinished)
    {
    }
    public void startPlaying(final Context context, final int resId)
    {
    }
    public boolean preload(Context context, int resourceId)
    {
        return true;
    }


    public static class PlaySoundParams
    {
        /** an additional volume scaling that will be applied to this particular play action */
        public float volume = 1.0f;

        /** whether to wait for any currently-playing non-looping sound to finish before playing */
        public boolean waitForNonLoopingSoundsToFinish = true;

        /** -1 means playing loops forever, 0 is play once, 1 is play twice, etc */
        public int loopControl = 0;

        /** playback rate (1.0 = normal playback, range 0.5 to 2.0) */
        public float rate = 1.0f;

        //--------------

        public PlaySoundParams() { }

        public PlaySoundParams(boolean wait) { this.waitForNonLoopingSoundsToFinish = wait; }

        public PlaySoundParams(PlaySoundParams them)
        {
            this.volume = them.volume;
            this.waitForNonLoopingSoundsToFinish = them.waitForNonLoopingSoundsToFinish;
            this.loopControl = them.loopControl;
            this.rate = them.rate;
        }

        public boolean isLooping()
        {
            return loopControl == -1;
        }
    }

}
