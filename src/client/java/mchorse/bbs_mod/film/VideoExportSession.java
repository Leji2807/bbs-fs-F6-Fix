package mchorse.bbs_mod.film;

import mchorse.bbs_mod.BBSModClient;
import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.audio.MinecraftSoundCapture;
import mchorse.bbs_mod.audio.MinecraftSoundMixer;
import mchorse.bbs_mod.audio.Wave;
import mchorse.bbs_mod.audio.wav.WaveReader;
import mchorse.bbs_mod.client.BBSRendering;
import mchorse.bbs_mod.utils.StringUtils;
import mchorse.bbs_mod.utils.VideoMuxer;
import mchorse.bbs_mod.utils.VideoRecorder;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;

/**
 * Owns the lifecycle of a single video export with added [DEBUG-EXPORT] logs
 * and stack traces to analyze premature session termination during F6 Replay Export.
 */
public abstract class VideoExportSession
{
    public enum State
    {
        IDLE,
        WARMUP,
        RECORDING
    }

    protected State state = State.IDLE;
    protected long warmupEndsAtMs;

    protected File audioFile;
    protected int textureId;
    protected int width;
    protected int height;

    private String movieName;
    private File deferredAudioFile;
    private double recordingFrameRate;
    private long recordingStartedAtMs;

    private FinishedListener finishedListener;

    protected VideoRecorder getRecorder()
    {
        return BBSModClient.getVideoRecorder();
    }

    public boolean isExporting()
    {
        return this.state != State.IDLE;
    }

    public boolean isWarmingUp()
    {
        return this.state == State.WARMUP;
    }

    public boolean isRecording()
    {
        return this.getRecorder().isRecording();
    }

    public long getWarmupRemainingMs()
    {
        if (this.state != State.WARMUP)
        {
            return 0L;
        }

        return Math.max(0L, this.warmupEndsAtMs - System.currentTimeMillis());
    }

    public void setFinishedListener(FinishedListener listener)
    {
        this.finishedListener = listener;
    }

    protected final boolean begin(int textureId, int width, int height, long delayMs)
    {
        System.out.println("[DEBUG-EXPORT] begin() called with textureId=" + textureId + ", w=" + width + ", h=" + height + ", delayMs=" + delayMs);

        if (this.isExporting() || this.getRecorder().isRecording())
        {
            System.out.println("[DEBUG-EXPORT] Aborted begin(): Session is already exporting or VideoRecorder is already recording!");
            return false;
        }

        this.textureId = textureId;
        this.width = width;
        this.height = height;
        this.audioFile = null;

        boolean prepSuccess = this.prepare();
        System.out.println("[DEBUG-EXPORT] prepare() returned: " + prepSuccess);

        if (!prepSuccess)
        {
            System.out.println("[DEBUG-EXPORT] prepare() failed, resetting session.");
            this.reset();
            return false;
        }

        this.applyExportTarget();

        if (delayMs > 0L)
        {
            System.out.println("[DEBUG-EXPORT] Entering WARMUP state for " + delayMs + " ms.");
            this.state = State.WARMUP;
            this.warmupEndsAtMs = System.currentTimeMillis() + delayMs;
            this.onWarmupStarted();
        }
        else
        {
            System.out.println("[DEBUG-EXPORT] Proceeding directly to beginRecording().");
            this.beginRecording();
        }

        return this.isExporting();
    }

    public final void update()
    {
        if (this.state == State.WARMUP)
        {
            // MODIFIKASI: Mengabaikan respon dari shouldAbortWarmup()
            if (this.shouldAbortWarmup())
            {
                System.out.println("[DEBUG-EXPORT] [FIX] Warmup wanted to abort, but we are IGNORING it to force export!");
                // this.cancel(); // <-- Baris ini dimatikan
                // return;        // <-- Baris ini dimatikan
            }

            if (!this.isWarmupReady() || System.currentTimeMillis() < this.warmupEndsAtMs)
            {
                return;
            }

            System.out.println("[DEBUG-EXPORT] Warmup finished, starting recording.");
            this.beginRecording();
        }
        else if (this.state == State.RECORDING)
        {
            boolean finished = this.isFinished();
            int counter = this.getRecorder().getCounter();

            if (finished)
            {
                System.out.println("[DEBUG-EXPORT] isFinished() returned TRUE at recorded frame count: " + counter);
                this.stop();
            }
        }
    }

    private void beginRecording()
    {
        System.out.println("[DEBUG-EXPORT] beginRecording() executing...");
        VideoRecorder recorder = this.getRecorder();

        this.state = State.RECORDING;

        String movieName = this.getMovieName();

        if (movieName == null || movieName.isEmpty())
        {
            movieName = StringUtils.createTimestampFilename();
        }

        File muxAudioFile = this.audioFile;
        boolean captureSounds = BBSSettings.videoExportMinecraftSounds.get();

        if (captureSounds)
        {
            this.deferredAudioFile = this.audioFile;
            muxAudioFile = null;
        }

        try
        {
            System.out.println("[DEBUG-EXPORT] Calling VideoRecorder.startRecording() for movie: " + movieName);
            recorder.startRecording(movieName, muxAudioFile, this.textureId, this.width, this.height);
        }
        catch (Exception e)
        {
            System.out.println("[DEBUG-EXPORT] Exception during VideoRecorder.startRecording():");
            e.printStackTrace();
            this.cancel();
            return;
        }

        if (!recorder.isRecording())
        {
            System.out.println("[DEBUG-EXPORT] VideoRecorder.isRecording() is FALSE right after startRecording() call!");
            this.cancel();
            return;
        }

        this.movieName = movieName;
        this.recordingStartedAtMs = System.currentTimeMillis();

        if (captureSounds)
        {
            this.recordingFrameRate = BBSRendering.getVideoFrameRate();
            BBSModClient.getMinecraftSoundCapture().begin();
        }

        System.out.println("[DEBUG-EXPORT] Recording successfully started! FPS=" + BBSRendering.getVideoFrameRate());
        this.onRecordingStarted();
    }

    public final void stop()
    {
        System.out.println("[DEBUG-EXPORT] stop() called explicitly.");
        this.finish(false);
    }

    public final void cancel()
    {
        System.out.println("[DEBUG-EXPORT] cancel() called explicitly.");
        this.finish(true);
    }

    private void finish(boolean cancelled)
    {
        if (this.state == State.IDLE)
        {
            return;
        }

        System.out.println("[DEBUG-EXPORT] finish() triggered! cancelled=" + cancelled + ", currentState=" + this.state);

        // Print StackTrace to pinpoint EXACTLY what line of code or event triggered finish/cancel/stop
        new Exception("[DEBUG-EXPORT STACKTRACE] Session finished caller trace:").printStackTrace();

        VideoRecorder recorder = this.getRecorder();
        int recordedFrames = recorder.getCounter();
        MinecraftSoundCapture capture = BBSModClient.getMinecraftSoundCapture();
        boolean postPass = capture.isActive();

        if (recorder.isRecording())
        {
            try
            {
                recorder.stopRecording(!postPass);
            }
            catch (Exception e)
            {
                System.out.println("[DEBUG-EXPORT] Exception in recorder.stopRecording():");
                e.printStackTrace();
            }
        }

        if (postPass)
        {
            this.finishCapturedSounds(capture, recordedFrames);
            recorder.playFinishEffects();
        }
        else if (this.deferredAudioFile != null)
        {
            this.deferredAudioFile.delete();
        }

        this.state = State.IDLE;
        this.teardown(cancelled);
        this.reset();

        FinishedListener listener = this.finishedListener;
        this.finishedListener = null;

        if (listener != null)
        {
            listener.onFinished(cancelled);
        }
    }

    private void finishCapturedSounds(MinecraftSoundCapture capture, int recordedFrames)
    {
        capture.end();

        File deferred = this.deferredAudioFile;
        this.deferredAudioFile = null;

        try
        {
            if (recordedFrames <= 0)
            {
                if (deferred != null)
                {
                    deferred.delete();
                }
                return;
            }

            File folder = BBSRendering.getVideoFolder();
            File audio = new File(folder, this.movieName + ".wav");

            if (!MinecraftSoundMixer.mixToFile(audio, capture.getSounds(), capture.getFrames(), readWave(deferred), 48000, this.recordingFrameRate, recordedFrames))
            {
                System.out.println("[DEBUG-EXPORT] Sound mixer failed.");
                return;
            }

            File video = this.findRecordedVideo(folder);

            if (video != null && VideoMuxer.mux(video, audio, this.movieName) != null && deferred != null)
            {
                deferred.delete();
            }
        }
        catch (Throwable e)
        {
            e.printStackTrace();
        }
    }

    private static Wave readWave(File file)
    {
        if (file == null || !file.isFile())
        {
            return null;
        }

        try (InputStream stream = new FileInputStream(file))
        {
            return new WaveReader().read(stream);
        }
        catch (Exception e)
        {
            e.printStackTrace();
            return null;
        }
    }

    private File findRecordedVideo(File folder)
    {
        File[] files = folder.listFiles();

        if (files == null)
        {
            return null;
        }

        String prefix = this.movieName + ".";
        long notBefore = this.recordingStartedAtMs - 10_000L;
        File found = null;

        for (File file : files)
        {
            if (!file.isFile() || !file.getName().startsWith(prefix))
            {
                continue;
            }

            if (isExportArtifact(file.getName().substring(prefix.length())))
            {
                continue;
            }

            if (file.lastModified() < notBefore)
            {
                continue;
            }

            if (found == null || file.lastModified() > found.lastModified())
            {
                found = file;
            }
        }

        return found;
    }

    protected static boolean isExportArtifact(String rest)
    {
        rest = rest.toLowerCase();
        return rest.equals("wav") || rest.equals("log") || rest.endsWith(".log") || rest.startsWith("tmp.");
    }

    private void reset()
    {
        this.state = State.IDLE;
        this.warmupEndsAtMs = 0L;
        this.audioFile = null;
        this.textureId = 0;
        this.width = 0;
        this.height = 0;
        this.movieName = null;
        this.deferredAudioFile = null;
        this.recordingFrameRate = 0D;
        this.recordingStartedAtMs = 0L;
    }

    /* Hooks */

    protected String getMovieName()
    {
        return StringUtils.createTimestampFilename();
    }

    protected abstract boolean prepare();

    protected void applyExportTarget()
    {}

    protected void onWarmupStarted()
    {}

    protected boolean shouldAbortWarmup()
    {
        return false;
    }

    protected boolean isWarmupReady()
    {
        return true;
    }

    protected abstract void onRecordingStarted();

    protected abstract boolean isFinished();

    protected abstract void teardown(boolean cancelled);

    @FunctionalInterface
    public interface FinishedListener
    {
        void onFinished(boolean cancelled);
    }
}
