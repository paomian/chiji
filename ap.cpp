#include <AudioToolbox/AudioToolbox.h>
#include <CoreFoundation/CoreFoundation.h>

#include "ap.hpp"

#include <iostream>

#define checkStatus(status) checkStatus_(status, __FILE__, __LINE__)

void checkStatus_(OSStatus status, const char* file, int line) {
  if(status != noErr) {
    std::cerr << file << ":" << line << ": ";
    char cc[5];
    *((unsigned int*)cc) = status;
    cc[4] = 0;
    std::cerr << "Error status " << status << ": " << cc << std::endl;
  }
}

static const int kNumberBuffers = 3;
struct AQPlayerState {
  AudioStreamBasicDescription   mDataFormat;
  AudioQueueRef                 mQueue;
  AudioQueueBufferRef           mBuffers[kNumberBuffers];
  AudioFileID                   mAudioFile;
  UInt32                        perPacketBufferByteSize;
  SInt64                        mCurrentPacket;
  UInt32                        mNumPacketsToRead;
  AudioStreamPacketDescription  *mPacketDescs;
  bool                          mIsRunning;
};

class AudioPlayerOsx : public AudioPlayer {
public:
  bool load(CFURLRef url);
  virtual bool isPlaying() const;
  virtual double duration() const;
  virtual double progress() const;
  virtual void play();
  virtual void seek(double t);

  ~AudioPlayerOsx();
private:
  double timeBase;
  AQPlayerState aqData;

  void primeBuffer();
  static void deriveBufferSize(AudioStreamBasicDescription &ASBDesc,
                               UInt32                      maxPacketSize,
                               Float64                     seconds,
                               UInt32                      *outBufferSize,
                               UInt32                      *outNumPacketsToRead);
  void seekToPacket(uint64_t packet);
};




static void HandleOutputBuffer(void                *aqData,
                               AudioQueueRef,
                               AudioQueueBufferRef inBuffer)
{
  OSStatus status;
  AQPlayerState *pAqData = (AQPlayerState *) aqData;
  if (pAqData->mIsRunning == 0) return;
  UInt32 numPackets = 5;
  UInt32 s = 20135;
  //  std::cout << "perPacketBufferByteSize:" << pAqData -> perPacketBufferByteSize << std::endl;
  status = AudioFileReadPacketData (pAqData->mAudioFile,
                                    false,
                                    //&pAqData -> perPacketBufferByteSize,
                                    &s,
                                    pAqData->mPacketDescs,
                                    pAqData->mCurrentPacket,
                                    &numPackets,
                                    inBuffer->mAudioData);
  std::cout << "s:" << s << std::endl;
  checkStatus(status);
  if (numPackets > 0) {
    //inBuffer->mAudioDataByteSize = pAqData -> perPacketBufferByteSize;
    inBuffer->mAudioDataByteSize = s;
    status = AudioQueueEnqueueBuffer(pAqData->mQueue,
                                     inBuffer,
                                     (pAqData->mPacketDescs ? numPackets : 0),
                                     pAqData->mPacketDescs);
    checkStatus(status);
    pAqData->mCurrentPacket += numPackets;
  } else {
    status = AudioQueueStop (
                             pAqData->mQueue,
                             false
                             );
    checkStatus(status);
    pAqData->mIsRunning = false;
  }
}

bool AudioPlayerOsx::load(CFURLRef url) {
  memset(&aqData,0,sizeof(aqData));
  timeBase = 0;
  auto status = AudioFileOpenURL(url,kAudioFileReadPermission,0,&aqData.mAudioFile);
  checkStatus(status);
  if( status != noErr ) return false;
  UInt32 dataFormatSize = sizeof (aqData.mDataFormat);

  status = AudioFileGetProperty (aqData.mAudioFile,
                                 kAudioFilePropertyDataFormat,
                                 &dataFormatSize,
                                 &aqData.mDataFormat);
  checkStatus(status);

  status = AudioQueueNewOutput (&aqData.mDataFormat,
                                HandleOutputBuffer,
                                &aqData,
                                CFRunLoopGetCurrent(),
                                kCFRunLoopCommonModes,
                                0,
                                &aqData.mQueue);
  checkStatus(status);

  UInt32 maxPacketSize;
  UInt32 propertySize = sizeof (maxPacketSize);
  status = AudioFileGetProperty (aqData.mAudioFile,
                                 kAudioFilePropertyPacketSizeUpperBound,
                                 &propertySize,
                                 &maxPacketSize);
  checkStatus(status);

  deriveBufferSize (aqData.mDataFormat,
                    maxPacketSize,
                    0.5,
                    &aqData.perPacketBufferByteSize,
                    &aqData.mNumPacketsToRead);

  if (aqData.mDataFormat.mBytesPerPacket == 0 ||
      aqData.mDataFormat.mFramesPerPacket == 0) {
    aqData.mPacketDescs =
      (AudioStreamPacketDescription*)malloc(aqData.mNumPacketsToRead * sizeof (AudioStreamPacketDescription));
  } else {
    aqData.mPacketDescs = NULL;
  }


  UInt32 cookieSize = sizeof (UInt32);
  OSStatus couldNotGetProperty =
    AudioFileGetPropertyInfo (aqData.mAudioFile,
                              kAudioFilePropertyMagicCookieData,
                              &cookieSize,
                              nullptr);
  checkStatus(couldNotGetProperty);

  if (!couldNotGetProperty && cookieSize) {
    char* magicCookie =
      (char *) malloc (cookieSize);

    status = AudioFileGetProperty (aqData.mAudioFile,
                                   kAudioFilePropertyMagicCookieData,
                                   &cookieSize,
                                   magicCookie);
    checkStatus(status);

    status = AudioQueueSetProperty (aqData.mQueue,
                                    kAudioQueueProperty_MagicCookie,
                                    magicCookie,
                                    cookieSize);
    checkStatus(status);

    free (magicCookie);
  }

  return true;
}

void AudioPlayerOsx::primeBuffer() {
  OSStatus status;
  for (int i = 0; i < kNumberBuffers; ++i) {
    status = AudioQueueAllocateBuffer(aqData.mQueue,
                                      aqData.perPacketBufferByteSize,
                                      &aqData.mBuffers[i]);
    checkStatus(status);
#if 1
    HandleOutputBuffer (&aqData,
                        aqData.mQueue,
                        aqData.mBuffers[i]);
#endif
  }
  /*
    status = AudioQueuePrime (aqData.mQueue,
    kNumberBuffers,
    nullptr);
    checkStatus(status);
  */
}

void AudioPlayerOsx::play() {
  aqData.mIsRunning = true;
  aqData.mCurrentPacket = 0;

  primeBuffer();

  Float32 gain = 1.0;
  // Optionally, allow user to override gain setting here
  auto status = AudioQueueSetParameter (aqData.mQueue,
                                        kAudioQueueParam_Volume,
                                        gain);
  checkStatus(status);

  status = AudioQueueStart(aqData.mQueue,
                           nullptr);
  checkStatus(status);
}

double AudioPlayerOsx::duration() const {
  double dur = 0;
  unsigned int sz = sizeof(dur);
  OSStatus status = AudioFileGetProperty(aqData.mAudioFile, kAudioFilePropertyEstimatedDuration, (UInt32*)&sz, &dur);
  checkStatus(status);
  return dur;
}

void AudioPlayerOsx::seekToPacket(uint64_t packet) {
  AudioQueueStop(aqData.mQueue, true);
  aqData.mCurrentPacket = rand()%1000;
  primeBuffer();
  AudioQueueStart(aqData.mQueue, NULL);

}

void AudioPlayerOsx::seek(double sec) {
  double frame = sec * aqData.mDataFormat.mSampleRate;

  AudioFramePacketTranslation trans;
  trans.mFrame = frame;

  unsigned int sz = sizeof(trans);
  OSStatus status = AudioFileGetProperty(aqData.mAudioFile, kAudioFilePropertyFrameToPacket, (UInt32*)&sz, &trans);

  seekToPacket(trans.mPacket);
  trans.mFrameOffsetInPacket = 0; // Don't support sub packet seeking..

  status = AudioFileGetProperty(aqData.mAudioFile, kAudioFilePropertyPacketToFrame, (UInt32*)&sz, &trans);

  timeBase = trans.mFrame / aqData.mDataFormat.mSampleRate;

}

double AudioPlayerOsx::progress() const {
  double p = 0;

  AudioTimeStamp timeStamp;
  AudioQueueGetCurrentTime (aqData.mQueue,
                            NULL,
                            &timeStamp,
                            NULL);

  //        checkStatus(status);

  p = timeStamp.mSampleTime/aqData.mDataFormat.mSampleRate + timeBase;

  return p;
}

AudioPlayerOsx::~AudioPlayerOsx() {

  OSStatus status;

  status = AudioQueueDispose (
                              aqData.mQueue,
                              true
                              );
  checkStatus(status);


  status = AudioFileClose(aqData.mAudioFile);
  checkStatus(status);

  free (aqData.mPacketDescs);


}

void AudioPlayerOsx::deriveBufferSize (
                                       AudioStreamBasicDescription &ASBDesc,
                                       UInt32                      maxPacketSize,
                                       Float64                     seconds,
                                       UInt32                      *outBufferSize,
                                       UInt32                      *outNumPacketsToRead
                                       ) {
  static const int maxBufferSize = 0x50000;
  static const int minBufferSize = 0x4000;

  if (ASBDesc.mFramesPerPacket != 0) {
    Float64 numPacketsForTime =
      ASBDesc.mSampleRate / ASBDesc.mFramesPerPacket * seconds;
    *outBufferSize = numPacketsForTime * maxPacketSize;
  } else {
    *outBufferSize =
      maxBufferSize > maxPacketSize ?
      maxBufferSize : maxPacketSize;
  }

  if (
      *outBufferSize > maxBufferSize &&
      *outBufferSize > maxPacketSize
      )
    *outBufferSize = maxBufferSize;
  else {
    if (*outBufferSize < minBufferSize)
      *outBufferSize = minBufferSize;
  }

  *outNumPacketsToRead = *outBufferSize / maxPacketSize;
}

bool AudioPlayerOsx::isPlaying() const { return aqData.mIsRunning; }

AudioPlayer* AudioPlayer::file(const char *fn) {

  CFURLRef url = CFURLCreateFromFileSystemRepresentation(NULL, (UInt8*)fn, strlen(fn), false);

  AudioPlayerOsx* ap = new AudioPlayerOsx;

  if(!ap->load(url)) {
    delete ap;
    ap = NULL;
  }

  CFRelease(url);

  return ap;
}
