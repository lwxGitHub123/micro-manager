#include "AcqThread.h"

#include "PVCAMAdapter.h"

AcqThread::AcqThread(Universal* camera) : camera_(camera), requestStop_(false), resumeEvent_(true, false)
{
}

AcqThread::~AcqThread()
{
    Stop();
}

void AcqThread::Start()
{
    camera_->LogAdapterMessage("AcqThead starting");
    this->activate();
}

void AcqThread::Stop()
{
    camera_->LogAdapterMessage("AcqThead exit requested");
    requestStop_ = true;
    Resume();
    // Wait for the thread function to exit
    this->wait();
    camera_->LogAdapterMessage("AcqThead exited");
}

void AcqThread::Pause()
{
    // No logging, this is called frequently
    resumeEvent_.Reset();
}

void AcqThread::Resume()
{
    // No logging, this is called frequently
    resumeEvent_.Set();
}

int AcqThread::svc(void)
{
    camera_->LogAdapterMessage("AcqThead loop started");
    int nRet = DEVICE_OK;
    while(!requestStop_)
    {
        resumeEvent_.Wait();
        if (requestStop_)
            break;

        nRet = camera_->acquireFrameSeq();
        if (nRet != DEVICE_OK)
            continue; // Error logged, ignore and try again
        nRet = camera_->waitForFrameSeq();
        if (nRet != DEVICE_OK)
            continue; // Error logged, ignore and try again

        // Frame successfully arrived and ready in the buffer. If we are
        // not using callbacks we need to manually push it to the core.
        if (!camera_->acqCfgCur_.CallbacksEnabled)
            camera_->FrameAcquired();
    }
    camera_->LogAdapterMessage("AcqThead loop exited");
    return nRet;
}
