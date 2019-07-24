package com.intel.realsense.librealsense;

public class DepthSmearInvalidationFilter extends Filter {

    public DepthSmearInvalidationFilter(){
        mHandle = nCreate(mQueue.getHandle());
    }

    private static native long nCreate(long queueHandle);
}