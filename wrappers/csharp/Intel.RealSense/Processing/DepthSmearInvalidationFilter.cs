﻿// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

namespace Intel.RealSense
{
    using System;

    public class DepthSmearInvalidationFilter : ProcessingBlock
    {
        private static IntPtr Create()
        {
            object error;
            return NativeMethods.rs2_create_zero_order_invalidation_block(out error);
        }

        internal DepthSmearInvalidationFilter(IntPtr ptr)
            : base(ptr)
        {
        }

        public DepthSmearInvalidationFilter()
            : base(Create())
        {
        }

        [Obsolete("This method is obsolete. Use Process method instead")]
        public VideoFrame ApplyFilter(Frame original, FramesReleaser releaser)
        {
            return Process(original).DisposeWith(releaser) as VideoFrame;
        }
    }
}