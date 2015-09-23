.global omxcv_bgr2rgb_neon

omxcv_bgr2rgb_neon:
    mov r2, r2, lsr #4              @Width/16 (Number of runs needed)
    loop2:
        pld [r0, #384]              @Preload for reading
        vld3.8 {d0-d2}, [r0]!       @Read in 8 pixels
        vld3.8 {d3-d5}, [r0]!       @Read in 8 more pixels
        vswp d0, d2                 @Swap R/B
        vswp d3, d5                 @Swap R/B
        subs r2, r2, #1             @Decrement counter
        vst3.8 {d0-d2}, [r1]!       @Store 8 pixels
        vst3.8 {d3-d5}, [r1]!       @Store 8 more pixels
        bgt loop2                   @Loop
    bx lr                           @Return
