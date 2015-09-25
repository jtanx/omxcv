.global omxcv_bgr2rgb_neon

@void omxcv_bgr2rgb_neon32(const uint8_t *src, uint8_t *dst, int pixels)
omxcv_bgr2rgb_neon32:
    mov r2, r2, lsr #5              @Width/32 (Number of runs needed)
    vpush {q4-q5}                   @q4-q11 must be saved by us
    loop3:
        @pld [r0, #512]             @Preload for reading
        vld3.8 {d0-d2}, [r0]!       @Read in 8 pixels
        vld3.8 {d3-d5}, [r0]!       @Read in 8 more pixels
        vld3.8 {d6-d8}, [r0]!       @Read in 8 more pixels
        vld3.8 {d9-d11}, [r0]!      @Read in 8 more pixels
        vswp d0, d2                 @Swap R/B
        vswp d3, d5                 @Swap R/B
        vswp d6, d8
        vswp d9, d11
        subs r2, r2, #1             @Decrement counter
        vst3.8 {d0-d2}, [r1]!       @Store 8 pixels
        vst3.8 {d3-d5}, [r1]!       @Store 8 more
        vst3.8 {d6-d8}, [r1]!       @Store 8 more pixels
        vst3.8 {d9-d11}, [r1]!      @Store 8 more pixels
        bgt loop3                   @Loop
    vpop {q4-q5}                    @Restore q4-q5
    bx lr                           @Return

@void omxcv_bgr2rgb_neon(const uint8_t *src, uint8_t *dst, int pixels)
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

@void omxcv_bgr2rgb_neon1(const uint8_t *src, uint8_t *dst, int pixels)
omxcv_bgr2rgb_neon1:
    mov r2, r2, lsr #3              @Width/8 (Number of runs needed)
    loop:
        pld [r0, #192]              @Preload for reading
        vld3.8 {d0-d2}, [r0]!       @Read in 8 pixels
        vswp d0, d2                 @Swap R/B
        subs r2, r2, #1             @Decrement counter
        vst3.8 {d0-d2}, [r1]!       @Store 8 pixels
        bgt loop2                   @Loop
    bx lr                           @Return
