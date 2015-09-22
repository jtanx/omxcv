.global omxcv_bgr2rgb_neon

omxcv_bgr2rgb_neon:
    loop:
        vld3.8 {d0-d2}, [r0]!
        vswp d0, d2
        vst3.8 {d0-d2}, [r1]!
        subs r2, r2, #1
        bgt loop
        bx lr
