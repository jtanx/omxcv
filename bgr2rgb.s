.global omxcv_bgr2rgb

omxcv_bgr2rgb:
    @Call void omxcv_bgr2rgb(uint32_t *in, uint32_t size)
    ldr r2, [r0],#0 @Load the data in
    rev r3, r2      @Swap the byte order
    str r3, [r0],#4 @Store the swapped data and increment counter
    subs r1, r1,#1  @Decrement byte counter
    bne omxcv_bgr2rgb @Continue until byte counter is zero
    bx lr
