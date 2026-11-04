#ifndef TRI_NATIVE_PANEL_H
#define TRI_NATIVE_PANEL_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* One context per helper process; same owner thread for every API.
 * X11 depth24, BGRX32 little endian, MIT-SHM and SHAPE>=1.1 required.
 * parent=0: root override_redirect overlay; otherwise child of parent XID.
 * x/y relative to parent; target 1..960 x 1..540 must fit current parent.
 * Starts hidden; empty ShapeInput means pointer/touch hit testing passes through.
 * Draw/rect/visible/close calls are synchronous; do not call concurrently.
 */
void *tri_panel_open(unsigned long parent,int x,int y,int width,int height);
/* Hides before moving/resizing; the next valid draw remaps it. */
int tri_panel_rect(void *,int x,int y,int width,int height);
int tri_panel_visible(void *,int visible);
/* Original source always960x540. format0=GRAY8_RGBPATCH8_LE,1=BGR24.
 * Crop is [x0,y0,x1,y1], exclusive upper bounds. Aspect-fit, centred black
 * letterbox, nearest pixel-centre resampling. No brightness/color transform.
 * Gray patches: x:u16LE,y:u16LE,R,G,B,reserved0; strictly increasing y*960+x.
 * ms[0]=decode/resize/map wallms; ms[1]=XShm submission to matching completion.
 * Success0 proves X-server consumption only, not physical screen scanout.
 * Errors -1..-3 are rejected input; -4..-9 poison the display context.
 * Caller must bind metadata/run/view/identity and reject stale/repeated frames.
 */
int tri_panel_draw(void *,const unsigned char *,size_t,int format,int patches,
                   int x0,int y0,int x1,int y1,double ms[2]);
/* Readback for diagnostic verification only; no inclusion in throughput FPS.
 * count=target width*height; output top-down 0x00RRGGBB words. */
int tri_panel_readback(void *,uint32_t *,size_t count);
/* Returns0 for verified empty input region; diagnostic (not every frame). */
int tri_panel_input_rectangles(void *);
/* Always releases this local display context.0=clean/no recorded X error.
 * X-server loss may terminate this helper through standard Xlib I/O handling;
 * service must observe child exit and report display unavailable. */
int tri_panel_close(void *);
#ifdef __cplusplus
}
#endif
#endif
