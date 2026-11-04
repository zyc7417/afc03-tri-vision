/* Local, input-transparent X11 panel. All Xlib calls belong to one caller.
 * Matching XShmCompletion proves server consumption, never physical scanout.
 * Derived from display.c; no device, camera, inference or production changes.
 * Linux build: cc -O3 -std=c11 -Wall -Wextra -fPIC -shared display_panel.c
 *             -o display_panel.so $(pkg-config --cflags --libs x11 xext)
 */
#define _POSIX_C_SOURCE 200809L
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <X11/extensions/shape.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <poll.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define W 960
#define H 540
#define N (W*H)
typedef struct {
 Display *d; Window win,parent; GC gc; XImage *im; XShmSegmentInfo sh;
 int attached,complete,err,x,y,w,h,visible,handler_installed;
 uint32_t *scratch,*target; unsigned long request;
 int (*previous_handler)(Display*,XErrorEvent*);
} Context;
static int x_error=0;
static Context *active=NULL; /* one display owner per helper process */
static int error_handler(Display*d,XErrorEvent*e){(void)d;x_error=e->error_code;return 0;}
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static int geometry(int x,int y,int w,int h){return x>=0&&y>=0&&x<=16384&&y<=16384&&w>=1&&h>=1&&w<=W&&h<=H;}

int tri_panel_close(void *v){
 Context*c=v;if(!c)return 0;int result=c->err;
 if(c->d){
  if(c->attached){XShmDetach(c->d,&c->sh);XSync(c->d,False);}
  if(c->im){c->im->data=NULL;XDestroyImage(c->im);}
  if(c->gc)XFreeGC(c->d,c->gc);
  if(c->win)XDestroyWindow(c->d,c->win);
  XCloseDisplay(c->d);if(x_error)result=-1;
  if(c->handler_installed)XSetErrorHandler(c->previous_handler);
 }
 if(c->sh.shmaddr&&c->sh.shmaddr!=(char*)-1)shmdt(c->sh.shmaddr);
 if(c->sh.shmid>=0)shmctl(c->sh.shmid,IPC_RMID,NULL);
 free(c->scratch);free(c->target);if(active==c)active=NULL;free(c);return result;
}

void *tri_panel_open(unsigned long parent,int x,int y,int w,int h){
 if(active||!geometry(x,y,w,h))return NULL;
 Context*c=calloc(1,sizeof(*c));if(!c)return NULL;c->sh.shmid=-1;
 c->d=XOpenDisplay(NULL);if(!c->d)goto fail;
 c->previous_handler=XSetErrorHandler(error_handler);c->handler_installed=1;x_error=0;
 int screen=DefaultScreen(c->d),shape_event,shape_error,major=0,minor=0;
 c->parent=parent?parent:RootWindow(c->d,screen);
 XWindowAttributes pa;
 if(!XGetWindowAttributes(c->d,c->parent,&pa)||pa.class!=InputOutput||pa.map_state!=IsViewable)goto fail;
 int depth=pa.depth;Visual*visual=pa.visual;
 if(depth!=24||visual->red_mask!=0xff0000||visual->green_mask!=0xff00||visual->blue_mask!=0xff||!XShmQueryExtension(c->d))goto fail;
 if(!XShapeQueryExtension(c->d,&shape_event,&shape_error)||!XShapeQueryVersion(c->d,&major,&minor)||major<1||(major==1&&minor<1))goto fail;
 if(x+w>pa.width||y+h>pa.height)goto fail;
 XSetWindowAttributes a={0};a.override_redirect=parent?False:True;a.background_pixel=0;a.event_mask=StructureNotifyMask;
 c->win=XCreateWindow(c->d,c->parent,x,y,(unsigned)w,(unsigned)h,0,depth,InputOutput,visual,CWOverrideRedirect|CWBackPixel|CWEventMask,&a);
 XStoreName(c->d,c->win,"AFC03 native fresh-frame panel");
 /* Empty ShapeInput makes the panel ignore pointer/touch hit testing. */
 XShapeCombineRectangles(c->d,c->win,ShapeInput,0,0,NULL,0,ShapeSet,Unsorted);
 c->gc=XCreateGC(c->d,c->win,0,NULL);
 c->im=XShmCreateImage(c->d,visual,depth,ZPixmap,NULL,&c->sh,W,H);
 if(!c->im||c->im->bits_per_pixel!=32||c->im->byte_order!=LSBFirst||c->im->bytes_per_line<W*4)goto fail;
 c->sh.shmid=shmget(IPC_PRIVATE,(size_t)c->im->bytes_per_line*H,IPC_CREAT|0600);if(c->sh.shmid<0)goto fail;
 c->sh.shmaddr=shmat(c->sh.shmid,NULL,0);if(c->sh.shmaddr==(char*)-1)goto fail;
 c->im->data=c->sh.shmaddr;c->sh.readOnly=True;
 if(!XShmAttach(c->d,&c->sh))goto fail;
 XSync(c->d,False);if(x_error)goto fail;c->attached=1;
 if(shmctl(c->sh.shmid,IPC_RMID,NULL))goto fail;
 c->sh.shmid=-1;
 c->scratch=malloc(N*sizeof(uint32_t));c->target=calloc(N,sizeof(uint32_t));if(!c->scratch||!c->target)goto fail;
 c->complete=XShmGetEventBase(c->d)+ShmCompletion;c->x=x;c->y=y;c->w=w;c->h=h;
 /* Start hidden. Only a validated current frame may make it visible. */
 XSync(c->d,False);if(x_error)goto fail;active=c;return c;
fail:tri_panel_close(c);return NULL;
}

int tri_panel_rect(void *v,int x,int y,int w,int h){
 Context*c=v;if(!c||c->err||!geometry(x,y,w,h))return -1;
 if(c->x==x&&c->y==y&&c->w==w&&c->h==h)return 0;
 XWindowAttributes pa;if(!XGetWindowAttributes(c->d,c->parent,&pa)||x+w>pa.width||y+h>pa.height||pa.map_state!=IsViewable){c->err=1;return -2;}
 /* Hide first: a resize must not expose pixels of a previous view. */
 if(c->visible){XUnmapWindow(c->d,c->win);c->visible=0;}
 XMoveResizeWindow(c->d,c->win,x,y,(unsigned)w,(unsigned)h);XSync(c->d,False);
 if(x_error){c->err=1;return -3;}
 c->x=x;c->y=y;c->w=w;c->h=h;return 0;
}

int tri_panel_visible(void *v,int show){
 Context*c=v;if(!c||c->err||(show!=0&&show!=1))return -1;
 if(show==c->visible)return 0;
 if(show)XMapRaised(c->d,c->win);else XUnmapWindow(c->d,c->win);
 XSync(c->d,False);if(x_error){c->err=1;return -2;}c->visible=show;return 0;
}

int tri_panel_draw(void*v,const unsigned char*p,size_t length,int format,int patches,
                   int x0,int y0,int x1,int y1,double*ms){
 Context*c=v;if(!c||!p||!ms||c->err)return -1;
 if(x0<0||y0<0||x0>=x1||y0>=y1||x1>W||y1>H)return -2;
 double start=now();
 if(format==0){
  if(patches<0||patches>32768||length!=(size_t)N+(size_t)patches*8)return -2;
  int last=-1;
  for(int i=0;i<patches;i++){
   const unsigned char*q=p+N+i*8;int x=q[0]+256*q[1],y=q[2]+256*q[3],index=y*W+x;
   if(x>=W||y>=H||q[7]||index<=last)return -3;
   last=index;
  }
 }else if(format==1){
  if(patches||length!=(size_t)N*3)return -2;
 }else return -2;
 int cw=x1-x0,ch=y1-y0,tw=c->w,th=c->h;
 if((int64_t)tw*ch>(int64_t)th*cw)tw=(int)((int64_t)th*cw/ch);
 else th=(int)((int64_t)tw*ch/cw);
 if(tw<1)tw=1;
 if(th<1)th=1;
 int ox=(c->w-tw)/2,oy=(c->h-th)/2;
 int xmap[W],firstx[W],lastx[W],firsty[H],lasty[H];
 for(int x=0;x<W;x++)firstx[x]=lastx[x]=-1;
 for(int y=0;y<H;y++)firsty[y]=lasty[y]=-1;
 for(int x=0;x<tw;x++){
  int sx=x0+(int)(((int64_t)(2*x+1)*cw)/(2*tw));xmap[x]=sx;
  if(firstx[sx]<0)firstx[sx]=x+ox;
  lastx[sx]=x+ox;
 }
 for(int y=0;y<c->h;y++)memset(c->im->data+y*c->im->bytes_per_line,0,(size_t)c->w*4);
 for(int y=0;y<th;y++){
  int sy=y0+(int)(((int64_t)(2*y+1)*ch)/(2*th));
  if(firsty[sy]<0)firsty[sy]=y+oy;
  lasty[sy]=y+oy;
  uint32_t*dst=(uint32_t*)(c->im->data+(y+oy)*c->im->bytes_per_line)+ox;
  if(format==0){
   const unsigned char*src=p+sy*W;
   for(int x=0;x<tw;x++)dst[x]=(uint32_t)src[xmap[x]]*0x010101;
  }else{
   const unsigned char*src=p+sy*W*3;
   for(int x=0;x<tw;x++){const unsigned char*q=src+xmap[x]*3;dst[x]=q[0]|((uint32_t)q[1]<<8)|((uint32_t)q[2]<<16);}
  }
 }
 if(format==0)for(int i=0;i<patches;i++){
  const unsigned char*q=p+N+i*8;int sx=q[0]+256*q[1],sy=q[2]+256*q[3];
  if(firstx[sx]<0||firsty[sy]<0)continue;
  uint32_t color=((uint32_t)q[4]<<16)|((uint32_t)q[5]<<8)|q[6];
  for(int y=firsty[sy];y<=lasty[sy];y++){
   uint32_t*dst=(uint32_t*)(c->im->data+y*c->im->bytes_per_line);
   for(int x=firstx[sx];x<=lastx[sx];x++)dst[x]=color;
  }
 }
 /* Map only after the new pixels exist. Client must gate this by current run. */
 if(!c->visible&&tri_panel_visible(c,1))return -4;
 double packed=now();c->request=NextRequest(c->d);
 if(!XShmPutImage(c->d,c->win,c->gc,c->im,0,0,0,0,(unsigned)c->w,(unsigned)c->h,True)){c->err=1;return -4;}
 XFlush(c->d);double deadline=now()+.5;
 for(;;){
  while(XPending(c->d)){
   XEvent e;XNextEvent(c->d,&e);
   if(e.type==DestroyNotify&&e.xdestroywindow.window==c->win){c->err=1;return -9;}
   if(e.type==c->complete){
    XShmCompletionEvent*s=(XShmCompletionEvent*)&e;
    if(s->drawable!=c->win||s->shmseg!=c->sh.shmseg||s->offset!=0||s->serial!=c->request||s->send_event){c->err=1;return -5;}
    ms[0]=(packed-start)*1000;ms[1]=(now()-packed)*1000;return 0;
   }
  }
  if(x_error){c->err=1;return -6;}
  double remain=deadline-now();if(remain<=0){c->err=1;return -7;}
  struct pollfd fd={ConnectionNumber(c->d),POLLIN,0};int r=poll(&fd,1,(int)(remain*1000)+1);
  if(r<0&&errno!=EINTR){c->err=1;return -8;}
  if(r>0&&(fd.revents&(POLLERR|POLLHUP|POLLNVAL))){c->err=1;return -8;}
 }
}

/* Diagnostic readback outside measured throughput; target rectangle only. */
int tri_panel_readback(void*v,uint32_t*out,size_t count){
 Context*c=v;if(!c||c->err||!out||!c->visible||count!=(size_t)c->w*c->h)return -1;
 XImage*r=XGetImage(c->d,c->win,0,0,(unsigned)c->w,(unsigned)c->h,AllPlanes,ZPixmap);if(!r)return -2;
 for(int y=0;y<c->h;y++)for(int x=0;x<c->w;x++)out[y*c->w+x]=XGetPixel(r,x,y)&0xffffff;
 XDestroyImage(r);return 0;
}
int tri_panel_input_rectangles(void*v){
 Context*c=v;if(!c||c->err)return -1;int count=-1,ordering=0;
 XRectangle*r=XShapeGetRectangles(c->d,c->win,ShapeInput,&count,&ordering);if(r)XFree(r);
 XSync(c->d,False);if(x_error){c->err=1;return -2;}return count;
}
