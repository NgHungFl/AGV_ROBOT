#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "main.h"
#include "line_sensor.c"
#include "agv_motion.h"
#include "agv_mission.h"
#include "encoder.h"
#include "wheel_control.h"
#include "line_tracking.h"
#include "tcs34725.h"
#include "bsp_uart.h"
#include "motor.h"
#include "load_detect.h"

/* ---------- stubs ---------- */
static uint32_t sim_ms = 0;
uint32_t HAL_GetTick(void){ return sim_ms; }
TIM_TypeDef t1,t4; TIM_HandleTypeDef htim1={&t1}, htim4={&t4};
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*h,uint32_t c){return HAL_OK;}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*h){return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*h,uint32_t*b,uint32_t n){return HAL_OK;}
static int verbose=0;
HAL_StatusTypeDef BSP_UART_Printf(const char*f,...){ if(verbose>=1){va_list a;va_start(a,f);vprintf(f,a);va_end(a);} return HAL_OK;}

static float duty[2]; static int brake[2];
Motor_Status_t Motor_SetDuty(Motor_ID_t m,int16_t d){duty[m]=d*100.0f/3599.0f;brake[m]=0;return MOTOR_OK;}
Motor_Status_t Motor_SetDutyPercent(Motor_ID_t m,float p){if(p>100)p=100;if(p<-100)p=-100;duty[m]=p;brake[m]=0;return MOTOR_OK;}
void Motor_Stop(Motor_ID_t m){duty[m]=0;brake[m]=0;}
void Motor_StopAll(void){Motor_Stop(0);Motor_Stop(1);}
void Motor_Brake(Motor_ID_t m){brake[m]=1;duty[m]=0;}
void Motor_BrakeAll(void){Motor_Brake(0);Motor_Brake(1);}

/* ---------- sim parameters ---------- */
static double P_Kl=MOTOR_L_GAIN_FWD, P_Kr=MOTOR_R_GAIN_FWD;
static double P_DBl=MOTOR_L_DB_FWD, P_DBr=MOTOR_R_DB_FWD;
static double P_L=ROBOT_SENSOR_DIST_MM, P_W=ROBOT_TRACK_WIDTH_MM, P_wheelScale=1.0;
static double P_noise=1.0, P_loadTau=1.4, P_loadK=0.92;
static int P_branchBlue=0;
static int heavy=0; static int P_dark=0;
static int block_present=0, block_blue=0;

static double X,Y,PSI,VL,VR,accL,accR;

TCS34725_ColorData_t dummy;
HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef*h){return HAL_OK;}
HAL_StatusTypeDef TCS34725_ReadColor(TCS34725_ColorData_t*c){
  if(block_present&&P_dark){c->clear=150+rand()%10; if(!block_blue){c->red=70;c->green=25;c->blue=20;}else{c->red=20;c->green=30;c->blue=75;}}
  else if(block_present){c->clear=2000+rand()%50; if(!block_blue){c->red=900;c->green=250;c->blue=200;}else{c->red=200;c->green=300;c->blue=900;}}
  else {c->clear=350+rand()%20;c->red=110;c->green=120;c->blue=100;}
  return HAL_OK;}

/* ---------- track ---------- */
#define MAXP 9000
#define POINTS_PER_BLOCK 64
typedef struct {double x,y,tx,ty;} Pt;
typedef struct {double xmin, xmax, ymin, ymax;} Bounds;
typedef struct {
  Pt p[MAXP];
  int n;
  Bounds bounds[(MAXP + POINTS_PER_BLOCK - 1) / POINTS_PER_BLOCK];
} Poly;
static Poly poly[3]; static int nPoly=0;
static double th_,tx_,ty_; 
static Poly *cur;
static void turtle_start(Poly*p,double x,double y,double h){cur=p;p->n=0;tx_=x;ty_=y;th_=h;}
static void emit(void){Pt*q=&cur->p[cur->n++];q->x=tx_;q->y=ty_;q->tx=cos(th_);q->ty=sin(th_);}
static void straight(double len){int n=(int)floor(len);double rem=len-n;for(int i=0;i<n;i++){tx_+=cos(th_);ty_+=sin(th_);emit();}tx_+=rem*cos(th_);ty_+=rem*sin(th_);}
static void arc(double R,double deg){ /* deg>0: trai (CCW) */
  double a=deg*M_PI/180.0; double s=a>0?1:-1; int n=(int)ceil(fabs(a)*R); double da=a/n;
  for(int i=0;i<n;i++){ double dl=R*fabs(da); double hm=th_+da/2; tx_+=dl*cos(hm); ty_+=dl*sin(hm); th_+=da; emit(); } (void)s;}
static void index_track(Poly *path)
{
  for (int i = 0; i < path->n; ++i)
  {
    const Pt *point = &path->p[i];
    Bounds *bounds = &path->bounds[i / POINTS_PER_BLOCK];
    if (i % POINTS_PER_BLOCK == 0)
    {
      *bounds = (Bounds){point->x, point->x, point->y, point->y};
      continue;
    }
    if (point->x < bounds->xmin) bounds->xmin = point->x;
    if (point->x > bounds->xmax) bounds->xmax = point->x;
    if (point->y < bounds->ymin) bounds->ymin = point->y;
    if (point->y > bounds->ymax) bounds->ymax = point->y;
  }
}

static double nearest_point(const Poly *path, double x, double y, double limit_sq,
                            int *nearest)
{
  double best_sq = limit_sq;
  *nearest = -1;
  for (int start = 0; start < path->n; start += POINTS_PER_BLOCK)
  {
    const Bounds *bounds = &path->bounds[start / POINTS_PER_BLOCK];
    const double dx = x < bounds->xmin ? bounds->xmin - x :
                      x > bounds->xmax ? x - bounds->xmax : 0.0;
    const double dy = y < bounds->ymin ? bounds->ymin - y :
                      y > bounds->ymax ? y - bounds->ymax : 0.0;
    if (dx * dx + dy * dy > best_sq) continue;

    int end = start + POINTS_PER_BLOCK;
    if (end > path->n) end = path->n;
    for (int i = start; i < end; ++i)
    {
      const double px = x - path->p[i].x;
      const double py = y - path->p[i].y;
      const double distance_sq = px * px + py * py;
      if (distance_sq <= best_sq)
      {
        best_sq = distance_sq;
        *nearest = i;
      }
    }
  }
  return best_sq;
}

static void build_track(void){
  double t=tan(22.5*M_PI/180.0)*800.0;
  turtle_start(&poly[0],3100,0,M_PI); emit(); straight(2600); arc(500,-180);
  double jx=tx_,jy=ty_,jh=th_;
  for(int b=0;b<2;b++){
    double sg=(b==0)?1:-1;
    turtle_start(&poly[1+b],jx,jy,jh); emit();
    straight(500-t); arc(800,45*sg); straight(500*sqrt(2.0)-2*t); arc(800,-45*sg); straight(1500-t); 
  }
  nPoly=3;
  for (int i = 0; i < nPoly; ++i) index_track(&poly[i]);
}
static int in_black(double qx,double qy){
  if(fabs(qx-1000)<=13 && fabs(qy)<=25) return 1;
  for(int k=0;k<nPoly;k++){
    const Poly *P = &poly[k];
    int bi;
    nearest_point(P, qx, qy, 13.0 * 13.0, &bi);
    if (bi < 0) continue;
    if(bi==0||bi==P->n-1){ double al=(qx-P->p[bi].x)*P->p[bi].tx+(qy-P->p[bi].y)*P->p[bi].ty;
      if(bi==0 && al<0) continue;
      if(bi==P->n-1 && al>0) continue; }
    return 1;
  }
  return 0;
}

static double frand(void){return (double)rand()/RAND_MAX-0.5;}
static void update_adc(void){
  static const uint16_t bl[5]={LINE_S1_BLACK,LINE_S2_BLACK,LINE_S3_BLACK,LINE_S4_BLACK,LINE_S5_BLACK};
  static const uint16_t wh[5]={LINE_S1_WHITE,LINE_S2_WHITE,LINE_S3_WHITE,LINE_S4_WHITE,LINE_S5_WHITE};
  double px=X+P_L*cos(PSI),py=Y+P_L*sin(PSI); double rx=sin(PSI),ry=-cos(PSI);
  for(int i=0;i<5;i++){
    double o=(i-2)*ROBOT_SENSOR_PITCH_MM; int c=0;
    for(int k=-2;k<=2;k++){double off=o+k*2.0; c+=in_black(px+off*rx,py+off*ry);}
    double cov=c/5.0;
    double adc=wh[i]+(bl[i]-wh[i])*cov;
    for (unsigned n = 0; n < LINE_OVERSAMPLE; ++n)
    {
      double raw = adc + P_noise * 2 * frand();
      adc_buffer[n * LINE_SENSOR_COUNT + LINE_SENSOR_COUNT - 1 - i] =
        (uint16_t)(fmax(raw, 0.0) + 0.5);
    }
  }
}
static void plant(double dt){
  for(int i=0;i<2;i++){
    double *v=(i==0)?&VL:&VR; double u=duty[i]; double K=(i==0)?P_Kl:P_Kr, db=(i==0)?P_DBl:P_DBr;
    double tau = (i == 0) ? MOTOR_L_TAU_S : MOTOR_R_TAU_S;
    if(heavy){K*=P_loadK;tau*=P_loadTau;}
    double target; 
    if(brake[i]){ *v += dt*(-*v/0.03); continue; }
    if(fabs(u)>db) target=(u>0?1:-1)*K*(fabs(u)-db); else target=0;
    if(u==0) tau=0.3;
    *v += dt*(target-*v)/tau;
  }
  double V=(VL+VR)/2, w=(VR-VL)/P_W;
  PSI+=w*dt; X+=V*cos(PSI)*dt; Y+=V*sin(PSI)*dt;
  double mm=ROBOT_MM_PER_COUNT*P_wheelScale;
  accL+=VL*dt/mm; accR+=VR*dt/mm;
  t1.CNT = (uint16_t)((int32_t)floor(accL) * ENCODER_LEFT_SIGN);
  t4.CNT = (uint16_t)((int32_t)floor(accR) * ENCODER_RIGHT_SIGN);
}

int main(int argc,char**argv){
  int seed=1; const char*csv=NULL;
  double offset = 0.0, heading = 0.0;
  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"blue"))P_branchBlue=1;
    else if(!strncmp(argv[i],"seed=",5))seed=atoi(argv[i]+5);
    else if(!strncmp(argv[i],"kl=",3))P_Kl=atof(argv[i]+3); else if(!strncmp(argv[i],"kr=",3))P_Kr=atof(argv[i]+3); else if(!strcmp(argv[i],"nomm")){P_Kl=P_Kr=10.5;P_DBl=P_DBr=5;}
    else if(!strncmp(argv[i],"L=",2))P_L=atof(argv[i]+2);
    else if(!strncmp(argv[i],"W=",2))P_W=atof(argv[i]+2);
    else if(!strncmp(argv[i],"scale=",6))P_wheelScale=atof(argv[i]+6);
    else if(!strncmp(argv[i],"offset=",7))offset=atof(argv[i]+7);
    else if(!strncmp(argv[i],"heading=",8))heading=atof(argv[i]+8);
    else if(!strncmp(argv[i],"noise=",6))P_noise=atof(argv[i]+6);
    else if(!strncmp(argv[i],"csv=",4))csv=argv[i]+4;
    else if(!strcmp(argv[i],"dark"))P_dark=1; else if(!strcmp(argv[i],"v"))verbose=1; else if(!strcmp(argv[i],"vv"))verbose=2; else if(!strcmp(argv[i],"vvv"))verbose=3;
  }
  srand(seed); build_track();
  block_blue=P_branchBlue;
  PSI = M_PI - heading * M_PI / 180.0;
  X = 3000.0 - P_L * cos(PSI);
  Y = offset - P_L * sin(PSI);
  Encoder_Init(); {static ADC_HandleTypeDef ha; LineSensor_Init(&ha);}
  AGV_Motion_Init(); {static I2C_HandleTypeDef hi; AGV_Mission_Init(&hi);};
  FILE *f = csv ? fopen(csv, "w") : NULL;
  if (csv && !f)
  {
    perror(csv);
    return EXIT_FAILURE;
  }
  if (f) fprintf(f, "t,x,y,s,v,state,mstate,derr,omega,e2_hat,theta_hat,line\n");
  /* ky vong nhanh */
  Poly*main_=&poly[0]; Poly*br=&poly[P_branchBlue?2:1];
  double maxe_fork=0,maxe_nf=0,maxe=0,maxe_t=0,maxe_x=0,maxe_y=0;
  double sum_sq = 0.0, straight_sq = 0.0, curve_sq = 0.0;
  unsigned samples = 0, straight_samples = 0, curve_samples = 0, lost_ticks = 0;
  uint32_t finished_ms = 0;
  int wait_t=-1, ended=0;
  double loadStopped_bar_x=0;
  for(uint32_t ms=0; ms<150000; ms++){
    sim_ms=ms;
    plant(0.001);
    const int control_tick = (ms % ROBOT_CONTROL_PERIOD_MS == 0);
    if(control_tick){ update_adc(); AGV_Motion_Tick(); if(verbose>=2 && ms<400){const LineSensor_Data_t*d=LineSensor_GetData();printf("t=%u raw %u %u %u %u %u val %u %u %u %u %u pos=%ld st=%d\n",ms,d->raw[0],d->raw[1],d->raw[2],d->raw[3],d->raw[4],d->value[0],d->value[1],d->value[2],d->value[3],d->value[4],(long)d->position,d->status);} }
    AGV_Mission_Run();
    MissionState_t ms_=AGV_Mission_GetState();
    if(ms_==MISSION_WAIT_LOAD){ if(wait_t<0){wait_t=ms; loadStopped_bar_x=X+P_L*cos(PSI);} if(ms-wait_t>1500){block_present=1;heavy=1;} }
    double bx=X+P_L*cos(PSI),by=Y+P_L*sin(PSI);
    if (control_tick && AGV_Motion_GetState() == AGV_MOTION_RUNNING &&
        AGV_Motion_GetEndDistance() == 0.0f)
    {
      int nearest;
      double error_sq = nearest_point(main_, bx, by, INFINITY, &nearest);
      const int on_curve = nearest > 2600 && nearest < main_->n - 1;
      double branch_sq = nearest_point(br, bx, by, error_sq, &nearest);
      double e = sqrt(branch_sq);
      sum_sq += branch_sq;
      ++samples;
      if (LineSensor_GetData()->status == LINE_LOST) ++lost_ticks;
      if (ms_ == MISSION_GO_LOAD)
      {
        straight_sq += error_sq;
        ++straight_samples;
      }
      if (on_curve && nearest < 0)
      {
        curve_sq += error_sq;
        ++curve_samples;
      }
      if(bx>=500&&bx<=1100&&by>700&&by<1300){if(e>maxe_fork)maxe_fork=e;} else if(e>maxe_nf)maxe_nf=e;
      if(e>maxe){maxe=e;maxe_t=ms;maxe_x=bx;maxe_y=by;}
    }
    if (f && control_tick)
    {
      fprintf(f, "%u,%.1f,%.1f,%.1f,%.1f,%d,%d,%.2f,%.3f,%.2f,%.4f,%d\n",
              ms, bx, by, AGV_Motion_GetDistance(), (VL + VR) / 2,
              (int)AGV_Motion_GetState(), (int)ms_, LineTracking_GetError(),
              AGV_Motion_GetOmegaCmd(), LineTracking_GetAxleErrorEst(),
              LineTracking_GetHeadingEst(), (int)LineSensor_GetData()->status);
    }
    if(ms_==MISSION_DONE||ms_==MISSION_FAULT){
      if (ended == 0) finished_ms = ms;
      if (ended++ > 1500) break;
    }
    if(verbose>=3 && ms%1000==0 && ms_==MISSION_WAIT_LOAD) printf("wl t=%u blk=%d base=%u clr=%u\n",ms,block_present,LoadDetect_GetBaseline(),LoadDetect_GetSample()->clear);
  }
  double bx=X+P_L*cos(PSI),by=Y+P_L*sin(PSI);
  Pt*e=&br->p[br->n-1];
  double endAlong=(bx-e->x)*e->tx+(by-e->y)*e->ty; double endLat=(bx-e->x)*(-e->ty)+(by-e->y)*e->tx;
  printf("final mission state=%d motion=%d\n",(int)AGV_Mission_GetState(),(int)AGV_Motion_GetState());
  printf("load stop: bar x=%.1f (muc tieu 1000) -> sai so doc = %.1f mm\n",loadStopped_bar_x,loadStopped_bar_x-1000);
  printf("  (xay ra o t=%.0f ms, vi tri bar=(%.0f,%.0f))\n",maxe_t,maxe_x,maxe_y);
  printf("  ngoai nga re: max=%.1f mm | trong nga re: max=%.1f mm\n",maxe_nf,maxe_fork);
  printf("max |lech tam hang mat so voi tam line| = %.1f mm\n",maxe);
  printf("tracking: rms=%.2f straight=%.2f curve=%.2f mm, lost_ms=%u, finish_ms=%u\n",
         samples ? sqrt(sum_sq / samples) : NAN,
         straight_samples ? sqrt(straight_sq / straight_samples) : NAN,
         curve_samples ? sqrt(curve_sq / curve_samples) : NAN,
         lost_ticks * ROBOT_CONTROL_PERIOD_MS, finished_ms);
  printf("end: sai so doc theo line = %+.1f mm (duong = qua cuoi line), ngang = %.1f mm; end_s=%.0f\n",endAlong,endLat,AGV_Motion_GetEndDistance());
  printf("end pos bar=(%.0f,%.0f) endpoint=(%.0f,%.0f)\n",bx,by,e->x,e->y);
  if(f)fclose(f);
  return (AGV_Mission_GetState() == MISSION_DONE) ? EXIT_SUCCESS : EXIT_FAILURE;
}
