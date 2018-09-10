#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

typedef struct {
  dBodyID body;
  dGeomID geom;
  dReal m;
  dReal r, l, w, h, d;
} MyObject;

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactgroup;
dsFunctions fn;

MyObject base, obj;
dJointID b, j;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  if(isGround) {
    for(int i=0; i<n; i++) {
      contact[i].surface.mu = dInfinity;
      contact[i].surface.mode = dContactBounce;
      contact[i].surface.bounce = 0.2;
      contact[i].surface.bounce_vel = 0.0;

      dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
      dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

double target = M_PI / 4;
static void P_control()
{
  double Kp = 10.0;   // 比例ゲイン
  double fMax = 100.0;

  double current = dJointGetHingeAngle(j);
  double v = target - current;

  dJointSetHingeParam(j, dParamVel, Kp*v);
  dJointSetHingeParam(j, dParamFMax, fMax);

}

void drawObj()
{
  dsSetColor(0.5, 0.5, 0.5);
  dsDrawCylinder(dBodyGetPosition(base.body),
                dBodyGetRotation(base.body), base.l, base.r);

  dsSetColor(1.0, 0.0, 0.0);
  dsDrawCapsule(dBodyGetPosition(obj.body),
                dBodyGetRotation(obj.body), obj.l, obj.r);
}

void createObj()
{
  base.m = 1.000;
  base.r = 1.00;
  base.l = 0.03;

  obj.m = 3.000;
  obj.r = 0.01;
  obj.l = 3.00;

  dMass mass;

  // オブジェクト
  base.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass, base.m, 3, base.r, base.l);
  dBodySetMass(base.body, &mass);
  dBodySetPosition(base.body, 0.0, 0.0, base.l/2);
  base.geom = dCreateCylinder(space, base.r, base.l);
  dGeomSetBody(base.geom, base.body);

  obj.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass, obj.m, 1, obj.r, obj.l);
  dBodySetMass(obj.body, &mass);
  dBodySetPosition(obj.body, 0.0, 0.0, obj.l/2 + obj.r + base.l/2);
  obj.geom = dCreateCapsule(space, obj.r, obj.l);
  dGeomSetBody(obj.geom, obj.body);

  // ジョイント
  b = dJointCreateFixed(world, 0);
  dJointAttach(b, base.body, 0);
  dJointSetFixed(b);

  j = dJointCreateHinge(world, 0);
  dJointAttach(j, obj.body, 0);
  dJointSetHingeAxis(j, 0, 1, 0);
  dJointSetHingeAnchor(j, 0.0, 0.0, base.l);
  dJointSetHingeParam(j, dParamHiStop,  M_PI/2);
  dJointSetHingeParam(j, dParamLoStop, -M_PI/2);
}

void simLoop(int pause)
{
  if(!pause) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, 0.01);
    dJointGroupEmpty(contactgroup);

    double d = dJointGetHingeAngle(j);
    printf("%f\n", d);
    P_control();
  }
  drawObj();
}

void destroyObj()
{
  dJointDestroy(b);
  dJointDestroy(j);
  dBodyDestroy(base.body);
  dGeomDestroy(base.geom);
  dBodyDestroy(obj.body);
  dGeomDestroy(obj.geom);
}

static void restart()
{
  destroyObj();
  dJointGroupDestroy(contactgroup);
  contactgroup = dJointGroupCreate(0);

  createObj();
}

double val = 0.01;
void command(int cmd)
{
  switch(cmd) {
    case 'r':
      restart();
      break;
    case ' ':
      printf("space: val=%f\n", val);
      dWorldSetGravity(world, val, val, -9.81);
      if(val == 0.01) val = 0.00;
      else val = 0.01;
      break;
    default:
      printf("not command");
  }
}

void start()
{
  static float xyz[3] = {0.0, -2.0, 1.0};
  static float hpr[3] = {90.0, 0.0, 0.0};

  dsSetViewpoint(xyz, hpr);
  dsSetSphereQuality(3);
}

void setDrawStuff()
{
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.path_to_textures = "../textures";
}

int main(int argc, char *argv[])
{
  setDrawStuff();
  dInitODE();
  world = dWorldCreate();
  dWorldSetGravity(world, 0.0, 0.0, -9.81);

  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);

  createObj();

  dsSimulationLoop(argc, argv, 500, 500, &fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}
