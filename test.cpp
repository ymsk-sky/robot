#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#endif

typedef struct {
  dBodyID body;
  dGeomID geom;
  dReal m;
} MyObject;

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactgroup;
dsFunctions fn;

MyObject bar[2];
dJointID bar_joint;

dReal bar_l[3] = {0.75, 0.1, 0.1};

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
      contact[i].surface.bounce = 0.0;
      contact[i].surface.bounce_vel = 0.0;

      dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
      dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

void drawFunc()
{
  dsSetColor(1.0, 0.0, 0.0);
  dsDrawBox(dBodyGetPosition(bar[0].body),
            dBodyGetRotation(bar[0].body), bar_l);

  dsSetColor(0.0, 1.0, 0.0);
}

void destroyFunc()
{
  dBodyDestroy(bar[0].body);
  dGeomDestroy(bar[0].geom);
}

void createFunc()
{
  bar[0].m = 1.0;
  bar[1].m = 1.0;

  dMass mass;

  bar[0].body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, bar[0].m, bar_l[0], bar_l[1], bar_l[2]);
  dBodySetMass(bar[0].body, &mass);
  dBodySetPosition(bar[0].body, 0.0, 0.0, 0.5);
  // set rotation
  const dMatrix3 ir = {1,0,0,0, 0,1,0,0, M_PI/3,0,0,0};
  dBodySetRotation(bar[0].body, ir);
  // --
  bar[0].geom = dCreateBox(space, bar_l[0], bar_l[1], bar_l[2]);
  dGeomSetBody(bar[0].geom, bar[0].body);
}

void simLoop(int pause)
{
  if(!pause) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, 0.01);
    dJointGroupEmpty(contactgroup);
  }
  drawFunc();
}

static void restart()
{
  destroyFunc();
  dJointGroupDestroy(contactgroup);
  contactgroup = dJointGroupCreate(0);
  createFunc();
}

void command(int cmd)
{
  switch(cmd) {
    case 'r':
      restart();
      break;
  }
}

void start()
{
  static float xyz[3] = {0.0, 2.0, 0.5};
  static float hpr[3] = {-90.0, 0.0, 0.0};

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
  dWorldSetGravity(world, 0.0, 0.0, -5.0);

  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);

  createFunc();

  dsSimulationLoop(argc, argv, 500, 500, &fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}
