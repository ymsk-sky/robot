/*
 * 2018 / 07 / 25 作成
 * log20180726 必要関数作成・テンプレのみ記述、規定作成
 * log20180727 必要関数作成・コンパイル通過確認
 *             head, bodyのcreate, draw: エラー吐いてる
 * log20180730 エラー解消, 外見完成
 *             コントロールは../13_manipulator(DEMURA.NET)を参考にすると良いかも
 * log20180731 外見微修正
 * log20180801 pause対応, restart対応(destroyRobot関数 追加)
 *             control(P制御)できるようになった, bodyとfootのmをMyObjectで定義
 *             重心を求める関数(cogSensor)作成
 * log20180802 コントロール部修正
 * log20180809 balance関数追加
 * log20180816 github利用開始
 * 二足歩行ロボット（動歩行）シミュレーション
 * by yamasaki
 *
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * 関数(function)のカッコは一行使う
 * ピリオドカンマの後に文字が続くときは半角スペースを入れる
 * ポインタのアスタリスクは変数名にくっつける
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD      // 球
#define dsDrawBox dsDrawBoxD            // 直方体
#define dsDrawCylinder dsDrawCylinderD  // 筒
#define dsDrawCapsule dsDrawCapsuleD    // カプセル
#endif

#define W 720     // グラフィックウィンドウの幅   Width
#define H 450     // グラフィックウィンドウの高さ Height
#define NUM 2     // 左右対称に組み立てる際に使用（左1右1で計2）
#define HIP 0     // 股関節を指定
#define ANKLE 1   // 足首を指定

typedef struct {  // MyObject構造体
  dBodyID body; // 剛体のID番号(動力学計算用)
  dGeomID geom; // ジオメトリのID番号(衝突検出計算用)
  dReal l;      // 剛体の長さ
  dReal r;      // 剛体の半径
  dReal m;      // 剛体の質量
} MyObject;

/***** シミュレータ形成 *****/
static dWorldID world;              // 動力学ワールド
static dSpaceID space;              // 衝突検出用スペース
static dGeomID ground;              // 地面
static dJointGroupID contactgroup;  // コンタクトグループ
dsFunctions fn;                     // ドロースタッフ用の構造体

/***** 実体を形成 *****/
MyObject head;        // 頭 sphere
MyObject body;        // 胴体 box
MyObject leg[NUM];    // 脚 capsule
MyObject foot[NUM];   // 足先 box

/***** 剛体同士を接続 *****/
dJointID neck_joint;        // 頭と胴体のジョイント(首部分)
dJointID hip_joint[NUM];    // 胴体と脚のジョイント(hip joint:股関節)
dJointID ankle_joint[NUM];  // 脚と足先のジョイント(ankle:足首)

/***** 各剛体の長さパラメータを設定 *****/
static const dReal BODY_L[3] = {0.20, 0.40, 0.50};  // 胴体(body)のxyz長パラメータ
static const dReal FOOT_L[3] = {0.20, 0.02, 0.01};  // 足先(foot)のxyz長パラメータ

/***** 各ジョイントの最大最小角度 *****/
static const dReal HIP_MAX = M_PI/3;
static const dReal HIP_MIN = -M_PI/3;
static const dReal ANKLE_MAX = M_PI/3;
static const dReal ANKLE_MIN = -M_PI/3;

/***** 変動するパラメータ *****/
dReal hip_target_angle[2] = {0.0, 0.0};           // hip_jointの目標ヒンジ角度
dReal ankle_target_angle[2] = {0.0, 0.0};         // ankle_jointの目標ヒンジ角度

dReal hip_current_angle[2] = {0.0, 0.0};          // hip_jointの現在のヒンジ角度
dReal ankle_current_angle[2] = {0.0, 0.0};        // ankle_jointの現在のヒンジ角度
double tpos[3] = {0.0, 0.0, 0.0};                 // 物体の重心の座標

// コールバック関数(衝突をジョイント拘束で実装している)
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10;  // 同時に衝突する可能性のある点数
  dContact contact[N];

  // 接触している物体のどちらかが地面ならisGroundに非0をセット
  int isGround = ((ground == o1) || (ground == o2));

  // 2つの剛体がジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  // 衝突情報の生成
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  if(isGround) {
    for(int i=0; i<n; i++) {
      contact[i].surface.mu = dInfinity;            // 摩擦係数
      contact[i].surface.mode = dContactBounce;     // 接触面の反発性を設定
      contact[i].surface.bounce = 0.0;              // 反発係数
      contact[i].surface.bounce_vel = 0.0;          // 反発最低速度
      //contact[i].surface.soft_erp = 0.2;            // 関節誤差修正
      //contact[i].surface.soft_cfm = 0.0001;         // 拘束力混合

      // 接触ジョイントの生成
      dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
      // 接触している2つの剛体を接触ジョイントにより拘束
      dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

/***** ***** ***** センサ実装 ***** ***** *****/
/*
 * cogSensor: ロボットの重心を求める関数
 *             ロボットを構成するすべての剛体の重心と質量から、全体の重心を求めている
 */
// ジョイント全ての現在の角度を求める
static void angleSensor()
{
  for(int i=0; i<NUM; i++) {
    hip_current_angle[i] = dJointGetHingeAngle(hip_joint[i]);
    ankle_current_angle[i] = dJointGetHingeAngle(ankle_joint[i]);
  }
}
// 重心を求める関数
static void cogSensor(double cog[3])
{
  double mass[6];           // 6物体の重量
  double sum_mass = 0.0;    // 物体の質量の合計

  const dReal *pos_head;
  const dReal *pos_body;
  const dReal *pos_leg[2];
  const dReal *pos_foot[2];

  pos_head = dBodyGetPosition(head.body);
  pos_body = dBodyGetPosition(body.body);
  for(int i=0; i<NUM; i++) {
    pos_leg[i] = dBodyGetPosition(leg[i].body);
    pos_foot[i] = dBodyGetPosition(foot[i].body);
  }

  mass[0] = head.m;
  mass[1] = body.m;
  mass[2] = leg[0].m;
  mass[3] = leg[1].m;
  mass[4] = foot[0].m;
  mass[5] = foot[1].m;

  for(int i=0; i<6; i++) sum_mass += mass[i];   // 質量の合計を計算

  for(int i=0; i<3; i++) {
    cog[i] = (pos_head[i]*mass[0] + pos_body[i]*mass[1] +
                pos_leg[0][i]*mass[2] + pos_leg[1][i]*mass[3] +
                 pos_foot[0][i]*mass[4] + pos_foot[1][i]*mass[5]) / sum_mass;
  }
}

// ジョイントの角度の可動域内かチェックする関数
void checkAngleRange()
{
  if(hip_target_angle[0] > HIP_MAX) hip_target_angle[0] = HIP_MAX;
  if(hip_target_angle[0] < HIP_MIN) hip_target_angle[0] = HIP_MIN;
  if(hip_target_angle[1] > HIP_MAX) hip_target_angle[1] = HIP_MAX;
  if(hip_target_angle[1] < HIP_MIN) hip_target_angle[1] = HIP_MIN;
  if(ankle_target_angle[0] > ANKLE_MAX) ankle_target_angle[0] = ANKLE_MAX;
  if(ankle_target_angle[0] < ANKLE_MIN) ankle_target_angle[0] = ANKLE_MIN;
  if(ankle_target_angle[1] > ANKLE_MAX) ankle_target_angle[1] = ANKLE_MAX;
  if(ankle_target_angle[1] < ANKLE_MIN) ankle_target_angle[1] = ANKLE_MIN;
}

// その場で立ち続ける動作を行なう関数
static void balance()
{
  //
}

// control(P制御)
static void control()
{
  double k1 = 10.0, fMax = 1000.0;

  for(int i=0; i<NUM; i++) {
    double tmp_hip_target_angle = dJointGetHingeAngle(hip_joint[i]);
    double tmp_ankle_target_angle = dJointGetHingeAngle(ankle_joint[i]);
    double h = hip_target_angle[i] - tmp_hip_target_angle;
    double a = ankle_target_angle[i] - tmp_ankle_target_angle;

    dJointSetHingeParam(hip_joint[i], dParamVel, k1*h);
    dJointSetHingeParam(hip_joint[i], dParamFMax, fMax);
    dJointSetHingeParam(ankle_joint[i], dParamVel, k1*a);
    dJointSetHingeParam(ankle_joint[i], dParamFMax, fMax);
  }

  //printf("股関節 右:%f 左:%f ", hip_target_angle[0], hip_target_angle[1]);
  //printf("足首 右:%f 左:%f\n", ankle_target_angle[0], ankle_target_angle[1]);
}

// ロボットを描画する関数
void drawRobot()
{
  // head - sphere
  dsSetColor(1.0, 0.0, 0.0);  // 赤
  dsDrawSphere(dBodyGetPosition(head.body),
               dBodyGetRotation(head.body), head.r);

  // body - box
  dsSetColor(0.5, 0.5, 0.5);  // 灰
  dsDrawBox(dBodyGetPosition(body.body),
            dBodyGetRotation(body.body), BODY_L);

  // leg - capsule
  dsSetColor(0.0, 0.0, 1.0);  // 青
  for(int i=0; i<NUM; i++) {
    dsDrawCapsule(dBodyGetPosition(leg[i].body),
                  dBodyGetRotation(leg[i].body), leg[i].l, leg[i].r);
  }

  // foot - box
  dsSetColor(0.0, 0.0, 0.0);  // 黒　
  for(int i=0; i<NUM; i++) {
    dsDrawBox(dBodyGetPosition(foot[i].body),
              dBodyGetRotation(foot[i].body), FOOT_L);
  }
}

// ロボットを創造する関数
void createRobot()
{
  // 重心位置として定義したが使用していない(20180730)
  dReal x0 = 0.00;                  // X軸 初期位置
  dReal y0 = 0.00;                  // Y軸 初期位置
  dReal z0 = 0.70;                  // Z軸 初期位置

  head.r = 0.10;                    // 頭(head)の半径r
  head.m = 4.0;                     // 頭(head)の質量m

  body.m = 12.0;                    // 胴体(body)の質量m

  for(int i=0; i<NUM; i++) {
    leg[i].r = 0.03;                // 脚(leg)の半径r
    leg[i].l = 0.59 - leg[i].r * 2; // 脚(leg)の長さl
    leg[i].m = 1.50;                // 脚(leg)の質量m

    foot[i].m = 0.50;                  // 足先(foot)の質量m
  }

  dMass mass;

  /***** MyObject *****/
  // 頭【球 - sphere】
  head.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, head.m, head.r);
  dBodySetMass(head.body, &mass);
  dBodySetPosition(head.body, 0.0, 0.0,
                    FOOT_L[2] + leg[0].l + 2*leg[0].r + BODY_L[2] + head.r);

  head.geom = dCreateSphere(space, head.r);
  dGeomSetBody(head.geom, head.body);

  // 胴体【直方体 - box】
  body.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, body.m, BODY_L[0], BODY_L[1], BODY_L[2]);
  dBodySetMass(body.body, &mass);
  dBodySetPosition(body.body, 0.0, 0.0,
                    FOOT_L[2] + leg[0].l + 2*leg[0].r + BODY_L[2]/2);

  body.geom = dCreateBox(space, BODY_L[0], BODY_L[1], BODY_L[2]);
  dGeomSetBody(body.geom, body.body);

  // 脚【カプセル - capsule】
  for(int i=0; i<NUM; i++) {
    leg[i].body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass, leg[i].m, 3, leg[i].r, leg[i].l);
    // 第三引数の3は長軸方向(1=x軸, 2=y軸, 3=z軸)
    dBodySetMass(leg[i].body, &mass);
    leg[i].geom = dCreateCapsule(space, leg[i].r, leg[i].l);
    dGeomSetBody(leg[i].geom, leg[i].body);
  }
  dBodySetPosition(leg[0].body, 0.0, -BODY_L[1]/4,
                                FOOT_L[2] + leg[0].r + leg[0].l/2);
  dBodySetPosition(leg[1].body, 0.0,  BODY_L[1]/4,
                                FOOT_L[2] + leg[0].r + leg[1].l/2);

  // 足先【直方体 - box】
  for(int i=0; i<NUM; i++) {
    foot[i].body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, foot[i].m, FOOT_L[0], FOOT_L[1], FOOT_L[2]);
    dBodySetMass(foot[i].body, &mass);
    foot[i].geom = dCreateBox(space, FOOT_L[0], FOOT_L[1], FOOT_L[2]);
    dGeomSetBody(foot[i].geom, foot[i].body);
  }
  dBodySetPosition(foot[0].body, 0.05, -BODY_L[1]/4, FOOT_L[2]/2);
  dBodySetPosition(foot[1].body, 0.05,  BODY_L[1]/4, FOOT_L[2]/2);

  /***** ジョイント *****/
  // head - body
  neck_joint = dJointCreateFixed(world, 0);         // Fixed:固定ジョイント
  dJointAttach(neck_joint, head.body, body.body);
  dJointSetFixed(neck_joint);

  // body - leg
  for(int i=0; i<NUM; i++) {
    hip_joint[i] = dJointCreateHinge(world, 0);     // Hinge: 蝶番ジョイント
    dJointAttach(hip_joint[i], body.body, leg[i].body);
    dJointSetHingeAxis(hip_joint[i], 0, 1, 0);
    dJointSetHingeParam(hip_joint[i], dParamLoStop, -M_PI/3);
    dJointSetHingeParam(hip_joint[i], dParamHiStop,  M_PI/3);
  }
  dJointSetHingeAnchor(hip_joint[0], 0.0, -BODY_L[1]/4,
                                     FOOT_L[2] + leg[0].l);
  dJointSetHingeAnchor(hip_joint[1], 0.0,  BODY_L[1]/4,
                                     FOOT_L[2] + leg[1].l);

  // leg - foot
  for(int i=0; i<NUM; i++) {
    ankle_joint[i] = dJointCreateHinge(world, 0);     // Hinge: 蝶番ジョイント
    dJointAttach(ankle_joint[i], leg[i].body, foot[i].body);
    dJointSetHingeAxis(ankle_joint[i], 0, 1, 0);
    dJointSetHingeParam(ankle_joint[i], dParamLoStop, -M_PI/3);
    dJointSetHingeParam(ankle_joint[i], dParamHiStop,  M_PI/3);
  }
  dJointSetHingeAnchor(ankle_joint[0], 0.0, -BODY_L[1]/4, FOOT_L[2]/4);
  dJointSetHingeAnchor(ankle_joint[1], 0.0,  BODY_L[1]/4, FOOT_L[2]/4);
}

// シミュレーションループ
void simLoop(int pause)
{
  if(!pause) {
    dSpaceCollide(space, 0, &nearCallback);   // 衝突検出関数
    dWorldStep(world, 0.01);                  // シミュレーションを1ステップ進める
    dJointGroupEmpty(contactgroup);           // ジョイントグループを空にする

    balance();        // 立ち続ける関数
    control();        // 動作を制御する関数（commandでジョイントの角度を変更する）

    angleSensor();
    printf("股関節\t右:%f\t左:%f\n", hip_current_angle[0], hip_current_angle[1]);
    printf("足首\t右:%f\t左:%f\n", ankle_current_angle[0], ankle_current_angle[1]);
  }
  drawRobot();
}

void destroyRobot()
{
  dJointDestroy(neck_joint);
  dBodyDestroy(head.body);
  dBodyDestroy(body.body);
  dGeomDestroy(head.geom);
  dGeomDestroy(body.geom);

  for(int i=0; i<NUM; i++) {
    dJointDestroy(hip_joint[i]);
    dJointDestroy(ankle_joint[i]);
    dBodyDestroy(leg[i].body);
    dBodyDestroy(foot[i].body);
    dGeomDestroy(leg[i].geom);
    dGeomDestroy(foot[i].geom);
  }
}

// リセット関数（シミュレータのやり直し）
static void restart()
{
  destroyRobot();
  dJointGroupDestroy(contactgroup);
  contactgroup = dJointGroupCreate(0);

  hip_target_angle[0] = 0.0, hip_target_angle[1] = 0.0;
  ankle_target_angle[0] = 0.0, ankle_target_angle[1] = 0.0;

  createRobot();
}

// キー入力を処理する関数
static void command(int cmd)
{
  switch(cmd) {
    case 'j': // hip R +
      hip_target_angle[0] += 0.1;
      break;
    case 'k': // hip R -
      hip_target_angle[0] -= 0.1;
      break;
    case 'n': // ankle R +
      ankle_target_angle[0] += 0.1;
      break;
    case 'm': // ankle R -
      ankle_target_angle[0] -= 0.1;
      break;
    case 'f': // hip L +
      hip_target_angle[1] += 0.1;
      break;
    case 'd': // hip L -
      hip_target_angle[1] -= 0.1;
      break;
    case 'v': // ankle L +
      ankle_target_angle[1] += 0.1;
      break;
    case 'c': // ankle L -
      ankle_target_angle[1] -= 0.1;
      break;
    case 'a':
      ankle_target_angle[0] = ANKLE_MIN / 2;
      ankle_target_angle[1] = ANKLE_MIN / 2;
      break;
    case 'r':
      printf("restart\n");
      restart();
      break;
    case 'o':
      cogSensor(tpos);
      printf("重心(%f, %f, %f)\n", tpos[0], tpos[1], tpos[2]);
      break;
    default:
      printf("cannot use button\n");
      break;
  }

  // 可動域を超えないようにする
  checkAngleRange();
}

// 前処理
void start()
{
  /***** 視点とカメラの設定をする。 *****
   * xyz: 点(x, y, z)の位置に視線を設置
   * hpr: 視線の方向(heading, pitch, roll)を設定 *単位[deg]に注意
   * - h: heading x軸の旋回
   * - p: pitch   上下方向
   * - r: roll    左右の傾き
   */
  static float xyz[3] = {0.0, 2.0, 0.8};     // 視点の位置
  static float hpr[3] = {-90.0, 0.0, 0.0};   // 視点の方向
  dsSetViewpoint(xyz, hpr);                  // カメラを設定

  // 球は三角錐で形成する。その品質を設定。3で充分キレイ
  // 値が高すぎると描画に時間がかかる
  dsSetSphereQuality(3);                    // 球の品質を設定
}

// 描画関数の設定
void setDrawStuff()
{
  /*
   * 各パラメータに実装した関数のポインタを渡す。
   * 実装していない(ex: キー入力は受け付けない→command関数なし)ときは
   * nullを渡す(ex: fn.command = null)。
   * テクスチャのパスは注意 *間違っているとエラー出る。
   */
  fn.version = DS_VERSION;              // ドロースタッフのバージョン
  fn.start = &start;                    // 前処理(start関数)のポインタ
  fn.step = &simLoop;                   // simLoop関数のポインタ
  fn.command = &command;                // command関数のポインタ
  fn.path_to_textures = "../textures";  // 読み込むテクスチャのパス
}

// main関数
int main(int argc, char *argv[])
{
  setDrawStuff();                           // 描画関数の設定
  dInitODE();                               // ODEの初期化
  world = dWorldCreate();                   // 世界の創造
  dWorldSetGravity(world, 0, 0, -9.81);     // 重力を展開(g=9.81m/s^2)

  space = dHashSpaceCreate(0);              // 衝突用空間を創造
  contactgroup = dJointGroupCreate(0);      // ジョイントグループ生成
  ground = dCreatePlane(space, 0, 0, 1, 0); // 地面を平面ジオメトリで生成

  createRobot();                            // ロボットを生成

  dsSimulationLoop(argc, argv, W, H, &fn);  // シミュレーションループ
  dSpaceDestroy(space);                     // スペースを破棄
  dWorldDestroy(world);                     // ワールドを破棄
  dCloseODE();                              // ODEの終了

  return 0;
}
