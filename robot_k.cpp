/*MATLABで計算したゲインを入力としてバランスをとらせる*/
/*スペースキーで開始*/

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

int flag = 0;       //足が地面についてるか判定フラグ
int flag_a = 0;

typedef struct {
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
MyObject arm[NUM];		//腕

/***** 剛体同士を接続 *****/
dJointID neck_joint;        // 頭と胴体のジョイント(首部分)
dJointID hip_joint[NUM];    // 胴体と脚のジョイント(hip joint:股関節)
dJointID ankle_joint[NUM];  // 脚と足先のジョイント(ankle:足首)
dJointID fixed_joint[2];   //足を地面に固定したいときに使用
dJointID shol_joint[NUM];	//腕用

/***** 各剛体の長さパラメータを設定 *****/
static const dReal BODY_L[3] = { 0.20, 0.40, 0.50 };  // 胴体(body)のxyz長パラメータ
static const dReal FOOT_L[3] = { 0.1, 0.1, 0.02}; // 足先(foot)のxyz長パラメータ　足が突っかかっちゃうからとりあえず板型に変更
static const dReal ARM_L[3] = { 0.1,0.05,0.5 };       //腕のxyz

/***** 各ジョイントの最大最小角度 *****/
static const dReal HIP_MAX = 2.0 / 15.0 * M_PI;      // 小三の歩幅を参照
static const dReal HIP_MIN = -2.0 / 15.0 * M_PI;      // 小三の歩幅を参照
static const dReal ANKLE_MAX = M_PI / 3;
static const dReal ANKLE_MIN = -M_PI / 3;

/***** 変動するパラメータ *****/
dReal hip_target_angle[2] = { 0.0, 0.0 };           // hip_jointの目標ヒンジ角度
dReal ankle_target_angle[2] = { 0.0, 0.0 };         // ankle_jointの目標ヒンジ角度
dReal shol_target_angre[2] = { 0.0, 0.0 };
dReal hip_current_angle[2] = { 0.0, 0.0 };          // hip_jointの現在のヒンジ角度
dReal ankle_current_angle[2] = { 0.0, 0.0 };        // ankle_jointの現在のヒンジ角度
dReal body_angle[3] = { 0.0, 0.0, 0.0 };            // bodyのxyz軸の回転
dReal body_angular_velocity[3] = { 0.0, 0.0, 0.0 }; // bodyのxyz軸の角速度

double tpos[3] = { 0.0, 0.0, 0.0 };                 // 物体の重心の座標
bool space_trigger = false;     // スペースキー押下フラグ
bool stand_flag = false;        // つま先立ちフラグ

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
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
	// 衝突情報の生成
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if (isGround)
	{
		for (int i = 0; i < n; i++) {
			contact[i].surface.mu =200;            // 摩擦係数
			contact[i].surface.mode = dContactBounce;     // 接触面の反発性を設定
			contact[i].surface.bounce = 0.8;           // 反発係数
			contact[i].surface.bounce_vel = 0.0;          // 反発最低速度
			// 接触ジョイントの生成
			dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
			// 接触している2つの剛体を接触ジョイントにより拘束
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
				dGeomGetBody(contact[i].geom.g2));
		}

		if (flag == 0 && ground == o1 || flag == 0 && ground == o2) {
    // 足が地面についていないで片方が地面のとき
			if (o1 == foot[0].geom || o1 == foot[1].geom || o2 == foot[0].geom || o2 == foot[1].geom) {
      // 片方が左右どちらかの足のとき
				flag = 1;
			}
		}
		if (flag == 1 && ground == o1 || flag == 1 && ground == o2) {
    // 足が地面についていて片方が地面のとき
			if (o1 == foot[0].geom || o1 == foot[1].geom || o2 == foot[0].geom || o2 == foot[1].geom) {
        // 片方が左右どちらかの足のとき
				flag = 2;
			}
		}
	}
}

/***** ***** ***** センサ実装 ***** ***** *****/
// body(胴体)のxyz軸方向の角速度を取得する
static void setBodyAngularVelocity(dReal vel[3])
{
	const dReal *av;
	av = dBodyGetAngularVel(body.body);
	for (int i = 0; i<3; i++) {
		vel[i] = av[i];
	}
}

static void setLeg_LAngularVelocity(dReal vel[3])
{
	const dReal *av_L;
	av_L = dBodyGetAngularVel(leg[0].body);
	for (int i = 0; i<3; i++) {
		vel[i] = av_L[i];
	}
}

static void setLeg_RAngularVelocity(dReal vel[3])
{
	const dReal *av_R;
	av_R = dBodyGetAngularVel(leg[1].body);
	for (int i = 0; i<3; i++) {
		vel[i] = av_R[i];
	}
}

// body(胴体)のroll,pitch,yawを取得する
static void setBodyAngle(dReal rpy[3])
{
	// 内容はODE教本P150参照
	const dReal *rot = dBodyGetRotation(body.body);
	dReal r11, r12, r13, r21, r22, r23, r31, r32, r33;
	dReal pitch, yaw, roll;

	r11 = *(rot + 0); r12 = *(rot + 1); r13 = *(rot + 2);
	r21 = *(rot + 4); r22 = *(rot + 5); r23 = *(rot + 6);
	r31 = *(rot + 8); r32 = *(rot + 9); r33 = *(rot + 10);

	// r:roll(x軸の回転), p:pitch(y軸の回転), y:yaw(z軸の回転)
	rpy[0] = atan2(r32, r33) * 180.0 / M_PI;
	rpy[1] = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) * 180.0 / M_PI;
	rpy[2] = atan2(r21, r11) * 180.0 / M_PI;
}

// ジョイント全ての現在の角度を求める
static void angleSensor()
{
	for (int i = 0; i<NUM; i++) {
		hip_current_angle[i] = dJointGetHingeAngle(hip_joint[i]);
		ankle_current_angle[i] = dJointGetHingeAngle(ankle_joint[i]);
	}
}

// 重心を求める関数
static void cogSensor(double cog[3])
{
	// ロボットを構成するすべての剛体の重心と質量から、全体の重心を求めている
	double mass[8];           // 6物体の重量+腕が２
	double sum_mass = 0.0;    // 物体の質量の合計

	const dReal *pos_head;
	const dReal *pos_body;
	const dReal *pos_leg[2];
	const dReal *pos_foot[2];
	const dReal *pos_arm[2];
	pos_head = dBodyGetPosition(head.body);
	pos_body = dBodyGetPosition(body.body);
	for (int i = 0; i<NUM; i++) {
		pos_leg[i] = dBodyGetPosition(leg[i].body);
		pos_foot[i] = dBodyGetPosition(foot[i].body);
		pos_arm[i] = dBodyGetPosition(arm[i].body);
	}

	mass[0] = head.m;
	mass[1] = body.m;
	mass[2] = leg[0].m;
	mass[3] = leg[1].m;
	mass[4] = foot[0].m;
	mass[5] = foot[1].m;
	mass[6] = arm[0].m;
	mass[7] = arm[1].m;

	for (int i = 0; i</*6*/8; i++) sum_mass += mass[i];   // 質量の合計を計算
	for (int i = 0; i<3; i++) {
		cog[i] = (pos_head[i] * mass[0] + pos_body[i] * mass[1] +
		          pos_leg[0][i] * mass[2] + pos_leg[1][i] * mass[3] +
		          pos_foot[0][i] * mass[4] + pos_foot[1][i] * mass[5]) / sum_mass;
		          pos_arm[0][i] * mass[6] + pos_arm[1][i] * mass[7];
	}
}

// ジョイントの角度の可動域内かチェックする関数
void checkAngleRange()
{
	//変に角度を設定しちゃうと変な動きになる
	//とりあえず歩幅が±3deg 足先は±60deg
	if (hip_target_angle[0] > HIP_MAX) hip_target_angle[0] = 3.0*M_PI / 180;//HIP_MAX;
	if (hip_target_angle[0] < HIP_MIN) hip_target_angle[0] = -3.0*M_PI / 180;//HIP_MIN;
	if (hip_target_angle[1] > HIP_MAX) hip_target_angle[1] = 3.0*M_PI / 180;//HIP_MAX;
	if (hip_target_angle[1] < HIP_MIN) hip_target_angle[1] = -3.0*M_PI / 180;//HIP_MIN;
	if (ankle_target_angle[0] > ANKLE_MAX) ankle_target_angle[0] = 60 * M_PI / 180;//ANKLE_MAX;
	if (ankle_target_angle[0] < ANKLE_MIN) ankle_target_angle[0] = -60 * M_PI / 180;//;ANKLE_MIN;
	if (ankle_target_angle[1] > ANKLE_MAX) ankle_target_angle[1] = 60 * M_PI / 180;// ANKLE_MAX;
	if (ankle_target_angle[1] < ANKLE_MIN) ankle_target_angle[1] = -60 * M_PI / 180;//ANKLE_MIN;
}

// その場で立ち続ける動作を行なう関数
static void balance()
{
//以下は目標の歩幅を変更してMATLABから出したゲイン、試してないから動くかはわからない
/* 3.1623   15.5892    3.0473    2.8128    10deg*/
/*3.1623   17.2150    3.0209    3.0808     30deg*/
/* 3.1623   12.4907    2.4811    2.3378    15deg*/
	if (space_trigger) {
		if (!stand_flag) {
			if (hip_current_angle[0] > hip_current_angle[1] //L>R
				&& flag == 2  )                   //両足が着地フラグ
        {
				//θ2'を設定
				setBodyAngularVelocity(body_angular_velocity);  // 胴体の角速度を求める
				const dReal *Body_current_anglev = dBodyGetAngularVel(head.body);
				double Body_target_anglev = 0;
				double dtheta2 = Body_target_anglev - Body_current_anglev[1];

				//θ1'[0]を設定
				setLeg_LAngularVelocity(body_angular_velocity);  //足Lの角速度
				const dReal *Leg_L_current_anglev = dBodyGetAngularVel(leg[0].body);
				double Leg_L_target_anglev = 0;
				double dtheta1_0 = Leg_L_target_anglev - Leg_L_current_anglev[1];

				//θ1'[1]を設定
				setLeg_RAngularVelocity(body_angular_velocity);  //足Rの角速度
				const dReal *Leg_R_current_anglev = dBodyGetAngularVel(leg[1].body);
				double Leg_R_target_anglev = 0;
				double dtheta1_1 = Leg_R_target_anglev - Leg_R_current_anglev[1];

				//θ2を設定
				//上半身の角度
				setBodyAngle(body_angle);
				double body_target_angle = 0;
				double theta2 = body_target_angle - body_angle[1];

				angleSensor();

				hip_current_angle[0] = dJointGetHingeAngle(hip_joint[0]); //足の角度L
				hip_current_angle[1] = dJointGetHingeAngle(hip_joint[1]); //足の角度R

				ankle_current_angle[0] = dJointGetHingeAngle(ankle_joint[0]); //足先の角度L
				ankle_current_angle[1] = dJointGetHingeAngle(ankle_joint[1]); //足先の角度R

				hip_target_angle[0] = 3.0*M_PI / 180;       //足の角度の幅の目標値は3.0deg
				hip_target_angle[1] = -3.0*M_PI / 180;

				ankle_target_angle[0] = 60 * M_PI / 180;// ANKLE_MAX;// ANKLE_MIN; //足首
				ankle_target_angle[1] = -60 * M_PI / 180;// ANKLE_MAX;// ANKLE_MIN;

				double theta2_0 = ankle_target_angle[0] - ankle_current_angle[0]; //ankleL
				double theta2_1 = ankle_target_angle[1] - ankle_current_angle[1]; //ankleR

				//θ2[0],θ2[1]を設定
				double theta1_0 = hip_target_angle[0] - hip_current_angle[0]; //hipL
				double theta1_1 = hip_target_angle[1] - hip_current_angle[1]; //hipR

				//x=[θ1 θ2 θ1' θ2']にゲインK=[3.1623 12.4536 2.4839 2.3288]をかけたものを入力とする
				double K1 = 3.1623, K2 = 12.4356, K3 = 2.4839, K4 = 2.3288;
				double in_L = (theta1_0 * K1 + theta2 * K2 + dtheta1_0 * K3 + dtheta2 * K4);
				double in_R = (theta1_1 * K1 + theta2 * K2 + dtheta1_1 * K3 + dtheta2 * K4);  //3deg動かす入力

				double fmax = 20.0; //最大トルク

				//*fmaxすると力が強すぎるけど,1倍だと何も動かないのでとりあえず入力を3倍

				ankle_target_angle[1] = -60 * M_PI / 180 * 3;
				hip_target_angle[1] = in_R * M_PI / 180 * 3;
				ankle_target_angle[0] = 60 * M_PI / 180 * 3;

				//振り上げない足は動かない方が都合がいいのでとりあえず何もしない
				hip_target_angle[0] = in_L * M_PI / 180;
			}

			/*--------------------------------------------------------------------------------------------------------------------------------*/

			//逆足の場合でも同じことをする

			if (hip_current_angle[0] < hip_current_angle[1]
				&& flag == 2)

			{
				setBodyAngularVelocity(body_angular_velocity);  // 胴体の角速度を求める
				const dReal *Body_current_anglev = dBodyGetAngularVel(head.body);
				double Body_target_anglev = 0;
				double dtheta2 = Body_target_anglev - Body_current_anglev[1];

				setLeg_LAngularVelocity(body_angular_velocity);  //足Lの角速度
				const dReal *Leg_L_current_anglev = dBodyGetAngularVel(leg[0].body);
				double Leg_L_target_anglev = 0;
				double dtheta1_0 = Leg_L_target_anglev - Leg_L_current_anglev[1];

				setLeg_RAngularVelocity(body_angular_velocity);  //足Rの角速度
				const dReal *Leg_R_current_anglev = dBodyGetAngularVel(leg[1].body);
				double Leg_R_target_anglev = 0;
				double dtheta1_1 = Leg_R_target_anglev - Leg_R_current_anglev[1];

				//上半身の角度
				setBodyAngle(body_angle);
				double body_target_angle = 0;// 10.0 * M_PI / 180;//0
				double theta2 = body_target_angle - body_angle[1];


				angleSensor();

				hip_current_angle[0] = dJointGetHingeAngle(hip_joint[0]); //上半身の角度L
				hip_current_angle[1] = dJointGetHingeAngle(hip_joint[1]); //上半身の角度R

				ankle_current_angle[0] = dJointGetHingeAngle(ankle_joint[0]); //下半身の角度L
				ankle_current_angle[1] = dJointGetHingeAngle(ankle_joint[1]); //下半身の角度R

				hip_target_angle[0] = -3.0*M_PI / 180;       //上半身の角度の目標値は3.0deg
				hip_target_angle[1] = 3.0*M_PI / 180;

				ankle_target_angle[0] = -60 * M_PI / 180;
				ankle_target_angle[1] = 60 * M_PI / 180;

				double theta2_0 = ankle_target_angle[0] - ankle_current_angle[0]; //ankleL
				double theta2_1 = ankle_target_angle[1] - ankle_current_angle[1]; //ankleR

				double theta1_0 = hip_target_angle[0] - hip_current_angle[0]; //hipL
				double theta1_1 = hip_target_angle[1] - hip_current_angle[1]; //hipR

				double K1 = 3.1623, K2 = 12.4356, K3 = 2.4839, K4 = 2.3288;
				double in_L = (theta1_0 * K1 + theta2 * K2 + dtheta1_0 * K3 + dtheta2 * K4);
				double in_R = (theta1_1 * K1 + theta2 * K2 + dtheta1_1 * K3 + dtheta2 * K4);  //3.0deg動かす入力

				double fmax = 20.0; //トルク

				//同様に3倍

				ankle_target_angle[0] = -60 * M_PI / 180 * 3;
				//printf("if2\n");
				hip_target_angle[0] = in_L * M_PI / 180 * 3;
				ankle_target_angle[1] = 60 * M_PI / 180 * 3;
				hip_target_angle[1] = in_R * M_PI / 180;
			}
		}
	}
}

// control(P制御)
static void control()
{
	double k1 = 10.0;     // 比例定数
	double fMax = 20.0;   // 最大トルク

	for (int i = 0; i<NUM; i++) {
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

	//arm-box
	dsSetColor(1.0, 2.0, 0.2);  // 緑
	for (int i = 0; i<NUM; i++) {
		dsDrawBox(dBodyGetPosition(arm[i].body),
			dBodyGetRotation(arm[i].body), ARM_L);
	}
	// body - box
	dsSetColor(0.5, 0.5, 0.5);  // 灰
	dsDrawBox(dBodyGetPosition(body.body),
		dBodyGetRotation(body.body), BODY_L);

	// leg - capsule
	dsSetColor(0.0, 0.0, 1.0);  // 青
	for (int i = 0; i<NUM; i++) {
		dsDrawCapsule(dBodyGetPosition(leg[i].body),
			dBodyGetRotation(leg[i].body), leg[i].l, leg[i].r);
	}

	// foot - box
	dsSetColor(0.0, 0.0, 0.0);  // 黒　
	for (int i = 0; i<NUM; i++) {
		dsDrawBox(dBodyGetPosition(foot[i].body),
			dBodyGetRotation(foot[i].body), FOOT_L);
	}
}

// ロボットを創造する関数
void createRobot()
{
  head.r = 0.10;                    // 頭(head)の半径r
	head.m = 4.0;                     // 頭(head)の質量m
	body.m = 12.0;                    // 胴体(body)の質量m

	for (int i = 0; i<NUM; i++) {
		leg[i].r = 0.03;                // 脚(leg)の半径r
		leg[i].l = 0.59 - leg[i].r * 2; // 脚(leg)の長さl
		leg[i].m = 1.50;                // 脚(leg)の質量m
		arm[i].r = 0.03;                // (arm)の半径r
		arm[i].l = 0.59;                // 脚(arm)の長さl
		arm[i].m = 1.5;                // 脚(arm)の質量m
		foot[i].m = 0.50;                  // 足先(foot)の質量m
	}

	dMass mass;
	#define h 0.0		//	地面からの高さをここで設定する

	/***** MyObject *****/
	// 頭【球 - sphere】
	head.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, head.m, head.r);
	dBodySetMass(head.body, &mass);
	dBodySetPosition(head.body, 0.0, 0.0,
		FOOT_L[2] + leg[0].l + 2 * leg[0].r + BODY_L[2] + head.r + h);
	head.geom = dCreateSphere(space, head.r);
	dGeomSetBody(head.geom, head.body);

	// 腕【カプセル - capsule】
	for (int i = 0; i < NUM; i++) {
		arm[i].body = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, arm[i].m, 3, arm[i].r, arm[i].l);
		// 第三引数の3は長軸方向(1=x軸, 2=y軸, 3=z軸)
		dBodySetMass(arm[i].body, &mass);
		arm[i].geom = dCreateCapsule(space, arm[i].r, arm[i].l);
		dGeomSetBody(arm[i].geom, arm[i].body);
	}
	dBodySetPosition(arm[0].body, 0.0, -arm[1].r - BODY_L[1] / 2,
		FOOT_L[2] + leg[0].l + 2 * leg[0].r + BODY_L[2] / 2 + h);//重心の位置で決めている
	dBodySetPosition(arm[1].body, 0.0, arm[1].r + BODY_L[1] / 2,
		FOOT_L[2] + leg[0].l + 2 * leg[0].r + BODY_L[2] / 2 + h);//legはL＋2r

	// 胴体【直方体 - box】
	body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, body.m, BODY_L[0], BODY_L[1], BODY_L[2]);
	dBodySetMass(body.body, &mass);
	dBodySetPosition(body.body, 0.0, 0.0,
		FOOT_L[2] + leg[0].l + 2 * leg[0].r + BODY_L[2] / 2 + h);

	body.geom = dCreateBox(space, BODY_L[0], BODY_L[1], BODY_L[2]);
	dGeomSetBody(body.geom, body.body);

	// 脚【カプセル - capsule】
	for (int i = 0; i<NUM; i++) {
		leg[i].body = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, leg[i].m, 3, leg[i].r, leg[i].l);

		// 第三引数の3は長軸方向(1=x軸, 2=y軸, 3=z軸)
		dBodySetMass(leg[i].body, &mass);
		leg[i].geom = dCreateCapsule(space, leg[i].r, leg[i].l);
		dGeomSetBody(leg[i].geom, leg[i].body);
	}
	dBodySetPosition(leg[0].body, 0.0, -BODY_L[1] / 4,
		FOOT_L[2] + leg[0].r + leg[0].l / 2 + h);
	dBodySetPosition(leg[1].body, 0.0, BODY_L[1] / 4,
		FOOT_L[2] + leg[0].r + leg[1].l / 2 + h);

	// 足先【直方体 - box】
	for (int i = 0; i<NUM; i++) {
		foot[i].body = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetBoxTotal(&mass, foot[i].m, FOOT_L[0], FOOT_L[1], FOOT_L[2]);
		dBodySetMass(foot[i].body, &mass);
		foot[i].geom = dCreateBox(space, FOOT_L[0], FOOT_L[1], FOOT_L[2]);
		dGeomSetBody(foot[i].geom, foot[i].body);
	}
	dBodySetPosition(foot[0].body, 0.05, -BODY_L[1] / 4, FOOT_L[2] / 2 + h);
	dBodySetPosition(foot[1].body, 0.05, BODY_L[1] / 4, FOOT_L[2] / 2 + h);

	/***** ジョイント *****/
	// head - body
	neck_joint = dJointCreateFixed(world, 0);         // Fixed:固定ジョイント
	dJointAttach(neck_joint, head.body, body.body);
	dJointSetFixed(neck_joint);

	// body - arm
	for (int i = 0; i < NUM; i++) {
		shol_joint[i] = dJointCreateHinge(world, 0);     // Hinge: 蝶番ジョイント
		dJointAttach(shol_joint[i], body.body, arm[i].body);
		dJointSetHingeAxis(shol_joint[i], 0, 1, 0);
		dJointSetHingeParam(shol_joint[i], dParamLoStop, -M_PI / 3);
		dJointSetHingeParam(shol_joint[i], dParamHiStop, M_PI / 3);
	}
	dJointSetHingeAnchor(shol_joint[0], 0.0, -BODY_L[0] / 4, leg[0].l + arm[0].l + h);
	dJointSetHingeAnchor(shol_joint[1], 0.0, BODY_L[0] / 4, leg[1].l + arm[1].l + h);

	// body - leg
	for (int i = 0; i<NUM; i++) {
		hip_joint[i] = dJointCreateHinge(world, 0);     // Hinge: 蝶番ジョイント
		dJointAttach(hip_joint[i], body.body, leg[i].body);
		dJointSetHingeAxis(hip_joint[i], 0, 1, 0);
		dJointSetHingeParam(hip_joint[i], dParamLoStop, -M_PI / 3);
		dJointSetHingeParam(hip_joint[i], dParamHiStop, M_PI / 3);
	}
	dJointSetHingeAnchor(hip_joint[0], 0.0, -BODY_L[1] / 4,
		FOOT_L[2] + leg[0].l + h);
	dJointSetHingeAnchor(hip_joint[1], 0.0, BODY_L[1] / 4,
		FOOT_L[2] + leg[1].l + h);

	// leg - foot
	for (int i = 0; i<NUM; i++) {
		ankle_joint[i] = dJointCreateHinge(world, 0);     // Hinge: 蝶番ジョイント
		dJointAttach(ankle_joint[i], leg[i].body, foot[i].body);
		dJointSetHingeAxis(ankle_joint[i], 0, 1, 0);
		dJointSetHingeParam(ankle_joint[i], dParamLoStop, -M_PI / 3);
		dJointSetHingeParam(ankle_joint[i], dParamHiStop, M_PI / 3);
	}
	dJointSetHingeAnchor(ankle_joint[0], 0.0, -BODY_L[1] / 4, FOOT_L[2] / 4 + h);
	dJointSetHingeAnchor(ankle_joint[1], 0.0, BODY_L[1] / 4, FOOT_L[2] / 4 + h);
}

// シミュレーションループ
void simLoop(int pause)
{
	if (!pause) {
		dSpaceCollide(space, 0, &nearCallback);   // 衝突検出関数
		dWorldStep(world, 0.005);                  // シミュレーションを1ステップ進める
		dJointGroupEmpty(contactgroup);           // ジョイントグループを空にする
		balance();        // 立ち続ける関数
		control();        // 動作を制御する関数（commandでジョイントの角度を変更する）
		angleSensor();
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

	for (int i = 0; i<NUM; i++) {
		dJointDestroy(hip_joint[i]);
		dJointDestroy(ankle_joint[i]);
		dJointDestroy(shol_joint[i]);
		dBodyDestroy(leg[i].body);
		dBodyDestroy(foot[i].body);
		dBodyDestroy(arm[i].body);
		dGeomDestroy(leg[i].geom);
		dGeomDestroy(foot[i].geom);
		dGeomDestroy(arm[i].geom);
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
	shol_target_angre[0] = 0.0, shol_target_angre[1] = 0.0;
	space_trigger = false;
	stand_flag = false;
	createRobot();
}

// キー入力を処理する関数
static void command(int cmd)
{
	switch (cmd)
	{
		case 'a': // temp debug
			hip_target_angle[0] = HIP_MAX / 6;
			hip_target_angle[1] = HIP_MAX / 6;
			ankle_target_angle[0] = ANKLE_MIN * 2 / 3;
			ankle_target_angle[1] = ANKLE_MIN * 2 / 3;
			break;
		case 'r':
			printf("restart\n");
			restart();
			break;
		case 'o':
			cogSensor(tpos);
			printf("重心(%f, %f, %f)\n", tpos[0], tpos[1], tpos[2]);
			break;
		case 'b': // temp debug
			setBodyAngle(body_angle);
			printf("胴体回転:%f %f %f\n", body_angle[0], body_angle[1], body_angle[2]);
			break;
		case 'v': // temp debug
			setBodyAngularVelocity(body_angular_velocity);
			printf("胴体角速度:%f %f %f\n", body_angular_velocity[0], body_angular_velocity[1], body_angular_velocity[2]);
			break;
		case 'm': // temp debug
			setLeg_LAngularVelocity(body_angular_velocity);
			printf("足角速度:%f %f %f\n", body_angular_velocity[0], body_angular_velocity[1], body_angular_velocity[2]);
			break;
		case 'n': // temp debug
			setLeg_RAngularVelocity(body_angular_velocity);
			printf("足角速度:%f %f %f\n", body_angular_velocity[0], body_angular_velocity[1], body_angular_velocity[2]);
			break;
		case ' ':
			printf("space\n");
			space_trigger = !space_trigger;
			break;
		default:
			printf("cannot use this button\n");
			break;
	}
	// 可動域を超えないようにする
	checkAngleRange();
}

// 前処理
void start()
{
	static float xyz[3] = { -1.0, 2.0, 0.8 };     // 視点の位置
	static float hpr[3] = { -90.0, 0.0, 0.0 };   // 視点の方向
	dsSetViewpoint(xyz, hpr);                  // カメラを設定
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
	fn.path_to_textures = "../textures";
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
