#include "stdafx.h"

#include <stdio.h>
#include <gl/glut.h>
#include <PxPhysicsAPI.h>

#include <cstdio>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>

#include "Targa.h"

#pragma comment(lib, "PhysX3_x86.lib")
#pragma comment(lib, "PhysX3Common_x86.lib") 
#pragma comment(lib, "PhysX3Extensions.lib")
#pragma comment(lib, "PhysXProfileSDKCHECKED.lib")

using namespace physx;

#define ANIM_FPS	240		
#define SIM_TIME	1/240.0
//Ilość kulek
#define NUM_BALLS	7
//Rozmiar kulki
#define BALL_SIZE	0.8f

PxFoundation *pfund;
PxPhysics* pphys;
PxScene* gsc;
PxVec3 gg(0, -9.81, 0);
PxVec3 force(0.0, 0.0, 0.0);

PxDefaultErrorCallback gDefaultErrorCallback;
PxDefaultAllocator gDefaultAllocatorCallback;

PxRigidStatic* a_plane;
PxRigidDynamic* balls[NUM_BALLS];
PxRigidStatic* hooks[NUM_BALLS];
PxDistanceJoint* joints[NUM_BALLS];

int balls_to_move = 3;

//Funkcja tworzy dynamiczną kulkę
PxRigidDynamic* make_ball(PxMaterial *mat, float x, float y, float z) {
	//Pozycja i geometria obiektu
	PxTransform pose(PxVec3(x, y, z));
	PxSphereGeometry gball(BALL_SIZE);

	//Utworzenie obiektu
	PxRigidDynamic *actor = PxCreateDynamic(*pphys, pose, gball, *mat, 10.0f);

	//Dodanie do sceny
	gsc->addActor(*actor);

	return(actor);
}

//Funkcja tworzy statyczny uchwyt
PxRigidStatic* make_hook(PxMaterial *mat, float x, float y, float z){
	//Pozycja i geometria obiektu
	PxTransform pose(PxVec3(x, y, z));
	PxBoxGeometry gbox(BALL_SIZE / 2, BALL_SIZE / 2, BALL_SIZE / 2);

	//Utworzenie statycznego obiektu
	PxRigidStatic *hook = pphys->createRigidStatic(pose);
	PxShape* shape = hook->createShape(gbox, *mat);

	//Dodanie do sceny
	gsc->addActor(*hook);

	return(hook);
}

//Utworzenie sznurka (przegubu) pomiędzy kulką a uchwytem
PxDistanceJoint *make_joint(PxRigidStatic *hook, PxRigidDynamic *ball) {
	//Utworzenie przegubu pomiędzy dwoma obiektami
	PxDistanceJoint *joint = PxDistanceJointCreate(*pphys,
		hook, PxTransform(PxVec3(0, 0, 0)),
		ball, PxTransform(PxVec3(0, 0, 0)));

	//Ustawienie minimalnej i maksymalnej długości sznurka
	joint->setMinDistance(0.5);
	joint->setMaxDistance(5.0);

	joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
	joint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);

	return(joint);
}

//Funkcja tworzy statyczną podłogę
PxRigidStatic* make_plane(PxMaterial *mat) {
	PxTransform pose = PxTransform(PxVec3(0, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));

	PxRigidStatic *plane = pphys->createRigidStatic(pose);
	PxShape* shape = plane->createShape(PxPlaneGeometry(), *mat);

	gsc->addActor(*plane);
	return(plane);
}

//Inicjalizacja physxa
void init_physx() {
	pfund = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	if (!pfund) {
		puts("PxCreateFoundation kaput!");
		exit(0);
	}

	pphys = PxCreatePhysics(PX_PHYSICS_VERSION, *pfund, PxTolerancesScale());
	if (!pphys) {
		puts("PxCreatePhysics kaput!");
		exit(0);
	}

	PxSceneDesc sceneDesc(pphys->getTolerancesScale());
	sceneDesc.gravity = gg;

	PxDefaultCpuDispatcher *cpudisp = PxDefaultCpuDispatcherCreate(1);
	if (!cpudisp) {
		puts("PxDefaultCpuDispatcherCreate kaput!");
		exit(0);
	}
	sceneDesc.cpuDispatcher = cpudisp;

	sceneDesc.filterShader = &PxDefaultSimulationFilterShader;

	gsc = pphys->createScene(sceneDesc);
	if (!gsc) {
		puts("createScene kaput!");
		exit(0);
	}

	//Utworzenie materiału
	PxMaterial *mat = pphys->createMaterial(0, 0, 1.0f);
	a_plane = make_plane(mat);

	//Utworzenie obiektów (kulek, uchwytów i połączeń)
	float offset = -NUM_BALLS * BALL_SIZE / 2.0;
	for (int i = 0; i < NUM_BALLS; i++){
		hooks[i] = make_hook(mat, offset + i * BALL_SIZE * 2.15, BALL_SIZE * 8, 0);
		balls[i] = make_ball(mat, offset + i * BALL_SIZE * 2.15, BALL_SIZE * 2, 0);
		joints[i] = make_joint(hooks[i], balls[i]);
	}
}

void kill_physx() {
	if (pphys != NULL) {
		if (gsc != NULL)		gsc->release();
		pphys->release();
	}
	if (pfund != NULL)	pfund->release();
}

void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
	mat[0] = m.column0[0];	mat[1] = m.column0[1];	mat[2] = m.column0[2]; mat[3] = 0;

	mat[4] = m.column1[0];	mat[5] = m.column1[1];	mat[6] = m.column1[2];	mat[7] = 0;

	mat[8] = m.column2[0];	mat[9] = m.column2[1];	mat[10] = m.column2[2]; mat[11] = 0;

	mat[12] = t[0]; mat[13] = t[1];	mat[14] = t[2];	mat[15] = 1;
}

void SetupGLMatrix(const PxTransform &pose) {
	float glmat[16];

	PxMat33 m = PxMat33(pose.q);
	getColumnMajor(m, pose.p, glmat);

	glMultMatrixf(&(glmat[0]));
}

void SetupGLMatrix(const PxTransform &pose);

void draw_balls() {
	for (int i = 0; i < NUM_BALLS; i++) {
		PxTransform pose = hooks[i]->getGlobalPose();
		PxTransform pose2 = balls[i]->getGlobalPose();

		//Uchwyty
		glPushMatrix();
		glColor3f(1, 0, 0);
		SetupGLMatrix(pose);

		glPushMatrix();
		glutSolidCube(BALL_SIZE / 2);
		glPopMatrix();
		glPopMatrix();

		//Kulki
		glPushMatrix();
		glColor3f(1, 0, 0);
		SetupGLMatrix(pose2);

		glPushMatrix();
		glutSolidSphere(BALL_SIZE, 16, 16);
		glPopMatrix();
		glPopMatrix();

		//Połączenia
		glPushMatrix();
		PxVec3 pos1 = pose.transform(PxVec3(0, 0, 0));
		PxVec3 pos2 = pose2.transform(PxVec3(0, 0, 0));
		glBegin(GL_LINES);
		glVertex3f(pos1.x, pos1.y, pos1.z);
		glVertex3f(pos2.x, pos2.y, pos2.z);
		glEnd();
		glPopMatrix();
	}
}

void RenderScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	//Oświetlenie
	GLfloat lightAmb[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lightDif[] = { 0.1, 0.1, 0.1, 1.0 };
	GLfloat lightPos[] = { -30, 100, 100, 0 };
	GLfloat lightSpec[] = { 1, 1, 1, 1 };

	GLint shininess = 64;

	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glMaterialfv(GL_FRONT, GL_AMBIENT, lightAmb);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, lightDif);
	glMaterialfv(GL_FRONT, GL_SPECULAR, lightSpec);
	glMateriali(GL_FRONT, GL_SHININESS, shininess);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	//Kamera
	gluLookAt(10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 0, 1, 0);

	//Pobranie wyników symulacji
	gsc->fetchResults(true);

	//Narysowanie podłogi
	glColor3f(1, 1, 1);
	int floor_size = 10;
	glBegin(GL_QUADS);
	glVertex3f(-floor_size, 0, -floor_size);
	glVertex3f(-floor_size, 0, floor_size);
	glVertex3f(floor_size, 0, floor_size);
	glVertex3f(floor_size, 0, -floor_size);
	glEnd();

	//Narysowanie kulek
	draw_balls();

	//Przejście do następnego kroku symulacji
	gsc->simulate(SIM_TIME);

	glutSwapBuffers();
}

void ChangeSize(int w, int h) {
	if (h == 0)	h = 1;

	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40, (float)w / h, 1, 100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void ZegarFun(int val) {
	glutPostRedisplay();

	glutTimerFunc(1000 / ANIM_FPS, ZegarFun, 0);
}

void KeyFunUp(unsigned char key, int x, int y) {
	//Jeśli naciśnięte 'a' to przyłóż siłę do odpowiedniej ilości kulek
	if (key == 'a'){
		for (int i = balls_to_move; i > 0; i--)
			balls[NUM_BALLS - i]->addForce(PxVec3(50.0, 0.0, 0.0), PxForceMode::eIMPULSE);
	}
}

void KeyFunDown(unsigned char key, int x, int y) {
	if (key == 0x1B)	exit(0);
}

int main(int argc, wchar_t* argv[]) {
	glutInit(&argc, (char**)argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutCreateWindow("Wachadlo newtona");

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_ALPHA_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);

	glShadeModel(GL_SMOOTH);

	srand(time(NULL));

	init_physx();

	glutDisplayFunc(RenderScene);
	glutReshapeFunc(ChangeSize);
	glutKeyboardUpFunc(KeyFunUp);
	glutKeyboardFunc(KeyFunDown);

	glutTimerFunc(1000 / ANIM_FPS, ZegarFun, 0);
	glutMainLoop();

	kill_physx();
	return(0);
}

