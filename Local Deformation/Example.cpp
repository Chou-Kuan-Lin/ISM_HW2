#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <Windows.h>
#include <gl/GL.h>
#include <glut.h>

#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"

#include <Eigen/Sparse>
#include <Eigen/Core>
#include <time.h>

using namespace std;
using namespace Eigen;

_GLMmodel *mesh;
int WindWidth, WindHeight;

int last_x, last_y;
int selectedFeature = -1;
vector<int> featureList;

vector<int> selectControlPoints;	//����I
const int selectCount = 1000;	//����I�ƶq
int randomNumber = 0;	//�p��
bool check = false;	//�ˬd����I�O�_���ơB�s��(false���L���ơB�L�s���Ftrue�����ơB���s��)
vector<Triplet<double>> solver;	//The linear system
vector<Triplet<double>> B;
vector<int> connectControlPoints;	//�s���I
clock_t t1, t2;	//�p�ɾ�

void Reshape(int width, int height)
{
	int base = min(width, height);

	tbReshape(width, height);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLdouble)width / (GLdouble)height, 1.0, 128.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -3.5);

	WindWidth = width;
	WindHeight = height;
}

void Display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	tbMatrix();

	// render solid model
	glEnable(GL_LIGHTING);
	glColor3f(1.0, 1.0, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glmDraw(mesh, GLM_SMOOTH);

	// render wire model
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glLineWidth(1.0f);
	glColor3f(0.6, 0.0, 0.8);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glmDraw(mesh, GLM_SMOOTH);

	// render features
	glPointSize(10.0);
	glColor3f(1.0, 0.0, 0.0);
	glDisable(GL_LIGHTING);
	glBegin(GL_POINTS);
	for (int i = 0; i < featureList.size(); i++)
	{
		int idx = featureList[i];

		glVertex3fv((float *)&mesh->vertices[3 * idx]);
	}
	glEnd();

	glPopMatrix();

	glFlush();
	glutSwapBuffers();
}

vector3 Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];				//Model_view matrix
	double ProjectionMatrix[16];			//Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x, viewport[3] - (int)_2Dpos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = { 0.0 , 0.0 , 0.0 };

	gluUnProject(X, ((double)viewport[3] - Y), (double)Depth, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

	return vector3(wpos[0], wpos[1], wpos[2]);
}

void mouse(int button, int state, int x, int y)
{
	tbMouse(button, state, x, y);

	// add feature
	/*if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
	{
		int minIdx = 0;
		float minDis = 9999999.0f;

		vector3 pos = Unprojection(vector2((float)x , (float)y));

		for (int i = 0 ; i < mesh->numvertices ; i++)
		{
			vector3 pt(mesh->vertices[3 * i + 0] , mesh->vertices[3 * i + 1] , mesh->vertices[3 * i + 2]);
			float dis = (pos - pt).length();

			if (minDis > dis)
			{
				minDis = dis;
				minIdx = i;
			}
		}

		featureList.push_back(minIdx);
	}*/

	// manipulate feature
	/*if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		int minIdx = 0;
		float minDis = 9999999.0f;

		vector3 pos = Unprojection(vector2((float)x , (float)y));

		for (int i = 0 ; i < featureList.size() ; i++)
		{
			int idx = featureList[i];
			vector3 pt(mesh->vertices[3 * idx + 0] , mesh->vertices[3 * idx + 1] , mesh->vertices[3 * idx + 2]);
			float dis = (pos - pt).length();

			if (minDis > dis)
			{
				minDis = dis;
				minIdx = featureList[i];
			}
		}

		selectedFeature = minIdx;
	}*/

	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
		selectedFeature = -1;

	last_x = x;
	last_y = y;
}

void motion(int x, int y)
{
	tbMotion(x, y);

	if (selectedFeature != -1)
	{
		matrix44 m;
		vector4 vec = vector4((float)(x - last_x) / 100.0f, (float)(y - last_y) / 100.0f, 0.0, 1.0);

		gettbMatrix((float *)&m);
		vec = m * vec;

		mesh->vertices[3 * selectedFeature + 0] += vec.x;
		mesh->vertices[3 * selectedFeature + 1] -= vec.y;
		mesh->vertices[3 * selectedFeature + 2] += vec.z;
	}

	last_x = x;
	last_y = y;
}

void timf(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1, timf, 0);
}

int main(int argc, char *argv[])
{
	WindWidth = 400;
	WindHeight = 400;

	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };

	glutInit(&argc, argv);
	glutInitWindowSize(WindWidth, WindHeight);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("Least Squares Meshes Example");

	glutReshapeFunc(Reshape);
	glutDisplayFunc(Display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glClearColor(0, 0, 0, 0);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	tbInit(GLUT_LEFT_BUTTON);
	tbAnimate(GL_TRUE);

	glutTimerFunc(40, timf, 0); // Set up timer for 40ms, about 25 fps

	// load 3D model
	mesh = glmReadOBJ("../data/head.obj");

	glmUnitize(mesh);
	glmFacetNormals(mesh);
	glmVertexNormals(mesh, 90.0);

	//����I
	  //printf("%i", mesh->numvertices);	//10002
	  //printf("%i", mesh->numtriangles);	//20000
	srand(time(NULL));
	do
	{
		randomNumber = rand() % mesh->numvertices ;	//�üƲ���

		//�ˬd�O�_����
		for (int a = 0; a < selectControlPoints.size(); a++)
			if (selectControlPoints[a] == randomNumber)
			{
				check = true;
				break;
			}

		if (check == false)
			selectControlPoints.push_back(randomNumber);

		check = false;
	} while (selectControlPoints.size() != selectCount);

	//Timer
	t1 = clock();

	//�Ыدx�}
	//compute L Matrix
	for (int a = 0; a < mesh->numvertices; a++)
	{
		//�M��a���s���I
		//�d��T����b
		for (int b = 0; b < mesh->numtriangles; b++)
		{
			check = false;
			//�p�G�T����b��vindices��@��a�A�h�t�~���vindices�Pa�۳s
			for (int c = 0; c < 3; c++)
				if (mesh->triangles[b].vindices[c]-1 == a)
					check = true;

			//�p�G�T����b�Pa���s��
			if (check == true)
			{
				//�N�T����b��vindices��JconnectControlPoints�}�C��
				check = false;
				for (int c = 0; c < 3; c++)
				{
					//�ˬdconnectControlPoints�}�C���O�_�w�g���T����b��vindices
					for (int d = 0; d < connectControlPoints.size(); d++)
						if (mesh->triangles[b].vindices[c]-1 == connectControlPoints[d])
						{
							check = true;
							break;
						}

					//connectControlPoints�}�C���S���T����b��vindices�h��J
					if (check == false)
						connectControlPoints.push_back(mesh->triangles[b].vindices[c]-1);
				}
			}//�p�G�T����b�Pa���s��
		}//�d��T����b
		//�o�Ia���s���C��
		
		for (int b = 0; b < connectControlPoints.size(); b++)
			//�ۤv��1
			if (a == connectControlPoints[b])
				solver.push_back(Triplet<double>(a, a, 1));

			//�P�I���s������(1/x)(x=�s���I�ƶq)
			//size�n�����ۤv
			else
				solver.push_back(Triplet<double>(a, connectControlPoints[b], -pow((connectControlPoints.size() - 1), (-1))));
		//�P�I�L�s������0	

		//�M��connectCountrolPoints�}�C���U�@�I�ϥ�
		connectControlPoints.clear();
	}

	//�Ыدx�}2
	//�ۤv��1
	for (int a = 0;a < selectCount;a++)
		solver.push_back(Triplet<double>((mesh->numvertices + a), selectControlPoints[a], 1));

	//�Ыدx�}3
	//Compute C Matrix for vertices x, y, z axis
	VectorXd C((mesh->numvertices + selectCount) , 3);
	//Vi��0
	for (int a = 0;a < mesh->numvertices;a++)
		for (int b = 0;b < 3; b++)
			C(a , b) = 0;

	//Ci��1
	for (int a = 0; a < selectCount; a++)
		for (int b = 0;b < 3; b++) {
			C((mesh->numvertices + a), b) = mesh->vertices[(selectControlPoints[a] + 1) * 3 + b];
			B.push_back(Triplet<double>((mesh->numvertices + a), b, mesh->vertices[(selectControlPoints[a] + 1) * 3 + b]));
		}
	SparseMatrix<double> A(mesh->numvertices + selectCount, mesh->numvertices);
	A.setFromTriplets(solver.begin(), solver.end());

	//Convert into ATA=ATb
	//SparseMatrix<double> ATA = A.transpose()*A;
	//C = A.transpose()*C;
	
	SparseMatrix<double> Bx(mesh->numvertices + selectCount,3);
	Bx.setFromTriplets(B.begin(), B.end());
	Bx = A.transpose()*Bx;

	SimplicialCholesky<SparseMatrix<double>> chol(A.transpose()*A);
	MatrixXd result = chol.solve(Bx);

	//���ؼҲ�
	float error = 0;
	float temp = 0;
	for (int i = 0;i < mesh->numvertices;i++)
		for (int j = 0;j < 3;j++)
		{
			temp = mesh->vertices[(i + 1) * 3 + j];	//�쥻���I
			mesh->vertices[(i + 1) * 3 + j] = result(i, j);	//�s���I

			//Error
			error += pow(mesh->vertices[(i + 1) * 3 + j] - temp, 2);
		}

	error = pow((error / mesh->numvertices), 0.5);
	printf("Error : %f\n", error);

	//Time
	t2 = clock();
	
	printf("Time : %lf\n", (t2 - t1) / (double)(CLOCKS_PER_SEC));
	
	glutMainLoop();	//�`���I

	return 0;

}

