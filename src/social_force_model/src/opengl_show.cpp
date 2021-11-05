#include <iostream>
#include <fstream>
#include <GL/glut.h>
#include <vector>
#include <string>
#include "control.h"
#include "map.h"

using namespace std;

string game_txt = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/game.txt";
string map_txt = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/map.txt";
string ped_txt = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/ped.txt";
string vehicle_txt = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/vehicle.txt";
string light_txt = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/lights.txt";

GLsizei winWidth = 1600;
GLsizei winHeight = 900;
Environment *environment;
Control *control;
float fps = 0;
bool act = false;

void init();
void loadGameMatrix();
void loadMap();
void loadVehicle();
void loadPed();
void loadLight();

void display();

void drawMap();
void drawCrowd();
void drawCircle(float x, float y, float z, float r, Color color, int slices = 90);
void drawRectangle(float x, float y, float z, float length, float width, Color color, int fill);				//fill=0 不填充 fill=1 填充
void drawVehicle();
void drawLight();

void showInformation();
void drawText(float x, float y, char text[]);
void reshape(int width, int height);
void normalKey(unsigned char key, int xMousePos, int yMousePos);
void update();
void computeFPS();



int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(winWidth, winHeight);
	glutInitWindowPosition(200, 100);
	glutCreateWindow("Social Force Model");
	init();
	
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(normalKey);
	glutIdleFunc(update);
	glutMainLoop();
	return 1;
}

void init()				//初始化opengl
{
	GLfloat gnrlAmbient[] = { 0.8F, 0.8F, 0.8F, 1.0 };	//一般光强度
	GLfloat lghtDiffuse[] = { 0.7F, 0.7F, 0.7F, 1.0 };	//物体光强度
	GLfloat lghtPosition[] = { 4.0, -4.0, 4.0, 0.0 };	//灯光位置

	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_SMOOTH);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, gnrlAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lghtDiffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, lghtPosition);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);	
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);	

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);

	srand(1604010629);

	//读取文件
	environment = new Environment;
	control = new Control;
	loadGameMatrix();
	loadMap();
	loadPed();
	loadVehicle();
	loadLight();
}

void loadGameMatrix()
{
	ifstream game_file;
	string line;
	Evolution *evolution;
	
	game_file.open(game_txt.c_str());
	if(!game_file)
		cout<<"open game file failed!"<<endl;
	while(game_file.good())
	{
		double J, N, A, B;
		getline(game_file,line);
		if (line.length() == 0)
			break;
		std::stringstream ss(line);
		ss >> J >> N >> A >> B;
		evolution = new Evolution;
		evolution->setParam(J, N, A, B);
		control->addEvolution(evolution);
	}
	game_file.close();
	cout<<"game matrix loaded!"<<endl;
}

void loadMap()
{
	ifstream map_file;
	string line;
	
	Wall *wall;
	Seperator *sep;
	Zebra *zebra;
	
	map_file.open(map_txt.c_str());
	if(!map_file)
		cout<<"open map file failed!"<<endl;
	while(map_file.good())
	{
		string type;
		float x1, y1, x2, y2;
		int yaw;
		getline(map_file, line);
		if (line.length() == 0)
			break;
			
		std::stringstream ss(line);
		ss>>type;
		if (type == "wall")
		{
			ss >> x1 >> y1 >> x2 >> y2;
			wall = new Wall(x1, y1, x2, y2, 1.0);
			environment->addWall(wall);
		}
		else if (type == "sep")
		{
			ss >> x1 >> y1 >> x2 >> y2;
			sep = new Seperator(x1, y1, x2, y2);
			environment->addSep(sep);
		}
		else if (type == "zebra")
		{
			ss >> yaw >> x1 >> y1;
			zebra = new Zebra(x1, y1, yaw);
			environment->addZebra(zebra);
		}
	}
	cout<<"map loaded!"<<endl;
	control->getEnvironment(environment);
	map_file.close();
}

void loadPed()
{
	ifstream ped_file;
	string line;
	Pedestrian *ped;
	
	ped_file.open(ped_txt.c_str());
	if(!ped_file)
		cout<<"open ped file failed!"<<endl;
	while(ped_file.good())
	{
		float x1, y1, x2, y2;
		int g_id;
		getline(ped_file, line);
		if (line.length() == 0)
			break;
		ped = new Pedestrian;
		std::stringstream ss(line);
		ss >> g_id >> x1 >> y1 >> x2 >> y2;
		ped->setGroupId(g_id);
		ped->setPosition(randomFloat(x1-1.0, x1+1.0), randomFloat(y1-1.0, y1+1.0));
		ped->addPath(randomFloat(x2-1.0, x2+1.0), randomFloat(y2-1.0, y2+1.0));
		control->addPed(ped);
	}
	cout<<"ped loaded!"<<endl;
	ped_file.close();
}

void loadVehicle()
{
	ifstream car_file;
	string line;
	Vehicle *car;
	
	car_file.open(vehicle_txt.c_str());
	if(!car_file)
		cout<<"open vehicle file failed!"<<endl;
	while(car_file.good())
	{
		float x1, y1, x2, y2, speed;
		getline(car_file, line);
		if (line.length() == 0)
			break;
		car = new Vehicle;
		std::stringstream ss(line);
		ss >> x1 >> y1 >> x2 >> y2 >> speed;
		car->setPosition(x1, y1, -0.01F);
		car->setSpeed(speed);
		car->addPath(x2, y2);
		control->addCar(car);
	}
	cout<<"vehicle loaded!"<<endl;
	car_file.close();
}

void loadLight()
{
	ifstream light_file;
	string line;
	Traffic_light *light;
	
	light_file.open(light_txt.c_str());
	if(!light_file)
		cout<<"open light file failed!"<<endl;
	while(light_file.good())
	{
		float x1, y1, time, r;
		int color;
		getline(light_file, line);
		if (line.length() == 0)
			break;
		
		std::stringstream ss(line);
		ss >> x1 >> y1 >> r >> color >> time;
		light = new Traffic_light(x1, y1, r, color, time);
		control->addLight(light);
	}
	cout<<"lights loaded!"<<endl;
	light_file.close();
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	gluLookAt(0.0, 0.0, 35.0,		//相机位置
			  0.0, 0.0, 0.0,		//相机镜头方向对准物体在世界坐标位置
			  0.0, 1.0, 0.0);		//镜头向上方向在世界坐标的方向

	glPushMatrix();
	glScalef(1.0, 1.0, 1.0);
	
	//画图
	drawMap();
	drawCrowd();
	drawVehicle();
	drawLight();

	glPopMatrix();
	showInformation();
	glutSwapBuffers();
}

void drawMap()
{
	//wall
	std::vector<Wall *> walls = environment->getWalls();
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();

	for (Wall *wall : walls)
	{	
		glBegin(GL_LINES);
			glVertex3f(wall->getStartPoint().x, wall->getStartPoint().y,-0.02);
			glVertex3f(wall->getEndPoint().x, wall->getEndPoint().y,-0.02);
		glEnd();
	}
	glPopMatrix();

	//sep
	vector<Seperator *> seps = environment->getSeps();
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(1, 0X00FF);
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	for (Seperator *sep : seps) {
		glBegin(GL_LINES);
		glVertex3f(sep->getStartPoint().x, sep->getStartPoint().y, -0.02);
		glVertex3f(sep->getEndPoint().x, sep->getEndPoint().y, -0.02);
		glEnd();
	}
	glDisable(GL_LINE_STIPPLE);
	glPopMatrix();
	
	//zebra
	vector<Zebra *> zebras = environment->getZebras();
	
	for (Zebra *zebra : zebras)
	{
		if (zebra->getYaw() == 0)
		{
			drawRectangle(zebra->getPosition().x, zebra->getPosition().y, zebra->getPosition().z, zebra->getLength(), zebra->getWidth(), zebra->getColor(), 1);
			for (int i=1; i<=zebra->getNum(); ++i)
			{
				drawRectangle(zebra->getPosition().x, zebra->getPosition().y + zebra->getInterval() * 2 * i, zebra->getPosition().z, zebra->getLength(), zebra->getWidth(), zebra->getColor(), 1);
				drawRectangle(zebra->getPosition().x, zebra->getPosition().y - zebra->getInterval() * 2 * i, zebra->getPosition().z, zebra->getLength(), zebra->getWidth(), zebra->getColor(), 1);
			}
		}
		else if (zebra->getYaw() == 1)
		{
			drawRectangle(zebra->getPosition().x, zebra->getPosition().y, zebra->getPosition().z, zebra->getWidth(), zebra->getLength(), zebra->getColor(), 1);
			for (int i=1; i<=zebra->getNum(); ++i)
			{
				drawRectangle(zebra->getPosition().x - zebra->getInterval() * 2 * i, zebra->getPosition().y, zebra->getPosition().z, zebra->getWidth(), zebra->getLength(), zebra->getColor(), 1);
				drawRectangle(zebra->getPosition().x + zebra->getInterval() * 2 * i, zebra->getPosition().y, zebra->getPosition().z, zebra->getWidth(), zebra->getLength(), zebra->getColor(), 1);
			}
		}
	}
}

void drawCrowd()
{
	vector<Pedestrian *> crowds = control->getCrowd();
	for (Pedestrian *ped : crowds)
		drawCircle(ped->getPosition().x, ped->getPosition().y, ped->getPosition().z, ped->getRadius(), ped->getColor());
}

void drawCircle(float x, float y, float z, float r, Color color, int slices)
{
	float sliceAngle;
	Point current, next;

	glPushMatrix();	
	glColor3f(color.r,color.g,color.b);	
	glTranslatef(x, y, z);		 
	sliceAngle = static_cast<float>(360.0 / slices);		
	current.x = r;	
	current.y = 0;
	current.z = 0.0f;
	next.z = 0.0f;
	for (float angle = sliceAngle; angle <= 360; angle += sliceAngle) {
		next.x = r * cos(angle * PI / 180);
		next.y = r * sin(angle * PI / 180);  

		glBegin(GL_TRIANGLES);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(current.x, current.y, current.z);
			glVertex3f(next.x, next.y, next.z);
		glEnd();

		current = next;
	}
	glPopMatrix();
}

void drawRectangle(float x, float y, float z, float length, float width, Color color, int fill)
{
	glPushMatrix();
	glColor3f(color.r,color.g,color.b);
	if (fill == 0)
	{
		glBegin(GL_LINE_LOOP);
		glVertex3f(x - length / 2, y - width / 2, 0.0);
		glVertex3f(x + length / 2, y - width / 2, 0.0);
		glVertex3f(x + length / 2, y + width / 2, 0.0);
		glVertex3f(x - length / 2, y + width / 2, 0.0);
		glEnd();
	}
	else if (fill == 1)
	{
		glBegin(GL_QUADS);
		glVertex3f(x - length / 2, y - width / 2, z);
		glVertex3f(x + length / 2, y - width / 2, z);
		glVertex3f(x + length / 2, y + width / 2, z);
		glVertex3f(x - length / 2, y + width / 2, z);
		glEnd();
	}
	glPopMatrix();
}

void drawVehicle()
{
	vector<Vehicle *> cars = control->getCars();
	for (Vehicle *car : cars)
		drawRectangle(car->getPosition().x, car->getPosition().y, car->getPosition().z , car->getLength(), car->getWidth(), car->getColor(), 0);
}

void drawLight()
{
	vector<Traffic_light *> lights = control->getLights();
	for (Traffic_light *light : lights)
		drawCircle(light->getPosition().x, light->getPosition().y, light->getPosition().z, light->getRadius(), light->getColor());
}

void showInformation()
{
	Point margin;
	char totalCrowdsStr[5] = "\0", fpsStr[8] = "\0", frctnStr[6] = "\0", totalCarsStr[5] = "\0";
	margin.x = static_cast<float>(-winWidth) / 50;
	margin.y = static_cast<float>(winHeight) / 50 - 0.75F;
	
	glColor3f(0.0, 0.0, 0.0);
	drawText(margin.x, margin.y, "Total Ped:");
	totalCrowdsStr[0] = (char)('0' + control->getCrowdNum());
	drawText(margin.x + 4.0F, margin.y, totalCrowdsStr);

	drawText(margin.x, margin.y - 0.9F, "Total Car:");
	totalCarsStr[0] = (char)('0' + control->getCarsNum());
	drawText(margin.x + 4.0F, margin.y - 0.9F, totalCarsStr);

//	drawText(margin.x, margin.y - 1.8F, "FPS:");
//	_itoa_s(static_cast<int>(fps), fpsStr, 10);
//	strcat_s(fpsStr, ".");

//	_itoa_s((fps - static_cast<int>(fps)) * 100000, frctnStr, 10);
//	strncat_s(fpsStr, frctnStr, sizeof(fpsStr) - (strlen(fpsStr) + 1));
//	fpsStr[7] = '\0';
//	drawText(margin.x + 1.7F, margin.y - 1.8F, fpsStr);
}

void drawText(float x, float y, char text[])
{
	glDisable(GL_LIGHTING);	
	glDisable(GL_DEPTH_TEST);

	glPushMatrix();
		glTranslatef(x, y, 0.0);
		glScalef(0.0045F, 0.0045F, 0.0);
		glLineWidth(1.4F);

		int idx = 0;
		while (text[idx] != '\0')
			glutStrokeCharacter(GLUT_STROKE_ROMAN, text[idx++]);
	glPopMatrix();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
}

void reshape(int width, int height) {
	glViewport(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, static_cast<GLfloat>(width) / height, 1.0, 100.0);

	glMatrixMode(GL_MODELVIEW);

	winWidth = width;  
	winHeight = height;
}

void normalKey(unsigned char key, int xMousePos, int yMousePos) {
	switch (key) {
	case 'a':
		act = (!act) ? true : false;
		break;

	case 27:
		delete control;
 		control = 0;
		exit(0);
		break;
	}
}

void update() {
	int currTime, frameTime;
	static int prevTime;

	currTime = glutGet(GLUT_ELAPSED_TIME);
	frameTime = currTime - prevTime;
	prevTime = currTime;

	if (act) {
		control->act(static_cast<float>(frameTime) / 1800);
	}

	computeFPS();
	glutPostRedisplay();
	glutIdleFunc(update);
}

void computeFPS() {
	static int frameCount = 0;
	int currTime, frameTime;
	static int prevTime;

	frameCount++;
	currTime = glutGet(GLUT_ELAPSED_TIME); 
	frameTime = currTime - prevTime;

	if (frameTime > 1000) {
		fps = frameCount / (static_cast<float>(frameTime) / 1800);
		prevTime = currTime;
		frameCount = 0;
	}
}
