#include "Renderer.h"
#include <iostream>
#include <string>
#include <Windows.h>

using namespace nsRenderer;

vector<RenderObject>* Renderer::objects = new vector<RenderObject>();
Camera* Renderer::cam = new Camera(0., 0., 60);
Vector2D<int>* Renderer::windowsize = new Vector2D<int>(800, 600);
bool Renderer::keystate[256];
Vector2D<float>* Renderer::cursorvelocity = new Vector2D<float>(0.0,0.0);
Vector2D<int> *Renderer::cursorpos = new Vector2D<int>(0, 0);
bool Renderer::freemouse = true;
void (*Renderer::updatefunc)(int);
int Renderer::step=0;

HDC Renderer::hDC;
HFONT Renderer::hFontA, Renderer::hFontW, Renderer::hOldFont;
GLYPHMETRICSFLOAT Renderer::agmf[256];

void nsRenderer::Renderer::Begin(string title)
{
	init(title);
}

nsRenderer::Renderer::Renderer()
{
	for (int i = 0; i < 256; i++)
		keystate[i] = false;
	objects = new vector<RenderObject>;
}

void nsRenderer::Renderer::init(string title)
{

	cam->camtype = Camera::p3D;

	int argc = 1;
	char* argv =(char*) malloc(10);
	argv[0] = '\0';

	glutInit(&argc,&argv);
	glutSetOption(GLUT_MULTISAMPLE, 8);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize((int)windowsize->x, (int)windowsize->y);
	glutInitWindowPosition(0, 0);
	glutCreateWindow(title.c_str());

	hDC = wglGetCurrentDC();
	hFontA = CreateFont(24, 0, 0, 0, FW_NORMAL, TRUE, FALSE, 0,
		ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, (LPCSTR)"Times New Roman");
	hOldFont = (HFONT)SelectObject(hDC, hFontA);
	wglUseFontOutlinesA(hDC, 0, 256, 1000, 0.0f, 0.1f, WGL_FONT_POLYGONS, agmf);
	wglUseFontBitmapsA(hDC, 0, 256, 2000);
	SelectObject(hDC, hOldFont);
	DeleteObject(hFontA);

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboardpress);
	glutKeyboardUpFunc(keyboardup);
	glutPassiveMotionFunc(motion);
	glutIdleFunc(idle);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glutMainLoop();
}

void nsRenderer::Renderer::reshape(int x, int y)
{
	glViewport(0, 0, x, y);
}

void nsRenderer::Renderer::keyboardpress(unsigned char key, int x, int y) {
	Renderer::keystate[key] = true;
}

void nsRenderer::Renderer::keyboardup(unsigned char key, int x, int y) {
	Renderer::keystate[key] = false;
	if (key == 'z') {
		Renderer::freemouse = !Renderer::freemouse;
		if(Renderer::freemouse)
			glutSetCursor(GLUT_CURSOR_INHERIT);
		else
			glutSetCursor(GLUT_CURSOR_NONE);
	}
}





void nsRenderer::Renderer::display()
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (Renderer::cam->camtype == Camera::o2D)
		gluOrtho2D(0, 0, 800, 600);
	if (Renderer::cam->camtype == Camera::p3D)
		gluPerspective(45.0, (float)(windowsize->x / windowsize->y), 0.05, 3000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (Renderer::cam->camtype == Camera::o2D)
		glScaled(1 / 800., -1 / 600., 1);
	//glTranslated(-100, 0, 0);
	//glTranslated(-1., 1., 0.);
	if (Renderer::cam->camtype == Camera::p3D) {
		gluLookAt(cam->pos.x, -cam->pos.y, cam->pos.z, cam->pos.x + 1.0*cos(cam->direxy)*cos(cam->direz), -cam->pos.y - 1.0*sin(cam->direxy)*cos(cam->direz), cam->pos.z + 1.0*sin(cam->direz), 0.f, 0.f, 1.f);
	}
	//drawcoordinate();
	glListBase(2000);
	/*
	
	glColor3f(1., 1., 1.);
	glPushMatrix();//Text
		glTranslated(10.0, 48.0, 0.0);
		glRasterPos3d(0.0, 0.0, 0.0);
		glCallLists(3, GL_UNSIGNED_BYTE, "rew\0");
	glPopMatrix();*/
	
	for (auto ito = Renderer::objects->begin(); ito != Renderer::objects->end(); ++ito) {
		//cout << "object:" << ito->name << endl;
		glPushMatrix();
		glTranslatef(ito->pos.x, -ito->pos.y, ito->pos.z);
		glScalef(ito->scale.x, ito->scale.y, ito->scale.z);
		for (auto itp = ito->primitives.begin(); itp != ito->primitives.end(); ++itp) {
			//cout << "primitive:" << itp->name << endl;
			glColor3f(itp->style.color.r, itp->style.color.g, itp->style.color.b);
			switch (itp->objtype) {
			case RenderPrimitive::points:
				glPointSize(itp->style.size);
				glBegin(GL_POINTS);
				for (auto itpts = itp->pts.begin(); itpts != itp->pts.end(); ++itpts) {
					glVertex3f(itpts->x, -itpts->y, itpts->z);
				}
				glEnd();
				break;
			case RenderPrimitive::colorfulpoints:
				glPointSize(itp->style.size);
				glBegin(GL_POINTS);
				for (auto itpts = itp->pts.begin(), itcol=itp->col.begin(); itpts != itp->pts.end()&&itcol!=itp->col.end(); ++itpts,++itcol) {
					glColor3f(itcol->x, itcol->y, itcol->z);
					glVertex3f(itpts->x, -itpts->y, itpts->z);
				}
				glEnd();
				break;
			case RenderPrimitive::lines:
				glPointSize(itp->style.size);
				glBegin(GL_LINE_STRIP);
				for (auto itpts = itp->pts.begin(); itpts != itp->pts.end(); ++itpts) {
					glVertex3f(itpts->x, -itpts->y, itpts->z);
				}
				glEnd();
				break;
			case RenderPrimitive::arrows:
				glPointSize(itp->style.size);
				glBegin(GL_LINE_STRIP);
				for (auto itpts = itp->pts.begin(); itpts != itp->pts.end(); ++itpts) {
					glVertex3f(itpts->x, -itpts->y, itpts->z);
				}
				glEnd();
				break;
			case RenderPrimitive::trianglefan:
				glPointSize(itp->style.size);
				glBegin(GL_TRIANGLE_FAN);
				for (auto itpts = itp->pts.begin(); itpts != itp->pts.end(); ++itpts) {
					glVertex3f(itpts->x, -itpts->y, itpts->z);
				}
				glEnd();
				break;
			case RenderPrimitive::trianglestrip:
				glPointSize(itp->style.size);
				glBegin(GL_TRIANGLE_STRIP);
				for (auto itpts = itp->pts.begin(); itpts != itp->pts.end(); ++itpts) {
					glVertex3f(itpts->x, -itpts->y, itpts->z);
				}
				glEnd();
				break;
			case RenderPrimitive::text:
				glPushMatrix();
				glTranslated(itp->pts[0].x, -itp->pts[0].y, itp->pts[0].z);
				glRasterPos3d(0.0, 0.0, 0.0);
				glCallLists(strlen(itp->name.c_str()), GL_UNSIGNED_BYTE, itp->name.c_str());
				glPopMatrix();
				break;
			}
		}
		glPopMatrix();
	}
	glutSwapBuffers();
}

void nsRenderer::Renderer::mouse(int button, int state, int x, int y)
{
	//std::cout << x << " " << " " << y << endl;
}

void nsRenderer::Renderer::motion(int x, int y)
{
	if (!freemouse) {
		int centerx = Renderer::windowsize->x / 2;
		int centery = Renderer::windowsize->y / 2;
		(*Renderer::cursorpos) += Vector2D<int>(x - centerx, y - centery);
		glutWarpPointer(centerx, centery);
	}
}

void nsRenderer::Renderer::idle()
{
	Sleep(30);
	if (Renderer::keystate['w']) {
		Renderer::cam->pos += r_Vector3D(cos(Renderer::cam->direxy), sin(Renderer::cam->direxy), 0.0)*Renderer::cam->movspd;
	}
	if (Renderer::keystate['s']) {
		Renderer::cam->pos -= r_Vector3D(cos(Renderer::cam->direxy), sin(Renderer::cam->direxy), 0.0)*Renderer::cam->movspd;
	}
	if (Renderer::keystate['a']) {
		Renderer::cam->pos += r_Vector3D(sin(Renderer::cam->direxy), -cos(Renderer::cam->direxy), 0.0)*Renderer::cam->movspd;
	}
	if (Renderer::keystate['d']) {
		Renderer::cam->pos -= r_Vector3D(sin(Renderer::cam->direxy), -cos(Renderer::cam->direxy), 0.0)*Renderer::cam->movspd;
	}
	if (Renderer::keystate['q']) {
		Renderer::cam->pos += r_Vector3D(0.0, 0.0, 1.0)*Renderer::cam->movspd;
	}
	if (Renderer::keystate['e']) {
		Renderer::cam->pos -= r_Vector3D(0.0, 0.0, 1.0)*Renderer::cam->movspd;
	}
	if (Renderer::keystate['u']) {
		Renderer::step--;
	}
	if (Renderer::keystate['i']) {
		Renderer::step++;
	}
	if (!freemouse) {
		(*Renderer::cursorvelocity) *= 0.6;
		(*Renderer::cursorvelocity) += Vector2D<float>((*Renderer::cursorpos).x, (*Renderer::cursorpos).y)*0.4f;
		Renderer::cam->direxy += Renderer::cursorvelocity->x / 1000.;
		Renderer::cam->direz -= Renderer::cursorvelocity->y / 1000.;
		(*Renderer::cursorpos).x = 0;
		(*Renderer::cursorpos).y = 0;
		Renderer::cam->Validate();
	}
	(*updatefunc)(step);
	glutPostRedisplay();
}

nsRenderer::Camera::Camera(r_Real direxy_, r_Real direz_, r_Real fov_)
{
	direxy = direxy_;
	direz = direz_;
	fov = fov_;
	movspd = 1.0;
}

nsRenderer::Camera::~Camera()
{

}

void nsRenderer::Camera::Validate()
{
	clamp(&direz, (r_Real)-PI / 2, (r_Real)PI / 2);
	warp(&direxy, (r_Real)0, (r_Real)(2 * PI));
}

nsRenderer::RenderObject::RenderObject()
{

}

nsRenderer::RenderObject::~RenderObject()
{
}

void nsRenderer::RenderObject::Render()
{
}

void nsRenderer::RenderPrimitive::Render()
{
}

nsRenderer::StyleList::StyleList(RGBColor color_, r_Real size_)
{
}
