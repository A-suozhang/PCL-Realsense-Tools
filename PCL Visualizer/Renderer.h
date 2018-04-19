#pragma once
#include <GL/freeglut.h>
#include <vector>
#include "Config.h"
#include "MathUtilities.hpp"
using namespace nsMath;
using namespace std;
namespace nsRenderer {
	typedef Vector3D<r_Real> r_Vector3D;
	typedef Vector2D<r_Real> r_Vector2D;
	class RGBColor {
	public:
		r_Real r, g, b;
		RGBColor(r_Real r = 0., r_Real g = 0., r_Real b = 0.) :r(r), g(g), b(b) {}
	};

	class StyleList {
	public:
		RGBColor color;
		r_Real size;
		StyleList(r_Real size = 1.0, RGBColor color = RGBColor(0, 0, 0)) :size(size), color(color) {};
		StyleList(RGBColor color_, r_Real size_);
	};

	class RenderPrimitive {
	public:
		enum ObjType { points,colorfulpoints, lines, arrows, trianglefan, trianglestrip, text } objtype;
		vector <r_Vector3D> pts;
		vector <r_Vector3D> col;
		StyleList style;
		string name;
		RenderPrimitive(ObjType objtype = points, vector<r_Vector3D> pts = vector<r_Vector3D>(), StyleList style = StyleList()) :objtype(objtype), pts(pts), style(style) {};
		void Render();
	};

	class RenderObject {
	public:
		vector <RenderPrimitive> primitives;
		string name;
		r_Vector3D pos;
		r_Vector3D scale;
		RenderObject();
		~RenderObject();
		void Render();
	};

	class Camera {
	public:
		r_Vector3D pos;
		r_Real direxy, direz;
		r_Real fov;
		r_Real movspd;
		enum CamType { p3D, o3D, o2D } camtype;
		Camera(r_Real direxy_, r_Real direz_, r_Real fov_);
		~Camera();
		void Validate();
	};

	class Renderer
	{
	public:
		static vector<RenderObject> *objects;
		static Camera *cam;
		static Vector2D<int> *windowsize;
		static bool keystate[256];
		static Vector2D<int> *cursorpos;
		static Vector2D<float> *cursorvelocity;
		static bool freemouse;
		static HDC hDC;
		static HFONT hFontA, hFontW, hOldFont;
		static GLYPHMETRICSFLOAT agmf[256];
		void Begin(string title);
		Renderer();
		void init(string title);
		static void(*updatefunc)(int);
		static int step;
	private:
		static void reshape(int x, int y);
		static void display();
		static void mouse(int button, int state, int x, int y);
		static void motion(int x, int y);
		static void keyboardpress(unsigned char key, int x, int y);
		static void keyboardup(unsigned char key, int x, int y);
		static void idle();
	};
	
}
