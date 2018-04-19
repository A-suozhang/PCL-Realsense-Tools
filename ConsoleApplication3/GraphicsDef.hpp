#pragma once
#include "Renderer.h"
using namespace nsRenderer;
namespace nsGraphicsDef {
	typedef Vector3D<float> V3;
	typedef Vector2D<float> V2;
	vector<RenderPrimitive> coordinate( V3 sizlo, V3 sizhi, bool text, bool zaxis, bool color, V3 col, float arrsiz) {
		vector<RenderPrimitive> pv;
		RenderPrimitive p;
		p.style.size = 1;
		p.objtype = RenderPrimitive::lines;
		p.pts.clear();
		p.pts.push_back(V3(sizlo.x, 0, 0));
		p.pts.push_back(V3(sizhi.x, 0, 0));
		if (!color)
			p.style.color = RGBColor(1, 0, 0);
		else
			p.style.color = RGBColor(col.x, col.y, col.z);
		p.name = "x axis";
		pv.push_back(p);
		p.pts.clear();
		p.pts.push_back(V3(0, sizlo.y, 0));
		p.pts.push_back(V3(0, sizhi.y, 0));
		if (!color)
			p.style.color = RGBColor(0, 1, 0);
		else
			p.style.color = RGBColor(col.x, col.y, col.z);
		p.name = "y axis";
		pv.push_back(p);
		if (zaxis) {
			p.pts.clear();
			p.pts.push_back(V3(0, 0, sizlo.z));
			p.pts.push_back(V3(0, 0, sizhi.z));
			if (!color)
				p.style.color = RGBColor(0, 0, 1);
			else
				p.style.color = RGBColor(col.x, col.y, col.z);
			p.name = "z axis";
			pv.push_back(p);
		}

		if (text) {
			RenderPrimitive t;
			t.objtype = RenderPrimitive::text;
			if (!color)
				t.style.color = RGBColor(1, 1, 1);
			else
				t.style.color = RGBColor(col.x, col.y, col.z);

			t.pts.clear();
			t.pts.push_back(V3(sizhi.x, 0, 0));
			t.name = 't';
			pv.push_back(t);

			t.pts.clear();
			t.pts.push_back(V3(0, sizhi.y, 0));
			t.name = 'y';
			pv.push_back(t);

			
			for (int i = (int)sizlo.x; i < (int)sizhi.x; i++) {
				t.pts.clear();
				t.pts.push_back(V3(i-0.2, -0.5, 0));
				char s[10];
				_itoa_s(i, s, 10);
				t.name = s;
				pv.push_back(t);
			}

			if (zaxis) {
				t.pts.clear();
				t.pts.push_back(V3(0, 0, sizhi.z));
				t.name = 'z';
				pv.push_back(t);
			}
		}
		return pv;
	}
	vector<RenderPrimitive> plotline(vector<V2> datpoints,float xscale,float yscale,float xmin,float xmax,V3 col,float siz, bool fill) {
		vector<RenderPrimitive> pv;
		RenderPrimitive p;
		p.pts.clear();
		p.style.size = siz;
		p.style.color = RGBColor(col.x,col.y,col.z);
		if(!fill)
			p.objtype = RenderPrimitive::lines;
		else
			p.objtype = RenderPrimitive::trianglestrip;
		for (auto it = datpoints.begin(); it != datpoints.end(); ++it) {
			if (it->x > xmin&&it->x < xmax) {
				if (fill)
					p.pts.push_back(V3(xscale*(it->x), 0, 0));
				p.pts.push_back(V3(xscale*(it->x), yscale*(it->y), 0));
			}
		}
		pv.push_back(p);
		return pv;
	}
}