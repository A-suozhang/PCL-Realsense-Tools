#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include "Renderer.h"
#include "GraphicsDef.hpp"
#include "MathUtilities.hpp"
using namespace nsRenderer;
using namespace nsGraphicsDef;
using namespace std;

Renderer renderer;

typedef struct {
	double x, y, z;
	int label;
} t_POINT;

void readPCL(string path,vector<t_POINT>* res,int* type) {
	ifstream ifs(path);
	string line;
	int linen = 1;
	while (getline(ifs, line))
	{
		*type = 0;
		if (linen >= 12) {
			int sucn;
			double rx, ry, rz;
			int rlabel=0;
			t_POINT pt;
			/*sucn = sscanf_s(line.c_str(), "%lf %lf %lf %d", &rx, &ry, &rz, &rlabel);
			pt.x = rx; pt.y = ry; pt.z = rz; pt.label = rlabel;
			if (sucn == 4) {
				res->push_back(pt);
			}*/
			sucn = sscanf_s(line.c_str(), "%lf %lf %lf %d", &rx, &ry, &rz, &rlabel);
			pt.x = rx; pt.y = ry; pt.z = rz; pt.label = rlabel;
			if (sucn == 3) {
				res->push_back(pt);
			}
			if (sucn == 4) {
				*type = 1;
				res->push_back(pt);
			}
		}
		linen++;
	}
}

void setrenderobj(vector<t_POINT>* pts,int typ) {
	RenderObject coor;
	vector<RenderPrimitive> pvc = coordinate(
		V3(0, 0, 0), V3(5, 5, 5), false, true, \
		false, V3(1., 1., 1.), 0);
	coor.primitives.insert(coor.primitives.end(), pvc.begin(), pvc.end());
	coor.pos = V3(0, 0, 0);
	coor.scale = V3(1, 1, 1);
	renderer.objects->push_back(coor);

	RenderObject ptso;
	RenderPrimitive rp;
	rp.style.color = RGBColor(1, 1, 1);
	rp.style.size = 2;
	rp.objtype = typ==1?RenderPrimitive::colorfulpoints: RenderPrimitive::points;
	for (auto it = pts->begin(); it != pts->end(); ++it) {
		rp.pts.push_back(V3(it->x, it->y, it->z));
		rp.col.push_back(V3(((1923+4372*it->label)%247/247.0), ((1923 + 4372 * it->label) % 347 / 347.0), ((1923 + 4372 * it->label) % 547 / 547.0)));
	}
	ptso.primitives.push_back(rp);
	ptso.pos = V3(0, 0, 0);
	ptso.scale = V3(5, 5, 5);
	renderer.objects->push_back(ptso);
}

void update(int step) {
	//RenderObject renderobj;
	//renderer.objects->push_back(renderobj);
	(*renderer.objects)[1].scale = V3(2.0 + step / 10.0, 2.0 + step / 10.0, 2.0 + step / 10.0);
}

int main(int argc, char *argv[]) {
	vector<t_POINT> pts;
	string title="PCD Visualization: ";
	int typ;
	//cout << argc << endl;
	if (argc == 1) {
		readPCL("rabbit_gra.pcd", &pts, &typ);
	} else if(argc==2) {
		title.append(argv[1]);
		readPCL(argv[1], &pts, &typ);
	} else {
		cout << "invalid arguments" << endl;
	}
	setrenderobj(&pts, typ);
	renderer.updatefunc = &update;
	renderer.Begin(title);
	return 0;
}
