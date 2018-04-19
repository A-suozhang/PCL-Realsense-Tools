#include <iostream>
#include <algorithm>
#include "Renderer.h"	//��Ⱦ��ͷ�ļ�
#include "GraphicsDef.hpp"	//�ں�����ϵ��ͼ�ߵ�ͼ��Ԫ�صĶ���
#include "MathUtilities.hpp"	//�ں���ά��������ά�����Ķ����Լ����ǵĸ�������
//#include "stdafx.h"
using namespace nsRenderer;
using namespace nsGraphicsDef;

Renderer renderer;	//��Ⱦ������

bool print = true;

//��λ��Ծ����
inline float unitstep(float x) {
	return x > 0. ? 1. : 0.;
}

//������ɢ�������б�����������func���Ա�����Χtmin tmax������Ts����������Ϊ��ά������V2���б������ĺ����꣨t����������
vector<V2> generatedatalist(float func(float t),float tmin,float tmax,float Ts) {
	vector<V2> datp;
	for (float t = tmin; t < tmax; t += Ts) {
		datp.push_back(V2(t, func(t)));
	}
	return datp;
}

//�źŷ������㣬���������б���ÿ����ά�����ĺ��������-1��Ȼ�����°���������������
void reversesignal(vector<V2>* dat) {
	for (auto it = dat->begin(); it != dat->end(); ++it) {
		it->x *= -1.0;
	}
	std::sort(dat->begin(), dat->end(), [](V2 a, V2 b) {return a.x < b.x; });
}

//�ź�ʱ�����㣬���������б���ÿ����ά�����ĺ���������t
void delaysignal(vector<V2>* dat,float t) {
	for (auto it = dat->begin(); it != dat->end(); ++it) {
		it->x += t;
	}
}

//�źų˻����㣬�����������б��к�������ȣ���ľ���ֵС��eps���Ķ�ά��������������ˣ���������������б���
void multiplysignal(vector<V2>* res, vector<V2>* data, vector<V2>* datb) {
	const float eps=0.0001;
	res->clear();
	auto itbprev = datb->begin();
	for (auto ita = data->begin(); ita != data->end(); ++ita) {
		auto itb = itbprev;
		while ( itb != datb->end()) {
			if (fabs((itb->x) - (ita->x)) < eps) {
				res->push_back(V2(ita->x, (ita->y)*(itb->y)));
				break;
			}
			itb++;
		}
		if (itb != datb->end())
			itbprev = itb;
	}
}

//���źŻ��֣����þ��η����������б��е�ÿ�����ݵ���������������2�����ݵ�������֮�����
float integratesignal(vector<V2>* dat) {
	float v = 0.0;
	for (auto it = dat->begin(); it != dat->end(); ++it) {
		if ((it + 1) != dat->end()) {
			v += (((it + 1)->x) - (it->x))*it->y;
		}
	}
	return v;
}

//��Ⱦ�����º������ᱻ��Ⱦ�����ϵ��ã�����step����Ⱦ�����룬��ֵ�ɱ��û�ͨ�����̸ı䣩
void update(int step) {
	//�����ź�f1�������б�Ts=0.1
	vector<V2> data = generatedatalist([](float t) {return (float)(t*(unitstep(t) - unitstep(t - 2)) + (4 - t)*(unitstep(t - 2) - unitstep(t - 4))); }, -1.0, 5.0, 0.1);
	//�����ź�f2�������б�Ts=0.1
	vector<V2> datb = generatedatalist([](float t) {return (float)(unitstep(t + 2) - unitstep(t - 2)); }, -3.0, 3.0, 0.1);
	//�����ź�f1�����ݵ���ܵģ����ڻ�ͼ��ʾ�������б�Ts=0.01
	vector<V2> datadisp = generatedatalist([](float t) {return (float)(t*(unitstep(t) - unitstep(t - 2)) + (4 - t)*(unitstep(t - 2) - unitstep(t - 4))); }, -1.0, 5.0, 0.01);
	//�����ź�f1�����ݵ���ܵģ����ڻ�ͼ��ʾ�������б�Ts=0.01
	vector<V2> datbdisp = generatedatalist([](float t) {return (float)(unitstep(t + 2) - unitstep(t - 2)); }, -3.0, 3.0, 0.01);
	//����f1��f2����ʱ����˵������б����ڻ�ͼ��ʾ����Ts=0.01
	vector<V2> databdisp;

	//����������ʾ�������б�f2����ʱ�ơ�f1��f2����ʱ�����
	reversesignal(&datbdisp);
	delaysignal(&datbdisp, step/10.);
	multiplysignal(&databdisp, &datadisp, &datbdisp);

	reversesignal(&datb);	//��f2����
	delaysignal(&datb, -10.0);	//��f2ʱ�Ƶ�Զ��
	vector<V2> res;	//��������������б�
	for (int i = 0; i < 200; i++) {
		vector<V2> datr;	//�洢�ź���˵Ľ��
		delaysignal(&datb, 0.1);	//f2ʱ��Ts
		multiplysignal(&datr, &data, &datb);	//�ź����
		res.push_back(V2(-10 + 0.1*i, integratesignal(&datr)));	//���ֺ����������
	}
	if (print) {
		std::cout << "t, y" << endl;
		for (auto it = res.begin(); it != res.end(); ++it) {
			printf("%2.2f, %2.5f\n", it->x, it->y);
		}
		print = false;
	}


	//���濪ʼ�����ͼԪ��

	//ͼ�ζ���RenderObject���ں�ͼ�ζ������RenderPrimitive���б�
	RenderObject renderobj;

	//�õ�����ϵ��������б�
	vector<RenderPrimitive> pvc = coordinate(\
		V3(-5, -1, 0), V3(10, 10, 0), true, false, \
		true, V3(1., 1., 1.), 0);
	//�ϲ���ͼ�ζ���Ķ����б���
	renderobj.primitives.insert(renderobj.primitives.end(), pvc.begin(), pvc.end());

	//�õ���������f1��f2��f1��f2����ʱ������ͼ�ߵĶ����б��ϲ���ͼ�����Ķ����б���
	vector<RenderPrimitive> pvp = plotline(res, 1, 1, -5, 10, V3(1, 0, 0), 1, false);
	vector<RenderPrimitive> pvpA = plotline(datadisp, 1, 1, -5, 10, V3(0, 1, 0), 1, false);
	vector<RenderPrimitive> pvpAB = plotline(databdisp, 1, 1, -5, 10, V3(0, 0.3, 0), 1, true);
	vector<RenderPrimitive> pvpB = plotline(datbdisp, 1, 1, -5, 10, V3(0, 0, 1), 1, false);
	renderobj.primitives.insert(renderobj.primitives.end(), pvp.begin(), pvp.end());
	renderobj.primitives.insert(renderobj.primitives.end(), pvpA.begin(), pvpA.end());
	renderobj.primitives.insert(renderobj.primitives.end(), pvpB.begin(), pvpB.end());
	renderobj.primitives.insert(renderobj.primitives.end(), pvpAB.begin(), pvpAB.end());
	
	//��������
	RenderPrimitive linep;
	linep.objtype = RenderPrimitive::lines;
	linep.style.color = RGBColor(1, 0, 0);
	linep.pts.push_back(V3(step / 10., 0, -0.1));
	float tmpy=0.;
	for (auto it = res.begin(); it != res.end(); ++it) {
		if (fabs((it->x) - (step / 10.)) < 0.001) {
			tmpy = it->y;
			break;
		}
	}
	linep.pts.push_back(V3(step / 10., tmpy, -0.1));
	renderobj.primitives.push_back(linep);

	RenderPrimitive textlabel;
	char str[30]="t=\0";
	textlabel.objtype = RenderPrimitive::text;
	textlabel.pts.push_back(V3(5, 8, -0.1));
	textlabel.style.color = RGBColor(1, 1, 1);
	sprintf_s(str, "t=%.2f", step / 10.);
	string strs = str;
	textlabel.name = str;
	renderobj.primitives.push_back(textlabel);

	//����ͼ�ζ����λ�á����ű���
	renderobj.pos = V3(-300, -200, 0);
	renderobj.scale = V3(70, 70, 1);

	//����Ⱦ����ͼ�ζ����б���գ����������ɺõ���ͼ�ζ���
	renderer.objects->clear();
	renderer.objects->push_back(renderobj);
}
int main() {
	//ע����Ⱦ�����º���
	renderer.updatefunc = &update;
	//��ʼ��Ⱦ
	renderer.Begin();
	return 0;
}
