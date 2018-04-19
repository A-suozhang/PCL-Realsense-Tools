#include <iostream>
#include <algorithm>
#include "Renderer.h"	//渲染器头文件
#include "GraphicsDef.hpp"	//内含坐标系、图线等图形元素的定义
#include "MathUtilities.hpp"	//内含二维向量、三维向量的定义以及它们的各种运算
//#include "stdafx.h"
using namespace nsRenderer;
using namespace nsGraphicsDef;

Renderer renderer;	//渲染器对象

bool print = true;

//单位阶跃函数
inline float unitstep(float x) {
	return x > 0. ? 1. : 0.;
}

//生成离散的数据列表，参数：函数func、自变量范围tmin tmax、步长Ts，返回类型为二维向量（V2）列表，向量的横坐标（t）升序排列
vector<V2> generatedatalist(float func(float t),float tmin,float tmax,float Ts) {
	vector<V2> datp;
	for (float t = tmin; t < tmax; t += Ts) {
		datp.push_back(V2(t, func(t)));
	}
	return datp;
}

//信号反褶运算，即将数据列表中每个二维向量的横坐标乘以-1，然后重新按横坐标升序排序
void reversesignal(vector<V2>* dat) {
	for (auto it = dat->begin(); it != dat->end(); ++it) {
		it->x *= -1.0;
	}
	std::sort(dat->begin(), dat->end(), [](V2 a, V2 b) {return a.x < b.x; });
}

//信号时移运算，即将数据列表中每个二维向量的横坐标增加t
void delaysignal(vector<V2>* dat,float t) {
	for (auto it = dat->begin(); it != dat->end(); ++it) {
		it->x += t;
	}
}

//信号乘积运算，将两个数据列表中横坐标相等（差的绝对值小于eps）的二维向量的纵坐标相乘，结果放入新数据列表中
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

//求信号积分，采用矩形法即对数据列表中的每个数据点的纵坐标乘以相邻2个数据点间横坐标之差求和
float integratesignal(vector<V2>* dat) {
	float v = 0.0;
	for (auto it = dat->begin(); it != dat->end(); ++it) {
		if ((it + 1) != dat->end()) {
			v += (((it + 1)->x) - (it->x))*it->y;
		}
	}
	return v;
}

//渲染器更新函数，会被渲染器不断调用（参数step由渲染器填入，该值可被用户通过键盘改变）
void update(int step) {
	//生成信号f1的数据列表，Ts=0.1
	vector<V2> data = generatedatalist([](float t) {return (float)(t*(unitstep(t) - unitstep(t - 2)) + (4 - t)*(unitstep(t - 2) - unitstep(t - 4))); }, -1.0, 5.0, 0.1);
	//生成信号f2的数据列表，Ts=0.1
	vector<V2> datb = generatedatalist([](float t) {return (float)(unitstep(t + 2) - unitstep(t - 2)); }, -3.0, 3.0, 0.1);
	//生成信号f1的数据点更密的（用于绘图演示）数据列表，Ts=0.01
	vector<V2> datadisp = generatedatalist([](float t) {return (float)(t*(unitstep(t) - unitstep(t - 2)) + (4 - t)*(unitstep(t - 2) - unitstep(t - 4))); }, -1.0, 5.0, 0.01);
	//生成信号f1的数据点更密的（用于绘图演示）数据列表，Ts=0.01
	vector<V2> datbdisp = generatedatalist([](float t) {return (float)(unitstep(t + 2) - unitstep(t - 2)); }, -3.0, 3.0, 0.01);
	//定义f1与f2反褶时移相乘的数据列表（用于绘图演示），Ts=0.01
	vector<V2> databdisp;

	//计算用于演示的数据列表：f2反褶时移、f1与f2反褶时移相乘
	reversesignal(&datbdisp);
	delaysignal(&datbdisp, step/10.);
	multiplysignal(&databdisp, &datadisp, &datbdisp);

	reversesignal(&datb);	//将f2反褶
	delaysignal(&datb, -10.0);	//将f2时移到远处
	vector<V2> res;	//定义卷积结果数据列表
	for (int i = 0; i < 200; i++) {
		vector<V2> datr;	//存储信号相乘的结果
		delaysignal(&datb, 0.1);	//f2时移Ts
		multiplysignal(&datr, &data, &datb);	//信号相乘
		res.push_back(V2(-10 + 0.1*i, integratesignal(&datr)));	//积分后填入卷积结果
	}
	if (print) {
		std::cout << "t, y" << endl;
		for (auto it = res.begin(); it != res.end(); ++it) {
			printf("%2.2f, %2.5f\n", it->x, it->y);
		}
		print = false;
	}


	//下面开始定义绘图元素

	//图形对象RenderObject，内含图形顶点对象RenderPrimitive的列表
	RenderObject renderobj;

	//得到坐标系顶点对象列表
	vector<RenderPrimitive> pvc = coordinate(\
		V3(-5, -1, 0), V3(10, 10, 0), true, false, \
		true, V3(1., 1., 1.), 0);
	//合并到图形对象的顶点列表中
	renderobj.primitives.insert(renderobj.primitives.end(), pvc.begin(), pvc.end());

	//得到卷积结果、f1、f2、f1乘f2反褶时移四条图线的顶点列表，合并到图像对象的顶点列表中
	vector<RenderPrimitive> pvp = plotline(res, 1, 1, -5, 10, V3(1, 0, 0), 1, false);
	vector<RenderPrimitive> pvpA = plotline(datadisp, 1, 1, -5, 10, V3(0, 1, 0), 1, false);
	vector<RenderPrimitive> pvpAB = plotline(databdisp, 1, 1, -5, 10, V3(0, 0.3, 0), 1, true);
	vector<RenderPrimitive> pvpB = plotline(datbdisp, 1, 1, -5, 10, V3(0, 0, 1), 1, false);
	renderobj.primitives.insert(renderobj.primitives.end(), pvp.begin(), pvp.end());
	renderobj.primitives.insert(renderobj.primitives.end(), pvpA.begin(), pvpA.end());
	renderobj.primitives.insert(renderobj.primitives.end(), pvpB.begin(), pvpB.end());
	renderobj.primitives.insert(renderobj.primitives.end(), pvpAB.begin(), pvpAB.end());
	
	//绘制竖线
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

	//设置图形对象的位置、缩放比例
	renderobj.pos = V3(-300, -200, 0);
	renderobj.scale = V3(70, 70, 1);

	//将渲染器的图形对象列表清空，再添入生成好的新图形对象
	renderer.objects->clear();
	renderer.objects->push_back(renderobj);
}
int main() {
	//注册渲染器更新函数
	renderer.updatefunc = &update;
	//开始渲染
	renderer.Begin();
	return 0;
}
