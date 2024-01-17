#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "ZBufferStruct.h"
using namespace std;
#define printV3(x) cout<<"("<<x[0]<<","<<x[1]<<","<<x[2]<<")"
#define printColor3(x) cout<<"("<<x.r<<","<<x.g<<","<<x.b<<")"
#define printV2(x) cout<<"("<<x[0]<<","<<x[1]<<")"
bool Import(const string& pFile) {
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(pFile,
		aiProcess_CalcTangentSpace|
		aiProcess_Triangulate |
		aiProcess_JoinIdenticalVertices|
		aiProcess_SortByPType);

	if (!scene) {
		printf(importer.GetErrorString());
		return false;
	}
	/*int meshes = scene->mNumMeshes;
	for (int i = 0; i < meshes; i++) {
		aiMesh* mesh = scene->mMeshes[i];
		printf(mesh->mName.C_Str());
		printf("\n");
		cout<<mesh->mNumVertices<<endl;
		for (int i = 0; i < mesh->mNumVertices; i++) {
			cout << i << ":";
			printV3(mesh->mVertices[i]);
			cout << endl;
			//printV3(mesh->mNormals[i]);
			printV2(mesh->mTextureCoords[0][i]);
			cout << endl;
			cout << mesh->mNumFaces<<endl;
 
		}
	}*/
	return true;
}
const int width = 800; //帧缓冲器的宽度
const int height = 600; //帧缓冲器的高度
const GLfloat minZ = -1000.0f; //最小深度值
Color frameBuffer[width][height]; //帧缓冲器
GLfloat zBuffer[width]; //z缓冲器
vector<zPolygon> polygons; //多边形列表
vector<ClassifyPolygonNode*> classifyPolygonTable; //分类多边形表
vector<ClassifyEdgeNode*> classifyEdgeTable; //分类边表
ActivePolygonNode* activePolygonHead; //活化多边形表头结点
ActiveEdgeNode* activeEdgeHead; //活化边表头结点

//计算多边形所在平面的方程系数
void calculatePlane(zPolygon& polygon, GLfloat& a, GLfloat& b, GLfloat& c, GLfloat& d) {
	//假设多边形是凸的，取前三个顶点构成一个平面
	zVertex v1 = polygon.vertices[0];
	zVertex v2 = polygon.vertices[1];
	zVertex v3 = polygon.vertices[2];
	//计算两个向量
	GLfloat x1 = v2.x - v1.x;
	GLfloat y1 = v2.y - v1.y;
	GLfloat z1 = v2.z - v1.z;
	GLfloat x2 = v3.x - v1.x;
	GLfloat y2 = v3.y - v1.y;
	GLfloat z2 = v3.z - v1.z;
	//计算向量叉积
	a = y1 * z2 - z1 * y2;
	b = z1 * x2 - x1 * z2;
	c = x1 * y2 - y1 * x2;
	//计算常数项
	d = -(a * v1.x + b * v1.y + c * v1.z);
}
//计算多边形的最高点和最低点的y坐标
void calculateYRange(zPolygon& polygon, GLint& yMax, GLint& yMin) {
	yMax = -1;
	yMin = height;
	for (zVertex& v : polygon.vertices) {
		//将顶点的y坐标转换为整数
		GLint y = round(v.y);
		//更新y的最大值和最小值
		if (y > yMax) yMax = y;
		if (y < yMin) yMin = y;
	}
}

//计算多边形跨越的扫描线数目
GLint calculateDY(zPolygon& polygon) {
	GLint yMax, yMin;
	calculateYRange(polygon, yMax, yMin);
	return yMax - yMin + 1;
}
//构建分类多边形边表的函数
void buildClassifyPolygonEdgeTable() {
	//初始化分类多边形表和分类边表
	for (int i = 0; i < height; i++) {
		classifyPolygonTable.push_back(NULL);
		classifyEdgeTable.push_back(NULL);
	}
	//遍历所有多边形
	for (int i = 0; i < polygons.size(); i++) {
		zPolygon& polygon = polygons[i];
		//计算多边形的平面方程系数
		GLfloat a, b, c, d;
		calculatePlane(polygon, a, b, c, d);
		//计算多边形的最高点和最低点的y坐标
		GLint yMax, yMin;
		calculateYRange(polygon, yMax, yMin);
		//创建一个分类多边形结点
		ClassifyPolygonNode* cpn = new ClassifyPolygonNode();
		cpn->a = a;
		cpn->b = b;
		cpn->c = c;
		cpn->d = d;
		cpn->id = i;
		cpn->dy = calculateDY(polygon);
		cpn->color = polygon.color;
		cpn->next = NULL;
		//将分类多边形结点插入到分类多边形表的对应位置
		ClassifyPolygonNode* cpnHead = classifyPolygonTable[yMin];
		if (cpnHead == NULL) {
			classifyPolygonTable[yMin] = cpn;
		}
		else {
			while (cpnHead->next != NULL) {
				cpnHead = cpnHead->next;
			}
			cpnHead->next = cpn;
		}
		//遍历多边形的所有边
		for (int j = 0; j < polygon.vertices.size(); j++) {
			//获取边的两个端点
			zVertex v1 = polygon.vertices[j];
			zVertex v2 = polygon.vertices[(j + 1) % polygon.vertices.size()];
			//将端点的y坐标转换为整数
			GLint y1 = round(v1.y);
			GLint y2 = round(v2.y);
			//如果端点的y坐标相同，说明是水平边，跳过
			if (y1 == y2) continue;
			//如果端点的y坐标不同，说明是非水平边，创建一个分类边结点
			ClassifyEdgeNode* cen = new ClassifyEdgeNode();
			//计算边的斜率
			GLdouble k = (v2.x - v1.x) / (v2.y - v1.y);
			//根据端点的y坐标大小，确定边的上端点和下端点
			if (y1 < y2) {
				cen->x = v1.x;
				cen->dx = -1.0 / k;
				cen->dy = y2 - y1;
			}
			else {
				cen->x = v2.x;
				cen->dx = 1.0 / k;
				cen->dy = y1 - y2;
			}
			//设置边所属的多边形编号
			cen->id = i;
			cen->next = NULL;
			//将分类边结点插入到分类边表的对应位置
			ClassifyEdgeNode* cenHead = classifyEdgeTable[std::min(y1, y2)];
			if (cenHead == NULL) {
				classifyEdgeTable[std::min(y1, y2)] = cen;
			}
			else {
				while (cenHead->next != NULL) {
					cenHead = cenHead->next;
				}
				cenHead->next = cen;
			}
		}
	}
}

//将分类多边形表和分类边表中的结点移动到活化多边形表和活化边表中的函数
void moveClassifyNodesToActiveNodes(GLint y) {
	//从分类多边形表中取出当前扫描线对应的多边形结点
	ClassifyPolygonNode* cpn = classifyPolygonTable[y];
	while (cpn != NULL) {
		//创建一个活化多边形结点
		ActivePolygonNode* apn = new ActivePolygonNode();
		apn->a = cpn->a;
		apn->b = cpn->b;
		apn->c = cpn->c;
		apn->d = cpn->d;
		apn->id = cpn->id;
		apn->dy = cpn->dy;
		apn->color = cpn->color;
		apn->next = NULL;
		//将活化多边形结点插入到活化多边形表的头部
		apn->next = activePolygonHead;
		activePolygonHead = apn;
		//释放分类多边形结点的内存
		ClassifyPolygonNode* temp = cpn;
		cpn = cpn->next;
		delete temp;
	}
	//从分类边表中取出当前扫描线对应的边结点
	ClassifyEdgeNode* cen = classifyEdgeTable[y];
	while (cen != NULL) {
		//创建一个活化边结点
		ActiveEdgeNode* aen = new ActiveEdgeNode();
		aen->xl = cen->x;
		aen->dxl = cen->dx;
		aen->dyl = cen->dy;
		aen->xr = cen->x;
		aen->dxr = cen->dx;
		aen->dyr = cen->dy;
		aen->zl = 0.0f;
		aen->dzx = 0.0f;
		aen->dzy = 0.0f;
		aen->id = cen->id;
		aen->next = NULL;
		//将活化边结点插入到活化边表的头部
		aen->next = activeEdgeHead;
		activeEdgeHead = aen;
		//释放分类边结点的内存
		ClassifyEdgeNode* temp = cen;
		cen = cen->next;
		delete temp;
	}
}

//对活化边表中的结点进行排序的函数
void sortActiveEdgeNodes() {
	//使用冒泡排序算法
	ActiveEdgeNode* p = activeEdgeHead;
	while (p != NULL) {
		ActiveEdgeNode* q = p->next;
		while (q != NULL) {
			//如果左交点的x坐标较大，交换两个结点的数据
			if (p->xl > q->xl) {
				swap(p->xl, q->xl);
				swap(p->dxl, q->dxl);
				swap(p->dyl, q->dyl);
				swap(p->xr, q->xr);
				swap(p->dxr, q->dxr);
				swap(p->dyr, q->dyr);
				swap(p->zl, q->zl);
				swap(p->dzx, q->dzx);
				swap(p->dzy, q->dzy);
				swap(p->id, q->id);
			}
			q = q->next;
		}
		p = p->next;
	}
}

//计算活化边表中的交点对的深度值和深度增量的函数
void calculateDepthAndIncrement() {
	GLint y = round(activeEdgeHead->xl);
	//遍历活化边表中的所有结点
	ActiveEdgeNode* aen = activeEdgeHead;
	while (aen != NULL) {
		//根据交点对所在的多边形编号，从活化多边形表中找到对应的平面方程系数
		ActivePolygonNode* apn = activePolygonHead;
		while (apn != NULL) {
			if (apn->id == aen->id) {
				//计算左交点的深度值
				aen->zl = -(apn->a * aen->xl + apn->b * y + apn->d) / apn->c;
				//计算深度增量
				aen->dzx = -apn->a / apn->c;
				aen->dzy = -apn->b / apn->c;
				break;
			}
			apn = apn->next;
		}
		aen = aen->next;
	}
}

//遍历活化边表中的交点对，更新帧缓冲器和z缓冲器的函数
void updateFrameBufferAndZBuffer(GLint y) {
	//遍历活化边表中的所有结点
	ActiveEdgeNode* aen = activeEdgeHead;
	while (aen != NULL) {
		//将左交点和右交点的x坐标转换为整数
		GLint xl = round(aen->xl);
		GLint xr = round(aen->xr);
		//初始化当前深度值为左交点的深度值
		GLfloat z = aen->zl;
		//遍历左交点和右交点之间的所有像素
		for (GLint x = xl; x <= xr; x++) {
			//如果当前深度值小于z缓冲器中的深度值，说明该像素可见
			if (z < zBuffer[x]) {
				//更新z缓冲器中的深度值
				zBuffer[x] = z;
				//根据交点对所在的多边形编号，从活化多边形表中找到对应的颜色
				ActivePolygonNode* apn = activePolygonHead;
				while (apn != NULL) {
					if (apn->id == aen->id) {
						//更新帧缓冲器中的颜色
						frameBuffer[x][y] = apn->color;
						break;
					}
					apn = apn->next;
				}
			}
			//更新当前深度值，使用增量方法
			z += aen->dzx;
		}
		aen = aen->next;
	}
}

//更新活化多边形表和活化边表中的结点的函数
void updateActiveNodes(GLint y) {
	//创建一个活化多边形表的临时头结点
	ActivePolygonNode* apnTemp = new ActivePolygonNode();
	apnTemp->next = activePolygonHead;
	//创建一个活化边表的临时头结点
	ActiveEdgeNode* aenTemp = new ActiveEdgeNode();
	aenTemp->next = activeEdgeHead;
	//遍历活化多边形表中的所有结点
	ActivePolygonNode* apn = apnTemp;
	while (apn->next != NULL) {
		//减少多边形跨越的剩余扫描线数目
		apn->next->dy--;
		//如果多边形跨越的剩余扫描线数目为0，说明该多边形已经处理完，删除该结点
		if (apn->next->dy == 0) {
			ActivePolygonNode* temp = apn->next;
			apn->next = apn->next->next;
			delete temp;
		}
		else {
			apn = apn->next;
		}
	}
	//遍历活化边表中的所有结点
	ActiveEdgeNode* aen = aenTemp;
	while (aen->next != NULL) {
		//减少边跨越的剩余扫描线数目
		aen->next->dyl--;
		aen->next->dyr--;
		//如果边跨越的剩余扫描线数目为0，说明该边已经处理完，删除该结点
		if (aen->next->dyl == 0 || aen->next->dyr == 0) {
			ActiveEdgeNode* temp = aen->next;
			aen->next = aen->next->next;
			delete temp;
		}
		else {
			//增加交点的x坐标，使用增量方法
			aen->next->xl += aen->next->dxl;
			aen->next->xr += aen->next->dxr;
			aen = aen->next;
		}
	}
	//更新活化多边形表和活化边表的头结点
	activePolygonHead = apnTemp->next;
	activeEdgeHead = aenTemp->next;
	//释放临时头结点的内存
	delete apnTemp;
	delete aenTemp;
}
int main()
{
    return 0;
}
