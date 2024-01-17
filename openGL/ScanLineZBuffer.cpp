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
const int width = 800; //֡�������Ŀ��
const int height = 600; //֡�������ĸ߶�
const GLfloat minZ = -1000.0f; //��С���ֵ
Color frameBuffer[width][height]; //֡������
GLfloat zBuffer[width]; //z������
vector<zPolygon> polygons; //������б�
vector<ClassifyPolygonNode*> classifyPolygonTable; //�������α�
vector<ClassifyEdgeNode*> classifyEdgeTable; //����߱�
ActivePolygonNode* activePolygonHead; //�����α�ͷ���
ActiveEdgeNode* activeEdgeHead; //��߱�ͷ���

//������������ƽ��ķ���ϵ��
void calculatePlane(zPolygon& polygon, GLfloat& a, GLfloat& b, GLfloat& c, GLfloat& d) {
	//����������͹�ģ�ȡǰ�������㹹��һ��ƽ��
	zVertex v1 = polygon.vertices[0];
	zVertex v2 = polygon.vertices[1];
	zVertex v3 = polygon.vertices[2];
	//������������
	GLfloat x1 = v2.x - v1.x;
	GLfloat y1 = v2.y - v1.y;
	GLfloat z1 = v2.z - v1.z;
	GLfloat x2 = v3.x - v1.x;
	GLfloat y2 = v3.y - v1.y;
	GLfloat z2 = v3.z - v1.z;
	//�����������
	a = y1 * z2 - z1 * y2;
	b = z1 * x2 - x1 * z2;
	c = x1 * y2 - y1 * x2;
	//���㳣����
	d = -(a * v1.x + b * v1.y + c * v1.z);
}
//�������ε���ߵ����͵��y����
void calculateYRange(zPolygon& polygon, GLint& yMax, GLint& yMin) {
	yMax = -1;
	yMin = height;
	for (zVertex& v : polygon.vertices) {
		//�������y����ת��Ϊ����
		GLint y = round(v.y);
		//����y�����ֵ����Сֵ
		if (y > yMax) yMax = y;
		if (y < yMin) yMin = y;
	}
}

//�������ο�Խ��ɨ������Ŀ
GLint calculateDY(zPolygon& polygon) {
	GLint yMax, yMin;
	calculateYRange(polygon, yMax, yMin);
	return yMax - yMin + 1;
}
//�����������α߱�ĺ���
void buildClassifyPolygonEdgeTable() {
	//��ʼ���������α�ͷ���߱�
	for (int i = 0; i < height; i++) {
		classifyPolygonTable.push_back(NULL);
		classifyEdgeTable.push_back(NULL);
	}
	//�������ж����
	for (int i = 0; i < polygons.size(); i++) {
		zPolygon& polygon = polygons[i];
		//�������ε�ƽ�淽��ϵ��
		GLfloat a, b, c, d;
		calculatePlane(polygon, a, b, c, d);
		//�������ε���ߵ����͵��y����
		GLint yMax, yMin;
		calculateYRange(polygon, yMax, yMin);
		//����һ���������ν��
		ClassifyPolygonNode* cpn = new ClassifyPolygonNode();
		cpn->a = a;
		cpn->b = b;
		cpn->c = c;
		cpn->d = d;
		cpn->id = i;
		cpn->dy = calculateDY(polygon);
		cpn->color = polygon.color;
		cpn->next = NULL;
		//���������ν����뵽�������α�Ķ�Ӧλ��
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
		//��������ε����б�
		for (int j = 0; j < polygon.vertices.size(); j++) {
			//��ȡ�ߵ������˵�
			zVertex v1 = polygon.vertices[j];
			zVertex v2 = polygon.vertices[(j + 1) % polygon.vertices.size()];
			//���˵��y����ת��Ϊ����
			GLint y1 = round(v1.y);
			GLint y2 = round(v2.y);
			//����˵��y������ͬ��˵����ˮƽ�ߣ�����
			if (y1 == y2) continue;
			//����˵��y���겻ͬ��˵���Ƿ�ˮƽ�ߣ�����һ������߽��
			ClassifyEdgeNode* cen = new ClassifyEdgeNode();
			//����ߵ�б��
			GLdouble k = (v2.x - v1.x) / (v2.y - v1.y);
			//���ݶ˵��y�����С��ȷ���ߵ��϶˵���¶˵�
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
			//���ñ������Ķ���α��
			cen->id = i;
			cen->next = NULL;
			//������߽����뵽����߱�Ķ�Ӧλ��
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

//���������α�ͷ���߱��еĽ���ƶ��������α�ͻ�߱��еĺ���
void moveClassifyNodesToActiveNodes(GLint y) {
	//�ӷ������α���ȡ����ǰɨ���߶�Ӧ�Ķ���ν��
	ClassifyPolygonNode* cpn = classifyPolygonTable[y];
	while (cpn != NULL) {
		//����һ�������ν��
		ActivePolygonNode* apn = new ActivePolygonNode();
		apn->a = cpn->a;
		apn->b = cpn->b;
		apn->c = cpn->c;
		apn->d = cpn->d;
		apn->id = cpn->id;
		apn->dy = cpn->dy;
		apn->color = cpn->color;
		apn->next = NULL;
		//�������ν����뵽�����α��ͷ��
		apn->next = activePolygonHead;
		activePolygonHead = apn;
		//�ͷŷ������ν����ڴ�
		ClassifyPolygonNode* temp = cpn;
		cpn = cpn->next;
		delete temp;
	}
	//�ӷ���߱���ȡ����ǰɨ���߶�Ӧ�ı߽��
	ClassifyEdgeNode* cen = classifyEdgeTable[y];
	while (cen != NULL) {
		//����һ����߽��
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
		//����߽����뵽��߱��ͷ��
		aen->next = activeEdgeHead;
		activeEdgeHead = aen;
		//�ͷŷ���߽����ڴ�
		ClassifyEdgeNode* temp = cen;
		cen = cen->next;
		delete temp;
	}
}

//�Ի�߱��еĽ���������ĺ���
void sortActiveEdgeNodes() {
	//ʹ��ð�������㷨
	ActiveEdgeNode* p = activeEdgeHead;
	while (p != NULL) {
		ActiveEdgeNode* q = p->next;
		while (q != NULL) {
			//����󽻵��x����ϴ󣬽���������������
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

//�����߱��еĽ���Ե����ֵ����������ĺ���
void calculateDepthAndIncrement() {
	GLint y = round(activeEdgeHead->xl);
	//������߱��е����н��
	ActiveEdgeNode* aen = activeEdgeHead;
	while (aen != NULL) {
		//���ݽ�������ڵĶ���α�ţ��ӻ����α����ҵ���Ӧ��ƽ�淽��ϵ��
		ActivePolygonNode* apn = activePolygonHead;
		while (apn != NULL) {
			if (apn->id == aen->id) {
				//�����󽻵�����ֵ
				aen->zl = -(apn->a * aen->xl + apn->b * y + apn->d) / apn->c;
				//�����������
				aen->dzx = -apn->a / apn->c;
				aen->dzy = -apn->b / apn->c;
				break;
			}
			apn = apn->next;
		}
		aen = aen->next;
	}
}

//������߱��еĽ���ԣ�����֡��������z�������ĺ���
void updateFrameBufferAndZBuffer(GLint y) {
	//������߱��е����н��
	ActiveEdgeNode* aen = activeEdgeHead;
	while (aen != NULL) {
		//���󽻵���ҽ����x����ת��Ϊ����
		GLint xl = round(aen->xl);
		GLint xr = round(aen->xr);
		//��ʼ����ǰ���ֵΪ�󽻵�����ֵ
		GLfloat z = aen->zl;
		//�����󽻵���ҽ���֮�����������
		for (GLint x = xl; x <= xr; x++) {
			//�����ǰ���ֵС��z�������е����ֵ��˵�������ؿɼ�
			if (z < zBuffer[x]) {
				//����z�������е����ֵ
				zBuffer[x] = z;
				//���ݽ�������ڵĶ���α�ţ��ӻ����α����ҵ���Ӧ����ɫ
				ActivePolygonNode* apn = activePolygonHead;
				while (apn != NULL) {
					if (apn->id == aen->id) {
						//����֡�������е���ɫ
						frameBuffer[x][y] = apn->color;
						break;
					}
					apn = apn->next;
				}
			}
			//���µ�ǰ���ֵ��ʹ����������
			z += aen->dzx;
		}
		aen = aen->next;
	}
}

//���»����α�ͻ�߱��еĽ��ĺ���
void updateActiveNodes(GLint y) {
	//����һ�������α����ʱͷ���
	ActivePolygonNode* apnTemp = new ActivePolygonNode();
	apnTemp->next = activePolygonHead;
	//����һ����߱����ʱͷ���
	ActiveEdgeNode* aenTemp = new ActiveEdgeNode();
	aenTemp->next = activeEdgeHead;
	//���������α��е����н��
	ActivePolygonNode* apn = apnTemp;
	while (apn->next != NULL) {
		//���ٶ���ο�Խ��ʣ��ɨ������Ŀ
		apn->next->dy--;
		//�������ο�Խ��ʣ��ɨ������ĿΪ0��˵���ö�����Ѿ������꣬ɾ���ý��
		if (apn->next->dy == 0) {
			ActivePolygonNode* temp = apn->next;
			apn->next = apn->next->next;
			delete temp;
		}
		else {
			apn = apn->next;
		}
	}
	//������߱��е����н��
	ActiveEdgeNode* aen = aenTemp;
	while (aen->next != NULL) {
		//���ٱ߿�Խ��ʣ��ɨ������Ŀ
		aen->next->dyl--;
		aen->next->dyr--;
		//����߿�Խ��ʣ��ɨ������ĿΪ0��˵���ñ��Ѿ������꣬ɾ���ý��
		if (aen->next->dyl == 0 || aen->next->dyr == 0) {
			ActiveEdgeNode* temp = aen->next;
			aen->next = aen->next->next;
			delete temp;
		}
		else {
			//���ӽ����x���꣬ʹ����������
			aen->next->xl += aen->next->dxl;
			aen->next->xr += aen->next->dxr;
			aen = aen->next;
		}
	}
	//���»����α�ͻ�߱��ͷ���
	activePolygonHead = apnTemp->next;
	activeEdgeHead = aenTemp->next;
	//�ͷ���ʱͷ�����ڴ�
	delete apnTemp;
	delete aenTemp;
}
int main()
{
    return 0;
}
