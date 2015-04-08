#include "BoundingBoxClass.h"
//  BoundingBoxClass
void BoundingBoxClass::Init(void)
{
	m_bInitialized = false;
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);
	m_v3Centroid = vector3(0.0f);
	m_sName = "NULL";
}
void BoundingBoxClass::Swap(BoundingBoxClass& other)
{
	std::swap(m_bInitialized, other.m_bInitialized);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);
	std::swap(m_v3Centroid, other.m_v3Centroid);
	std::swap(m_sName, other.m_sName);
}
void BoundingBoxClass::Release(void)
{
	//No pointers to release
}
//The big 3
BoundingBoxClass::BoundingBoxClass(){Init();}
BoundingBoxClass::BoundingBoxClass(BoundingBoxClass const& other)
{
	m_bInitialized = other.m_bInitialized;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;
	m_v3Centroid = other.m_v3Centroid;
	m_sName = other.m_sName;
}
BoundingBoxClass& BoundingBoxClass::operator=(BoundingBoxClass const& other)
{
	if(this != &other)
	{
		Release();
		Init();
		BoundingBoxClass temp(other);
		Swap(temp);
	}
	return *this;
}
BoundingBoxClass::~BoundingBoxClass(){Release();};
//Accessors
bool BoundingBoxClass::IsInitialized(void){ return m_bInitialized; }
vector3 BoundingBoxClass::GetMinimumOBB(void){ return m_v3Min; }
vector3 BoundingBoxClass::GetMaximumOBB(void){ return m_v3Max; }
vector3 BoundingBoxClass::GetCentroid(void){ return m_v3Centroid; }
String BoundingBoxClass::GetName(void){return m_sName;}
//Methods
void BoundingBoxClass::GenerateOrientedBoundingBox(String a_sInstanceName)
{
	//If this has already been initialized there is nothing to do here
	if(m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(pMeshMngr->IsInstanceCreated(a_sInstanceName))
	{
		m_sName = a_sInstanceName;
		
		std::vector<vector3> lVertices = pMeshMngr->GetVertices(m_sName);
		unsigned int nVertices = lVertices.size();
		m_v3Centroid = lVertices[0];
		m_v3Max = lVertices[0];
		m_v3Min = lVertices[0];
		for(unsigned int nVertex = 1; nVertex < nVertices; nVertex++)
		{
			//m_v3Centroid += lVertices[nVertex];
			if(m_v3Min.x > lVertices[nVertex].x)
				m_v3Min.x = lVertices[nVertex].x;
			else if(m_v3Max.x < lVertices[nVertex].x)
				m_v3Max.x = lVertices[nVertex].x;
			
			if(m_v3Min.y > lVertices[nVertex].y)
				m_v3Min.y = lVertices[nVertex].y;
			else if(m_v3Max.y < lVertices[nVertex].y)
				m_v3Max.y = lVertices[nVertex].y;

			if(m_v3Min.z > lVertices[nVertex].z)
				m_v3Min.z = lVertices[nVertex].z;
			else if(m_v3Max.z < lVertices[nVertex].z)
				m_v3Max.z = lVertices[nVertex].z;
		}
		m_v3Centroid = (m_v3Min + m_v3Max) / 2.0f;

		m_v3Size.x = glm::distance(vector3(m_v3Min.x, 0.0f, 0.0f), vector3(m_v3Max.x, 0.0f, 0.0f));
		m_v3Size.y = glm::distance(vector3(0.0f, m_v3Min.y, 0.0f), vector3(0.0f, m_v3Max.y, 0.0f));
		m_v3Size.z = glm::distance(vector3(0.0f, 0.0f, m_v3Min.z), vector3(0.0f, 0.0f, m_v3Max.z));

		m_bInitialized = true;
	}
}
void BoundingBoxClass::GenerateAxisAlignedBoundingBox(matrix4 a_m4ModeltoWorld)
{
	//Generate the Axis Aligned Bounding Box here based on the Oriented Bounding Box
	
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	std::vector<vector3> lVertices = pMeshMngr->GetVertices(m_sName);
	unsigned int nVertices = lVertices.size();

	vector3 AABB_MIN = static_cast<vector3>(a_m4ModeltoWorld * vector4(lVertices[0], 1.0f));
	vector3 AABB_MAX = static_cast<vector3>(a_m4ModeltoWorld * vector4(lVertices[0], 1.0f));

	for(unsigned int nVertex = 1; nVertex < nVertices; nVertex++)
	{
		vector3 vertex = static_cast<vector3>(a_m4ModeltoWorld * vector4(lVertices[nVertex], 1.0f));
		if (vertex.x > AABB_MAX.x){
			AABB_MAX.x = vertex.x;
		}
		
		if (vertex.y > AABB_MAX.y){
			AABB_MAX.y = vertex.y;
		}

		if (vertex.z > AABB_MAX.z){
			AABB_MAX.z = vertex.z;
		}

		if (vertex.x < AABB_MIN.x){
			AABB_MIN.x = vertex.x;
		}
		
		if (vertex.y < AABB_MIN.y){
			AABB_MIN.y = vertex.y;
		}

		if (vertex.z < AABB_MIN.z){
			AABB_MIN.z = vertex.z;
		}
	}
	vector3 aaBBCentroid = (AABB_MIN + AABB_MAX) / 2.0f;
	

	aaBBScale.x = glm::distance(vector3(AABB_MIN.x, 0.0f, 0.0f), vector3(AABB_MAX.x, 0.0f, 0.0f));
	aaBBScale.y = glm::distance(vector3(0.0f, AABB_MIN.y, 0.0f), vector3(0.0f, AABB_MAX.y, 0.0f));
	aaBBScale.z = glm::distance(vector3(0.0f, 0.0f, AABB_MIN.z), vector3(0.0f, 0.0f, AABB_MAX.z));

	/*aaBBScale.x = glm::distance(AABB_MIN.x, AABB_MAX.x);
	aaBBScale.y = glm::distance(AABB_MIN.y, AABB_MAX.y);
	aaBBScale.z = glm::distance(AABB_MIN.z, AABB_MAX.z);*/

	/*for(unsigned int nVertex = 1; nVertex < nVertices; nVertex++)
	{
		//Find the max X distance
		float fDistanceX = glm::distance(m_v3Centroid.x, lVertices[nVertex].x);
		if(aaBBScale.x < fDistanceX)
			aaBBScale.x = fDistanceX;
		//Find the max Y distance
		float fDistanceY = glm::distance(m_v3Centroid.y, lVertices[nVertex].y);
		if(aaBBScale.y < fDistanceY)
			aaBBScale.y = fDistanceY;
		//Find the max Z distance
		float fDistanceZ = glm::distance(m_v3Centroid.z, lVertices[nVertex].z);
		if(aaBBScale.z < fDistanceZ)
			aaBBScale.z = fDistanceZ;
	}*/

	float bank = 0; 
	float angle = 0;
	glm::axisAngle(a_m4ModeltoWorld, vector3(0.0f, 0.0f, 1.0f), angle);
	angle = glm::degrees(angle);

	//aaBox =  glm::translate(m_v3Centroid) *  glm::rotate(matrix4(IDENTITY), angle , vector3(0.0f, 0.0f, 1.0f)) * glm::scale(aaBBScale) * glm::rotate(matrix4(IDENTITY), -angle , vector3(0.0f, 0.0f, 1.0f));

	aaBox =  glm::translate(m_v3Centroid) * glm::scale(aaBBScale);
	

	

	/*float bank = 0; 
	float angle = 0;
	glm::axisAngle(a_m4ModeltoWorld, vector3(0.0f, 0.0f, 1.0f), angle);

	if(angle < 3){
		angle = 180 * angle / 3.14156;
	std::cout << angle << std::endl;*/

	
	//aaBox = a_m4ModeltoWorld * glm::translate(m_v3Centroid) * glm::rotate(matrix4(IDENTITY), -angle , vector3(0.0f, 0.0f, 1.0f));
	



}
void BoundingBoxClass::AddBoxToRenderList(matrix4 a_m4ModelToWorld, vector3 a_vColor, bool a_bRenderCentroid)
{
	if(!m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(a_bRenderCentroid)
		pMeshMngr->AddAxisToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid));
	pMeshMngr->AddAxisToQueue(aaBox);
	pMeshMngr->AddCubeToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid) * glm::scale(m_v3Size), a_vColor, MERENDER::WIRE);
	pMeshMngr->AddCubeToQueue(a_m4ModelToWorld * aaBox, a_vColor, MERENDER::WIRE);

}