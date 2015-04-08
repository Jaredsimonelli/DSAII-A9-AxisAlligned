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
vector3 BoundingBoxClass::GetMinimumAABB(void){ return AABB_MIN; }
vector3 BoundingBoxClass::GetMaximumAABB(void){ return AABB_MAX; }
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

	// Grab the mesh manager's vertices and vertice count
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	std::vector<vector3> lVertices = pMeshMngr->GetVertices(m_sName);
	unsigned int nVertices = lVertices.size();

	// Vector3 min and max init
	AABB_MIN = static_cast<vector3>(a_m4ModeltoWorld * vector4(lVertices[0], 1.0f));
	AABB_MAX = static_cast<vector3>(a_m4ModeltoWorld * vector4(lVertices[0], 1.0f));
	for(unsigned int nVertex = 1; nVertex < nVertices; nVertex++)
	{
		// Grab the model to world vertices 
		vector3 vertex = static_cast<vector3>(a_m4ModeltoWorld * vector4(lVertices[nVertex], 1.0f));

		//m_v3Centroid += vertex;
		if(AABB_MIN.x > vertex.x)
			AABB_MIN.x = vertex.x;
		else if(AABB_MAX.x < vertex.x)
			AABB_MAX.x = vertex.x;
			
		if(AABB_MIN.y > vertex.y)
			AABB_MIN.y = vertex.y;
		else if(AABB_MAX.y < vertex.y)
			AABB_MAX.y = vertex.y;

		if(AABB_MIN.z > vertex.z)
			AABB_MIN.z = vertex.z;
		else if(AABB_MAX.z < vertex.z)
			AABB_MAX.z = vertex.z;
	}
	vector3 aaBBCentroid = (AABB_MIN + AABB_MAX) / 2.0f;

	aaBBScale.x = glm::distance(vector3(AABB_MIN.x, 0.0f, 0.0f), vector3(AABB_MAX.x, 0.0f, 0.0f));
	aaBBScale.y = glm::distance(vector3(0.0f, AABB_MIN.y, 0.0f), vector3(0.0f, AABB_MAX.y, 0.0f));
	aaBBScale.z = glm::distance(vector3(0.0f, 0.0f, AABB_MIN.z), vector3(0.0f, 0.0f, AABB_MAX.z));

	aaBox =  glm::translate(aaBBCentroid) *   glm::scale(aaBBScale);
}

void BoundingBoxClass::AddBoxToRenderList(matrix4 a_m4ModelToWorld, vector3 a_vColor, bool a_bRenderCentroid)
{
	if(!m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(a_bRenderCentroid)
		pMeshMngr->AddAxisToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid));
	pMeshMngr->AddCubeToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid) * glm::scale(m_v3Size), a_vColor, MERENDER::WIRE);
	pMeshMngr->AddCubeToQueue(aaBox, a_vColor, MERENDER::WIRE);

}