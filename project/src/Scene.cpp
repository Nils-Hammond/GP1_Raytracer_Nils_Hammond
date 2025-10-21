#include <algorithm>
#include "Scene.h"
#include "Utils.h"
#include "Material.h"
#include <iostream>

namespace dae {

#pragma region Base Scene
	//Initialize Scene with Default Solid Color Material (RED)
	Scene::Scene() :
		m_Materials({ new Material_SolidColor({1,0,0}) })
	{
		m_SphereGeometries.reserve(32);
		m_PlaneGeometries.reserve(32);
		m_TriangleMeshGeometries.reserve(32);
		m_Lights.reserve(32);
	}

	Scene::~Scene()
	{
		for (auto& pMaterial : m_Materials)
		{
			delete pMaterial;
			pMaterial = nullptr;
		}

		m_Materials.clear();
	}

	void dae::Scene::GetClosestHit(const Ray& ray, HitRecord& closestHit) const
	{
		const std::vector<Sphere>& spheres = GetSphereGeometries();
		const std::vector<Plane>& planes = GetPlaneGeometries();
		float closestT = FLT_MAX;
		HitRecord tempHit{};

		//for (const Sphere& sphere : spheres)
		//{
		//	if (GeometryUtils::HitTest_Sphere(sphere, ray, tempHit) && tempHit.t < closestT)
		//	{
		//		closestT = tempHit.t;
		//		closestHit = tempHit;
		//	}
		//}
		IntersectSphereBVH(ray, closestHit, rootNodeIdx, false);
		closestT = closestHit.t;

		for (const TriangleMesh& mesh : m_TriangleMeshGeometries)
		{
			if (GeometryUtils::HitTest_TriangleMesh(mesh, ray, tempHit) && tempHit.t < closestT)
			{
				closestT = tempHit.t;
				closestHit = tempHit;
			}
		}
		for (const Plane& plane : planes)
		{
			if (GeometryUtils::HitTest_Plane(plane, ray, tempHit) && tempHit.t < closestT)
			{
				closestT = tempHit.t;
				closestHit = tempHit;
			}
		}
	}

	bool Scene::DoesHit(const Ray& ray) const
	{
		//todo W2
		const std::vector<Sphere>& spheres = GetSphereGeometries();
		const std::vector<Plane>& planes = GetPlaneGeometries();
		HitRecord tempHit{};

		if (IntersectSphereBVH(ray, tempHit, rootNodeIdx, true))
			return true;

		//for (const Sphere& sphere : spheres)
		//{
		//	if (GeometryUtils::HitTest_Sphere(sphere, ray, tempHit, true))
		//	{
		//		return true;
		//	}
		//}
		for (const TriangleMesh& mesh : m_TriangleMeshGeometries)
		{
			if (GeometryUtils::HitTest_TriangleMesh(mesh, ray))
			{
				return true;
			}
		}
		for (const Plane& plane : planes)
		{
			if (GeometryUtils::HitTest_Plane(plane, ray, tempHit, true))
			{
				return true;
			}
		}
		return false;
	}

#pragma region BVH Helpers
	void Scene::BuildSphereBVH()
	{
		if (m_SphereGeometries.empty()) return;
		nodesUsed = 1;
		m_SphereIndices.clear();
		m_SphereBVHNode.clear();
		const int numSpheres = static_cast<const int>(m_SphereGeometries.size());

		for (int i = 0; i < numSpheres; ++i)
		{
			m_SphereIndices.emplace_back(i);
		}
		for (int i = 0; i < numSpheres * 2 - 1; ++i)
		{
			m_SphereBVHNode.emplace_back();
		}
		BVHNode& root = m_SphereBVHNode[rootNodeIdx];
		root.leftNode = 0;
		root.firstPrimIdx = 0;
		root.primCount = numSpheres;

		UpdateNodeBounds(rootNodeIdx);

		Subdivide(rootNodeIdx);
	}

	void Scene::UpdateNodeBounds(int nodeIdx)
	{
		BVHNode& node = m_SphereBVHNode[nodeIdx];
		node.aabbMin = Vector3(FLT_MAX, FLT_MAX, FLT_MAX);
		node.aabbMax = Vector3(FLT_MIN, FLT_MIN, FLT_MIN);

		for (uint8_t first = node.firstPrimIdx, i = 0; i < node.primCount; i++)
		{
			uint8_t leafSphereIdx = m_SphereIndices[first + i];
			Sphere& leafSphere = m_SphereGeometries[leafSphereIdx];

			Vector3 sphereMin = leafSphere.origin - Vector3(leafSphere.radius, leafSphere.radius, leafSphere.radius);
			Vector3 sphereMax = leafSphere.origin + Vector3(leafSphere.radius, leafSphere.radius, leafSphere.radius);

			node.aabbMin.x = std::min(node.aabbMin.x, sphereMin.x);
			node.aabbMin.y = std::min(node.aabbMin.y, sphereMin.y);
			node.aabbMin.z = std::min(node.aabbMin.z, sphereMin.z);
			node.aabbMax.x = std::max(node.aabbMax.x, sphereMax.x);
			node.aabbMax.y = std::max(node.aabbMax.y, sphereMax.y);
			node.aabbMax.z = std::max(node.aabbMax.z, sphereMax.z);
		}
	}

	void Scene::Subdivide(int nodeIdx)
	{
		// terminate recursion
		BVHNode& node = m_SphereBVHNode[nodeIdx];
		if (node.primCount <= 2) return;

		// determine split axis and position
		Vector3 extent = node.aabbMax - node.aabbMin;
		int axis = 0;
		if (extent.y > extent.x) axis = 1;
		if (extent.z > extent[axis]) axis = 2;
		float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;

		// in-place partition
		int i = node.firstPrimIdx;
		int j = i + node.primCount - 1;
		while (i <= j)
		{
			if (m_SphereGeometries[m_SphereIndices[i]].origin[axis] < splitPos)
				i++;
			else
				std::swap(m_SphereIndices[i], m_SphereIndices[j--]);
		}

		// abort split if one of the sides is empty
		int leftCount = i - node.firstPrimIdx;
		if (leftCount == 0 || leftCount == node.primCount) return;

		// create child nodes
		int leftChildIdx = nodesUsed++;
		int rightChildIdx = nodesUsed++;
		m_SphereBVHNode[leftChildIdx].firstPrimIdx = node.firstPrimIdx;
		m_SphereBVHNode[leftChildIdx].primCount = leftCount;
		m_SphereBVHNode[rightChildIdx].firstPrimIdx = i;
		m_SphereBVHNode[rightChildIdx].primCount = node.primCount - leftCount;
		node.leftNode = leftChildIdx;
		node.primCount = 0;
		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);

		// recurse
		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}

	bool Scene::IntersectSphereBVH(const Ray& ray, HitRecord& closestHit, const int nodeIdx, bool ignoreHitRecord) const
	{
		if (m_SphereGeometries.empty()) return false;
		const BVHNode& node = m_SphereBVHNode[nodeIdx];
		bool hit = false;

		if (!GeometryUtils::IntersectAABB(node.aabbMin, node.aabbMax, ray)) return false;

		if (node.isLeaf())
		{
			for (uint8_t i = 0; i < node.primCount; i++)
			{
				HitRecord tempHit{};
				if (GeometryUtils::HitTest_Sphere(m_SphereGeometries[m_SphereIndices[node.firstPrimIdx + i]], ray, tempHit, ignoreHitRecord))
				{
					hit = true;
					if (!ignoreHitRecord && tempHit.t < closestHit.t)
					{
						closestHit = tempHit;
					}
				}
			}
		}
		else
		{
			if (IntersectSphereBVH(ray, closestHit, node.leftNode, ignoreHitRecord))
				hit = true;
			if (IntersectSphereBVH(ray, closestHit, node.leftNode + 1, ignoreHitRecord))
				hit = true;
		}
		return hit;
	}
#pragma endregion

#pragma region Scene Helpers
	Sphere* Scene::AddSphere(const Vector3& origin, float radius, unsigned char materialIndex)
	{
		Sphere s;
		s.origin = origin;
		s.radius = radius;
		s.materialIndex = materialIndex;

		m_SphereGeometries.emplace_back(s);
		return &m_SphereGeometries.back();
	}

	Plane* Scene::AddPlane(const Vector3& origin, const Vector3& normal, unsigned char materialIndex)
	{
		Plane p;
		p.origin = origin;
		p.normal = normal;
		p.materialIndex = materialIndex;

		m_PlaneGeometries.emplace_back(p);
		return &m_PlaneGeometries.back();
	}

	TriangleMesh* Scene::AddTriangleMesh(TriangleCullMode cullMode, unsigned char materialIndex)
	{
		TriangleMesh m{};
		m.cullMode = cullMode;
		m.materialIndex = materialIndex;

		m_TriangleMeshGeometries.emplace_back(m);
		return &m_TriangleMeshGeometries.back();
	}

	/*
	Triangle* Scene::AddTriangle(const Vector3& v0, const Vector3& v1, const Vector3& v2, TriangleCullMode cullMode, unsigned char materialIndex)
	{
		Triangle t{ v0, v1, v2 };
		t.cullMode = cullMode;
		t.materialIndex = materialIndex;
		m_SingleTriangleGeometries.emplace_back(t);
		return &m_SingleTriangleGeometries.back();
	}
	*/

	Light* Scene::AddPointLight(const Vector3& origin, float intensity, const ColorRGB& color)
	{
		Light l;
		l.origin = origin;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Point;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	Light* Scene::AddDirectionalLight(const Vector3& direction, float intensity, const ColorRGB& color)
	{
		Light l;
		l.direction = direction;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Directional;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	unsigned char Scene::AddMaterial(Material* pMaterial)
	{
		m_Materials.push_back(pMaterial);
		return static_cast<unsigned char>(m_Materials.size() - 1);
	}
#pragma endregion
#pragma endregion

#pragma region SCENE W1
	void Scene_W1::Initialize()
	{
		//default: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Spheres
		AddSphere({ -25.f, 0.f, 100.f }, 50.f, matId_Solid_Red);
		AddSphere({ 25.f, 0.f, 100.f }, 50.f, matId_Solid_Blue);

		//Plane
		AddPlane({ -75.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 75.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, -75.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 75.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 125.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);
	}

	void Scene_W1::Update(dae::Timer* pTimer)
	{
		Scene::Update(pTimer);
		
		if (m_pSphere1)
		{
			const float speed = 5.f;
			m_pSphere1->origin.y = sinf(pTimer->GetTotal() * speed) * 50.f;
			m_pSphere1->origin.x = cosf(pTimer->GetTotal() * speed) * 50.f;
		}
	}
#pragma endregion

#pragma region SCENE W2
	void Scene_W2::Initialize()
	{
		//m_Camera.forward = { 0.266f, -0.453f, .860f };
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Spheres
		AddSphere({ -1.75f, 1.f, 00.f }, .75f, matId_Solid_Red);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matId_Solid_Blue);
		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matId_Solid_Blue);
		AddSphere({ 0.f, 3.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matId_Solid_Blue);


		//Plane
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);

		AddPointLight({ 0.f, 5.f, -5.f }, 70.f, colors::White);

		BuildSphereBVH();
	}
	void Scene_W2::Update(dae::Timer* pTimer)
	{
		Scene::Update(pTimer);
		const float deltaTime = pTimer->GetElapsed();

		for (Sphere& sphere : m_SphereGeometries)
		{
			sphere.origin.y += sinf(pTimer->GetTotal()) * deltaTime * .5f;
			sphere.origin.x += cosf(pTimer->GetTotal()) * deltaTime * .5f;
		}
		BuildSphereBVH();
		//todo W2
	}
#pragma endregion
#pragma region SCENE W3
	void Scene_W3_Test::Initialize()
	{
		m_Camera.origin = { 0.f, 1.f, -5.f };
		m_Camera.fovAngle = 45.f;

		const unsigned char matLambertPhong_Red = AddMaterial(new Material_LambertPhong{ colors::Red, 1.f, 0.f, 60.f });
		const unsigned char matLambertPhong_Blue = AddMaterial(new Material_LambertPhong{ colors::Blue, 1.f, 1.f, 60.f });
		const unsigned char matLambertPhong_Yellow = AddMaterial(new Material_LambertPhong{ colors::Yellow, 0.75f, 0.25f, 60.f });

		//Spheres
		AddSphere({ -.75f, 1.f, 0.f }, 1.f, matLambertPhong_Red);
		AddSphere({ .75f, 1.f, 0.f }, 1.f, matLambertPhong_Blue);

		//Plane
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f,0.f }, matLambertPhong_Yellow);

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 25.f, colors::White);
		AddPointLight({ 0.f, 2.5f, -5.f }, 25.f, colors::White);

		BuildSphereBVH();
	}
	void Scene_W3_Test::Update(dae::Timer* pTimer)
	{
		Scene::Update(pTimer);
	}

	void Scene_W3::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, true, 1.f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, true, .6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, true, .1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, false, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, false, .6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, false, .1f));

		const auto matLambertPhong_Blue = AddMaterial(new Material_LambertPhong{ colors::Blue, 1.f, 1.f, 60.f });

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert{ { .49f, .57f, .57f }, 1.f });

		//Plane
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matLambert_GrayBlue); //BACK
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //BOTTOOM
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matLambert_GrayBlue); //TOP
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matLambert_GrayBlue); //RIGHT
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matLambert_GrayBlue); //LEFT

		//Spheres
		AddSphere({ -1.75f, 1.f, 0.f }, .75f, matCT_GrayRoughMetal);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matCT_GrayMediumMetal);
		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matCT_GraySmoothMetal);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic);
		AddSphere({ 0.f, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic);

		AddSphere({ 0.f, 5.f, 0.f }, .75f, matLambertPhong_Blue);

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, ColorRGB{1.f, .61f, .45f });
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f });
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });

		BuildSphereBVH();
	}
	void Scene_W3::Update(dae::Timer* pTimer)
	{
		Scene::Update(pTimer);
	}

#pragma endregion
#pragma region SCENE W4
#pragma region TEST SCENE
	void Scene_W4_TestScene::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		const auto matLambert_Red = AddMaterial(new Material_Lambert{ colors::Red, 1.f });
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert{ { .49f, .57f, .57f }, 1.f });
		const auto matLambert_White = AddMaterial(new Material_Lambert{ colors::White, 1.f });

		//Plane
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matLambert_GrayBlue); //BACK
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //BOTTOOM
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matLambert_GrayBlue); //TOP
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matLambert_GrayBlue); //RIGHT
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matLambert_GrayBlue); //LEFT

		//Triangle
		//AddTriangle({ -2.75f, 2.5f, 0.f }, { 0.75f-2.f, 1.f, 0.f }, { -2.75f, 1.f, 0.f }, TriangleCullMode::NoCulling, matLambert_GrayBlue);
		//AddTriangle({ -.75f, 2.5f, 0.f }, { 0.75f, 1.f, 0.f }, { -.75f, 1.f, 0.f }, TriangleCullMode::FrontFaceCulling, matLambert_GrayBlue);
		//AddTriangle({ -.75f+2.f, 2.5f, 0.f }, { 2.75f, 1.f, 0.f }, { -.75f+2.f, 1.f, 0.f }, TriangleCullMode::BackFaceCulling, matLambert_GrayBlue);

		//TriangleMesh
		//const auto triangleMesh0 = AddTriangleMesh(TriangleCullMode::FrontFaceCulling, matLambert_White);
		//triangleMesh0->positions = { {-2.75f, 2.5f, 0.f}, {0.75f-2, 1.f, 0.f}, {-2.75f, 1.f, 0.f} };
		//triangleMesh0->indices = { 0,1,2 }; // Triangle 1
		//triangleMesh0->CalculateNormals();
		//triangleMesh0->UpdateTransforms();

		m_pMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/simple_cube.obj", m_pMesh->positions, m_pMesh->normals, m_pMesh->indices);

		m_pMesh->Scale({ .7f, .7f, .7f });
		m_pMesh->Translate({ 0.f,1.f,0.f });
		
		m_pMesh->UpdateTransforms();

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, .61f, .45f });
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f });
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });

		BuildSphereBVH();
	}
	void Scene_W4_TestScene::Update(dae::Timer* pTimer)
	{
		Scene::Update(pTimer);
		m_pMesh->RotateY(PI_DIV_2 * pTimer->GetTotal());
		m_pMesh->UpdateTransforms();
	}
#pragma endregion
#pragma region FINAL SCENES
	void Scene_W4_ReferenceScene::Initialize()
	{
		sceneName = "Reference Scene";
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, true, 1.f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, true, .6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, true, .1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, false, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, false, .6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, false, .1f));

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert{ { .49f, .57f, .57f }, 1.f });
		const auto matLambert_White = AddMaterial(new Material_Lambert{ colors::White, 1.f });

		//Plane
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matLambert_GrayBlue); //BACK
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //BOTTOOM
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matLambert_GrayBlue); //TOP
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matLambert_GrayBlue); //RIGHT
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matLambert_GrayBlue); //LEFT

		//Spheres
		AddSphere({ -1.75f, 1.f, 0.f }, .75f, matCT_GrayRoughMetal);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matCT_GrayMediumMetal);
		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matCT_GraySmoothMetal);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic);
		AddSphere({ 0.f, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic);

		BuildSphereBVH();

		//Triangle Meshes
		const Triangle baseTriangle{ Vector3{-.75, 1.5, 0.f}, Vector3{.75, 0.f, 0.f}, Vector3{-.75, 0.f, 0.f} };

		m_Meshes[0] = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		m_Meshes[0]->AppendTriangle(baseTriangle, true);
		m_Meshes[0]->Translate({ -1.75f, 4.5f, 0.f });
		m_Meshes[0]->UpdateTransforms();

		m_Meshes[1] = AddTriangleMesh(TriangleCullMode::FrontFaceCulling, matLambert_White);
		m_Meshes[1]->AppendTriangle(baseTriangle, true);
		m_Meshes[1]->Translate({ 0.f, 4.5f, 0.f });
		m_Meshes[1]->UpdateTransforms();

		m_Meshes[2] = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		m_Meshes[2]->AppendTriangle(baseTriangle, true);
		m_Meshes[2]->Translate({ 1.75f, 4.5f, 0.f });
		m_Meshes[2]->UpdateTransforms();

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, .61f, .45f });
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f });
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });
	}
	void Scene_W4_ReferenceScene::Update(dae::Timer* pTimer)
	{
		Scene::Update(pTimer);

		const auto yawAnge = (cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2;
		for (auto& mesh : m_Meshes)
		{
			mesh->RotateY(yawAnge);
			mesh->UpdateTransforms();
		}
	}

	void Scene_W4_BunnyScene::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		const auto matLambert_Red = AddMaterial(new Material_Lambert{ colors::Red, 1.f });
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert{ { .49f, .57f, .57f }, 1.f });
		const auto matLambert_White = AddMaterial(new Material_Lambert{ colors::White, 1.f });

		//Plane
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matLambert_GrayBlue); //BACK
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //BOTTOOM
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matLambert_GrayBlue); //TOP
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matLambert_GrayBlue); //RIGHT
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matLambert_GrayBlue); //LEFT

		//Bnuuy
		m_pBunnyMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/lowpoly_bunny.obj", m_pBunnyMesh->positions, m_pBunnyMesh->normals, m_pBunnyMesh->indices);

		m_pBunnyMesh->Scale({ 2.f,2.f,2.f });
		m_pBunnyMesh->RotateY(PI);

		m_pBunnyMesh->UpdateTransforms();

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, .61f, .45f });
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f });
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });
	}
	void Scene_W4_BunnyScene::Update(dae::Timer* pTimer)
	{
	}
#pragma endregion
#pragma endregion
}
