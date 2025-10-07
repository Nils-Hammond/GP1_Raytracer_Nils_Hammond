#pragma once
#include <string>
#include <vector>
#include "Math.h"
#include "DataTypes.h"
#include "Camera.h"

namespace dae
{
	//Forward Declarations
	class Timer;
	class Material;
	struct Plane;
	struct Sphere;
	struct Light;

	//Scene Base Class
	class Scene
	{
	public:
		Scene();
		virtual ~Scene();

		Scene(const Scene&) = delete;
		Scene(Scene&&) noexcept = delete;
		Scene& operator=(const Scene&) = delete;
		Scene& operator=(Scene&&) noexcept = delete;

		virtual void Initialize() = 0;
		virtual void Update(dae::Timer* pTimer)
		{
			m_Camera.Update(pTimer);
		}

		Camera& GetCamera() { return m_Camera; }
		void GetClosestHit(const Ray& ray, HitRecord& closestHit) const;
		bool DoesHit(const Ray& ray) const;

		const std::vector<Plane>& GetPlaneGeometries() const { return m_PlaneGeometries; }
		const std::vector<Sphere>& GetSphereGeometries() const { return m_SphereGeometries; }
		const std::vector<Light>& GetLights() const { return m_Lights; }
		const std::vector<Material*> GetMaterials() const { return m_Materials; }

	protected:
		std::string	sceneName;

		std::vector<Plane> m_PlaneGeometries{};
		std::vector<Sphere> m_SphereGeometries{};
		std::vector<TriangleMesh> m_TriangleMeshGeometries{};
		std::vector<Light> m_Lights{};
		std::vector<Material*> m_Materials{};

		Camera m_Camera{};

		Sphere* AddSphere(const Vector3& origin, float radius, unsigned char materialIndex = 0);
		Plane* AddPlane(const Vector3& origin, const Vector3& normal, unsigned char materialIndex = 0);
		TriangleMesh* AddTriangleMesh(TriangleCullMode cullMode, unsigned char materialIndex = 0);

		Light* AddPointLight(const Vector3& origin, float intensity, const ColorRGB& color);
		Light* AddDirectionalLight(const Vector3& direction, float intensity, const ColorRGB& color);
		unsigned char AddMaterial(Material* pMaterial);

		std::vector<BVHNode> m_SphereBVHNode{};
		uint8_t rootNodeIdx{};
		uint8_t nodesUsed{ 1 };
		std::vector<uint8_t> m_SphereIndices{};

		void BuildSphereBVH();
		void UpdateNodeBounds(int nodeIdx);
		void Subdivide(int nodeIdx);
		bool IntersectSphereBVH(const Ray& ray, HitRecord& closestHit, const int nodeIdx, bool ignoreHitRecord = false) const;
	};

	//+++++++++++++++++++++++++++++++++++++++++
	//WEEK 1 Test Scene
	class Scene_W1 final : public Scene
	{
	public:
		Scene_W1() = default;
		~Scene_W1() override = default;

		void Update(dae::Timer* pTimer);

		Scene_W1(const Scene_W1&) = delete;
		Scene_W1(Scene_W1&&) noexcept = delete;
		Scene_W1& operator=(const Scene_W1&) = delete;
		Scene_W1& operator=(Scene_W1&&) noexcept = delete;

		void Initialize() override;
	private:
		Sphere* m_pSphere1{};
	};

	//+++++++++++++++++++++++++++++++++++++++++
	//WEEK 2 Test Scene
	class Scene_W2 final : public Scene
	{
	public:
		Scene_W2() = default;
		~Scene_W2() override = default;

		void Update(dae::Timer* pTimer) override;

		Scene_W2(const Scene_W2&) = delete;
		Scene_W2(Scene_W2&&) noexcept = delete;
		Scene_W2& operator=(const Scene_W2&) = delete;
		Scene_W2& operator=(Scene_W2&&) noexcept = delete;

		void Initialize() override;
	};

	//+++++++++++++++++++++++++++++++++++++++++
	//WEEK 3 Test Scene
	class Scene_W3_Test final : public Scene
	{
	public:
		Scene_W3_Test() = default;
		~Scene_W3_Test() override = default;
		void Update(dae::Timer* pTimer) override;
		Scene_W3_Test(const Scene_W3_Test&) = delete;
		Scene_W3_Test(Scene_W3_Test&&) noexcept = delete;
		Scene_W3_Test& operator=(const Scene_W3_Test&) = delete;
		Scene_W3_Test& operator=(Scene_W3_Test&&) noexcept = delete;
		void Initialize() override;
	};

	class Scene_W3 final : public Scene
	{
	public:
		Scene_W3() = default;
		~Scene_W3() override = default;
		void Update(dae::Timer* pTimer) override;
		Scene_W3(const Scene_W3&) = delete;
		Scene_W3(Scene_W3&&) noexcept = delete;
		Scene_W3& operator=(const Scene_W3&) = delete;
		Scene_W3& operator=(Scene_W3&&) noexcept = delete;
		void Initialize() override;
	};
};