//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();
	float aspectRatio = m_Width / (static_cast<float>(m_Height));
	float fov = tanf((camera.fovAngle * TO_RADIANS) / 2.f);
	
	const Matrix cameraToWorld = camera.CalculateCameraToWorld();

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			float ndcx = (2 * (px + 0.5f) / m_Width - 1) * aspectRatio * fov;
			float ndcy = (1 - 2 * (py + 0.5f) / m_Height) * fov;
			Vector3 rayDirection = Vector3{ ndcx, ndcy, 1 }.Normalized();
			rayDirection = cameraToWorld.TransformVector(rayDirection).Normalized();

			Ray hitRay{ camera.origin, rayDirection};

			ColorRGB finalColor;//{ rayDirection.x, rayDirection.y, rayDirection.z };

			HitRecord closestHit{};

			pScene->GetClosestHit(hitRay, closestHit);
			
			if (closestHit.didHit)
			{
				Vector3 lightDir = LightUtils::GetDirectionToLight(lights[0], closestHit.origin);
				Ray shadowRay{ closestHit.origin, lightDir.Normalized(), 0.0001f, lightDir.Magnitude()};

				finalColor = materials[closestHit.materialIndex]->Shade();

				if (pScene->DoesHit(shadowRay))
				{
					finalColor *= 0.5f;
				}
				//const float scaled_t = closestHit.t / 500.f;
				//finalColor = { scaled_t, scaled_t, scaled_t };
			}

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}
