#pragma once
#include <SDL_keyboard.h>
#include <SDL_mouse.h>
#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera final
	{
		Camera() = default;
		Camera(const Vector3& _origin, float _fovAngle) :
			origin{ _origin },
			fovAngle{ _fovAngle }
		{}

		Vector3 origin{};
		float fovAngle{ 90.f };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		float totalPitch{ 0.f };
		float totalYaw{ 0.f };

		float movementSpeed{ 2.5f };
		float rotationSpeed{ 0.1f };

		Matrix cameraToWorld{};

		Matrix CalculateCameraToWorld()
		{
			//todo: W2
			right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
			up = Vector3::Cross(forward, right).Normalized();
			cameraToWorld = Matrix(right, up, forward, origin);
			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
			if (pKeyboardState[SDL_SCANCODE_W])
				origin += forward * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_S])
				origin -= forward * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_A])
				origin -= right * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_D])
				origin += right * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_SPACE])
				origin += up * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_LCTRL])
				origin -= up * movementSpeed * deltaTime;

			if (pKeyboardState[SDL_SCANCODE_LEFT])
				fovAngle -= 10.f * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_RIGHT])
				fovAngle += 10.f * deltaTime;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			if (mouseState == (SDL_BUTTON(SDL_BUTTON_LEFT) | SDL_BUTTON(SDL_BUTTON_RIGHT)))
			{
				origin += up * static_cast<float>(mouseY) * movementSpeed * deltaTime;
				origin -= right * static_cast<float>(mouseX) * movementSpeed * deltaTime;
			}
			else if (mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT))
			{
				totalYaw += mouseX * rotationSpeed;
				totalPitch -= mouseY * rotationSpeed;
				/*
				if (totalPitch > 89.f)
					totalPitch = 89.f;
				else if (totalPitch < -89.f)
					totalPitch = -89.f;
					*/
				const float yawRad = totalYaw * TO_RADIANS;
				const float pitchRad = totalPitch * TO_RADIANS;
				Matrix rotation = Matrix::CreateRotation(pitchRad, yawRad, 0.f);
				forward = rotation.TransformVector(Vector3::UnitZ).Normalized();
			}
			else if (mouseState & SDL_BUTTON(SDL_BUTTON_LEFT))
			{
				origin -= forward * static_cast<float>(mouseY) * movementSpeed * deltaTime;
			}
		}
	};
}
