#pragma once
#define PI 3.1415926
#include <cmath>
namespace nsMath {
	template<typename T>
	class Vector2D {
	public:
		T x, y;
		Vector2D<T>();
		Vector2D<T>(T xp, T yp);
		~Vector2D<T>();
		Vector2D<T>& operator = (const Vector2D<T>& vec);
		Vector2D<T> operator + (const Vector2D<T>& vec) const;
		Vector2D<T> operator - (const Vector2D<T>& vec) const;
		Vector2D<T> operator * (const T val) const;
		Vector2D<T> operator / (const T val) const;
		Vector2D<T>& operator += (const Vector2D<T>& vec);
		Vector2D<T>& operator -= (const Vector2D<T>& vec);
		Vector2D<T>& operator *= (const T val);
		Vector2D<T>& operator /= (const T val);
		T Dot(const Vector2D<T>& vec);
		T Norm();
		Vector2D<T>& Normalize();
	};

	template<typename T>
	class Vector3D {
	public:
		T x, y, z;
		Vector3D<T>();
		Vector3D<T>(T xp, T yp, T zp);
		~Vector3D<T>();
		Vector3D<T>& operator = (const Vector3D<T>& vec);
		Vector3D<T> operator + (const Vector3D<T>& vec) const;
		Vector3D<T> operator - (const Vector3D<T>& vec) const;
		Vector3D<T> operator * (const T val) const;
		Vector3D<T> operator / (const T val) const;
		Vector3D<T>& operator += (const Vector3D<T>& vec);
		Vector3D<T>& operator -= (const Vector3D<T>& vec);
		Vector3D<T>& operator *= (const T val);
		Vector3D<T>& operator /= (const T val);
		T Dot(const Vector3D<T>& vec);
		T Norm();
		Vector3D<T>& Normalize();
	};

	template<typename T>
	void clamp(T *val, T min, T max) {
		if ((*val) > max) (*val) = max;
		if ((*val) < min) (*val) = min;
	}

	template<typename T>
	void warp(T *val, T min, T max) {
		if ((*val) > max) (*val) -= (max - min);
		if ((*val) < min) (*val) += (max - min);
	}

	template<typename T>
	Vector2D<T>::Vector2D() {
		this->x = this->y = 0;
	}

	template<typename T>
	Vector2D<T>::Vector2D(T xp, T yp)
	{
		this->x = xp;
		this->y = yp;
	}
	template<typename T>
	Vector2D<T>::~Vector2D() {
	}

	template<typename T>
	Vector2D<T>& Vector2D<T>::operator=(const Vector2D<T>& vec)
	{
		this->x = vec.x;
		this->y = vec.y;
		return *this;
	}

	template<typename T>
	Vector2D<T> Vector2D<T>::operator+(const Vector2D<T>& vec) const
	{
		return Vector2D<T>(vec.x + x, vec.y + y);
	}

	template<typename T>
	Vector2D<T> Vector2D<T>::operator-(const Vector2D<T>& vec) const
	{
		return Vector2D<T>(x - vec.x, y - vec.y);
	}

	template<typename T>
	Vector2D<T> Vector2D<T>::operator*(const T val) const
	{
		return Vector2D<T>(x*val, y*val);
	}

	template<typename T>
	Vector2D<T> Vector2D<T>::operator/(const T val) const
	{
		return Vector2D<T>(x / val, y / val);
	}

	template<typename T>
	Vector2D<T>& Vector2D<T>::operator+=(const Vector2D<T>& vec)
	{
		this->x += vec.x;
		this->y += vec.y;
		return *this;
	}

	template<typename T>
	Vector2D<T>& Vector2D<T>::operator-=(const Vector2D<T>& vec)
	{
		this->x -= vec.x;
		this->y -= vec.y;
		return *this;
	}

	template<typename T>
	Vector2D<T>& Vector2D<T>::operator*=(const T val)
	{
		this->x *= val;
		this->y *= val;
		return *this;
	}

	template<typename T>
	Vector2D<T>& Vector2D<T>::operator/=(const T val)
	{
		this->x /= val;
		this->y /= val;
		return *this;
	}

	template<typename T>
	T Vector2D<T>::Dot(const Vector2D<T>& vec)
	{
		return x * vec.x + y * vec.y;
	}

	template<typename T>
	T Vector2D<T>::Norm()
	{
		return sqrt(x*x + y * y);
	}

	template<typename T>
	Vector2D<T>& Vector2D<T>::Normalize()
	{
		this /= this->Norm();
		return *this;
	}


	///------------------------------


	template<typename T>
	Vector3D<T>::Vector3D() {
		this->x = this->y = this->z = 0;
	}
	template<typename T>
	Vector3D<T>::Vector3D(T xp, T yp, T zp)
	{
		this->x = xp;
		this->y = yp;
		this->z = zp;
	}
	template<typename T>
	Vector3D<T>::~Vector3D() {
	}

	template<typename T>
	Vector3D<T>& Vector3D<T>::operator=(const Vector3D<T>& vec)
	{
		this->x = vec.x;
		this->y = vec.y;
		this->z = vec.z;
		return *this;
	}

	template<typename T>
	Vector3D<T> Vector3D<T>::operator+(const Vector3D<T>& vec) const
	{
		return Vector3D<T>(vec.x + x, vec.y + y, vec.z + z);
	}

	template<typename T>
	Vector3D<T> Vector3D<T>::operator-(const Vector3D<T>& vec) const
	{
		return Vector3D<T>(vec.x - x, vec.y - y, vec.z - z);
	}

	template<typename T>
	Vector3D<T> Vector3D<T>::operator*(const T val) const
	{
		return Vector3D<T>(x*val, y*val, z*val);
	}

	template<typename T>
	Vector3D<T> Vector3D<T>::operator/(const T val) const
	{
		return Vector3D<T>(x / val, y / val, z / val);
	}

	template<typename T>
	Vector3D<T>& Vector3D<T>::operator+=(const Vector3D<T>& vec)
	{
		this->x += vec.x;
		this->y += vec.y;
		this->z += vec.z;
		return *this;
	}

	template<typename T>
	Vector3D<T>& Vector3D<T>::operator-=(const Vector3D<T>& vec)
	{
		this->x -= vec.x;
		this->y -= vec.y;
		this->z -= vec.z;
		return *this;
	}

	template<typename T>
	Vector3D<T>& Vector3D<T>::operator*=(const T val)
	{
		this->x *= val;
		this->y *= val;
		this->z *= val;
		return *this;
	}

	template<typename T>
	Vector3D<T>& Vector3D<T>::operator/=(const T val)
	{
		this->x /= val;
		this->y /= val;
		this->z / .val;
		return *this;
	}

	template<typename T>
	T Vector3D<T>::Dot(const Vector3D<T>& vec)
	{
		return x * vec.x + y * vec.y + z * vec.z;
	}

	template<typename T>
	T Vector3D<T>::Norm()
	{
		return sqrt(x*x + y * y + z * z);
	}

	template<typename T>
	Vector3D<T>& Vector3D<T>::Normalize()
	{
		this /= this->Norm();
		return *this;
	}

}