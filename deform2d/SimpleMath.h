#pragma once

#include <vector>

#if 0
template <typename T>
class vector: public std::vector<T> {
public:
	vector(int size): std::vector<T>(size) {}

private:
	vector& operator=(const vector& other);
};
#else
template <typename T>
using vector = std::vector<T>;
#endif

struct Vector2 {
	float x;
	float y;

	float& operator[](int index) {
		switch (index) {
		case 0:
			return x;
		case 1:
			return y;
		}
	}

	Vector2 operator+(const Vector2& other) {
		return { this->x + other.x, this->y + other.y };
	}

	Vector2 operator-(const Vector2& other) {
		return { this->x - other.x, this->y - other.y };
	}

	Vector2 operator-() {
		return { -this->x, -this->y };
	}

	Vector2 operator*(float other) {
		return { this->x * other, this->y * other };
	}

	float Dot(const Vector2& other) {
		return this->x * other.x + this->y * other.y;
	}

	float SqrMagnitude() {
		return x * x + y * y;
	}

	float Magnitude() {
		return std::sqrt(SqrMagnitude());
	}

	Vector2 Normalized() {
		float magnitude = Magnitude();

		if (magnitude == 0) {
			return { 0, 0 };
		}
		else {
			return { x / magnitude, y / magnitude };
		}
	}

private:
	//Vector2& operator=(const Vector2& other);
};

// =================================================================

struct Vector3 {
	float x;
	float y;
	float z;

	float& operator[](int index) {
		switch (index) {
		case 0:
			return x;
		case 1:
			return y;
		default:
			return z;
		}
	}

	Vector3 operator+(const Vector3& other) {
		return { this->x + other.x, this->y + other.y, this->z + other.z };
	}

	Vector3 operator-(const Vector3& other) {
		return { this->x - other.x, this->y - other.y, this->z - other.z };
	}

	Vector3 operator-() {
		return { -this->x, -this->y, -this->z };
	}

	Vector3 operator*(float other) {
		return { this->x * other, this->y * other, this->z * other };
	}

	float Dot(const Vector3& other) {
		return this->x * other.x + this->y * other.y + this->z * other.z;
	}

	float SqrMagnitude() {
		return x * x + y * y + z * z;
	}

	float Magnitude() {
		return std::sqrt(SqrMagnitude());
	}

	Vector3 Normalized() {
		float magnitude = Magnitude();

		if (magnitude == 0) {
			return { 0, 0 };
		}
		else {
			return { x / magnitude, y / magnitude, z / magnitude };
		}
	}

private:
	//Vector3& operator=(const Vector3& other);
};

// =================================================================

class Vector {
public:
	Vector(int size = 0) {
		SetSize(size);
		Allocate();
	}

	void SetSize(int size) {
		m_size = size;
	}

	int GetSize() {
		return m_size;
	}

	float& operator[](int index) {
		return m_items[index];
	}

	Vector operator*(float other) {
		Vector result = Vector(m_size);

		for (int i = 0; i < m_size; i++) {
			result.m_items[i] = m_items[i] * other;
		}

		return result;
	}

	Vector operator+(Vector& other) {
		Vector result = Vector(m_size);

		for (int i = 0; i < m_size; i++) {
			result.m_items[i] = m_items[i] + other[i];
		}

		return result;
	}

	Vector operator-() {
		Vector result = Vector(m_size);

		for (int i = 0; i < m_size; i++) {
			result.m_items[i] = -m_items[i];
		}

		return result;
	}

	float Dot(Vector& other) {
		float result = 0;

		for (int i = 0; i < m_size; i++) {
			result += m_items[i] * other[i];
		}

		return result;
	}

private:
	void Allocate() {
		m_items = vector<float>(m_size);
	}

	//Vector& operator=(const Vector other);

private:
	int m_size;
	vector<float> m_items;
};

// =================================================================

class Matrix {
public:
	Matrix(int rowCount = 0, int colCount = 0) {
		SetSize(rowCount, colCount);
		Allocate();
	}

	void SetSize(int rowCount, int colCount) {
		m_rowCount = rowCount;
		m_colCount = colCount;
	}

	int GetRows() {
		return m_rowCount;
	}

	int GetColumns() {
		return m_colCount;
	}

	vector<float>& operator[](int rowIndex) {
		return m_items[rowIndex];
	}

	Matrix operator*(float other) {
		Matrix result = Matrix(m_rowCount, m_colCount);

		for (int i = 0; i < m_rowCount; i++) {
			for (int j = 0; j < m_colCount; j++) {
				result.m_items[i][j] = m_items[i][j] * other;
			}
		}

		return result;
	}

	Vector operator*(Vector& other) {
		Vector result = Vector(m_rowCount);

		for (int i = 0; i < m_rowCount; i++) {
			float newItem = 0;

			for (int j = 0; j < m_colCount; j++) {
				newItem += m_items[i][j] * other[j];
			}

			result[i] = newItem;
		}

		return result;
	}

	Matrix operator*(Matrix& other) {
		Matrix result = Matrix(m_rowCount, other.m_colCount);

		for (int i = 0; i < m_rowCount; i++) {
			for (int j = 0; j < other.m_colCount; j++) {
				float item = 0;

				for (int k = 0; k < m_colCount; k++) {
					item += m_items[i][k] * other[k][j];
				}

				result.m_items[i][j] = item;
			}
		}

		return result;
	}

	Matrix operator+(Matrix& other) {
		Matrix result = Matrix(m_rowCount, m_colCount);

		for (int i = 0; i < m_rowCount; i++) {
			for (int j = 0; j < m_colCount; j++) {
				result.m_items[i][j] = m_items[i][j] + other[i][j];
			}
		}

		return result;
	}

	Matrix operator-() {
		Matrix result = Matrix(m_rowCount, m_colCount);

		for (int i = 0; i < m_rowCount; i++) {
			for (int j = 0; j < m_colCount; j++) {
				result.m_items[i][j] = -m_items[i][j];
			}
		}

		return result;
	}

	Matrix Transpose() {
		Matrix result = Matrix(m_colCount, m_rowCount);

		for (int i = 0; i < m_colCount; i++) {
			for (int j = 0; j < m_rowCount; j++) {
				result.m_items[i][j] = m_items[j][i];
			}
		}

		return result;
	}

	Matrix Inverse() {
		int iSize = GetRows();
		Matrix rkInvA = Matrix(iSize, iSize);

		for (int i = 0; i < iSize; i++) {
			for (int j = 0; j < iSize; j++) {
				rkInvA[i][j] = m_items[i][j];
			}
		}

		vector<int> aiColIndex(iSize);
		vector<int> aiRowIndex(iSize);
		vector<bool> abPivoted(iSize);

		int i1, i2, iRow = 0, iCol = 0;
		float fSave;

		// elimination by full pivoting
		for (int i0 = 0; i0 < iSize; i0++)
		{
			// search matrix (excluding pivoted rows) for maximum absolute entry
			float fMax = 0.0f;
			for (i1 = 0; i1 < iSize; i1++)
			{
				if (!abPivoted[i1])
				{
					for (i2 = 0; i2 < iSize; i2++)
					{
						if (!abPivoted[i2])
						{
							float fAbs = fabs(rkInvA[i1][i2]);

							if (fAbs > fMax)
							{
								fMax = fAbs;
								iRow = i1;
								iCol = i2;
							}
						}
					}
				}
			}

			if (fMax == 0.0f)
			{
				// matrix is not invertible
				return rkInvA;
			}

			abPivoted[iCol] = true;

			// swap rows so that A[iCol][iCol] contains the pivot entry
			if (iRow != iCol) {
				auto row1 = rkInvA[iRow];
				auto row2 = rkInvA[iCol];
				rkInvA[iRow] = row2;
				rkInvA[iCol] = row1;
			}

			// keep track of the permutations of the rows
			aiRowIndex[i0] = iRow;
			aiColIndex[i0] = iCol;

			// scale the row so that the pivot entry is 1
			float fInv = 1.0f / rkInvA[iCol][iCol];
			rkInvA[iCol][iCol] = 1.0f;
			for (i2 = 0; i2 < iSize; i2++)
				rkInvA[iCol][i2] *= fInv;

			// zero out the pivot column locations in the other rows
			for (i1 = 0; i1 < iSize; i1++)
			{
				if (i1 != iCol)
				{
					fSave = rkInvA[i1][iCol];
					rkInvA[i1][iCol] = 0.0f;
					for (i2 = 0; i2 < iSize; i2++)
						rkInvA[i1][i2] -= rkInvA[iCol][i2] * fSave;
				}
			}
		}

		// reorder rows so that A[][] stores the inverse of the original matrix
		for (i1 = iSize - 1; i1 >= 0; i1--)
		{
			if (aiRowIndex[i1] != aiColIndex[i1])
			{
				for (i2 = 0; i2 < iSize; i2++)
				{
					fSave = rkInvA[i2][aiRowIndex[i1]];
					rkInvA[i2][aiRowIndex[i1]] = rkInvA[i2][aiColIndex[i1]];
					rkInvA[i2][aiColIndex[i1]] = fSave;
				}
			}
		}

		return rkInvA;
	}

private:
	void Allocate() {
		m_items = vector<vector<float>>(m_rowCount);

		for (int i = 0; i < m_rowCount; i++) {
			m_items[i] = vector<float>(m_colCount, 0.0f);
		}
	}

private:
	vector<vector<float>> m_items;
	int m_rowCount;
	int m_colCount;
};
