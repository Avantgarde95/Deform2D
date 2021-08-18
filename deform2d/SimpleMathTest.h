#pragma once

#include "SimpleMath.h"

#include <iostream>
#include <sstream>

std::string vectorToString(Vector& v) {
	std::stringstream ss;
	ss << "(";

	for (int i = 0; i < v.GetSize(); i++) {
		ss << v[i];

		if (i < v.GetSize() - 1) {
			ss << ", ";
		}
	}

	ss << ")";
	return ss.str();
}

std::string matrixToString(Matrix& m) {
	std::stringstream ss;

	ss << "(";

	for (int i = 0; i < m.GetRows(); i++) {
		for (int j = 0; j < m.GetColumns(); j++) {
			ss << m[i][j];

			if (j < m.GetColumns() - 1) {
				ss << ", ";
			}
		}

		if (i < m.GetRows() - 1) {
			ss << " / ";
		}
	}

	ss << ")";
	return ss.str();
}

void testSimpleMath1() {
	Vector v1 = Vector(5);

	v1[0] = 3;
	v1[3] = 2;
	std::cout << "Set v1[0] = 3 and v1[3] = 2: " << vectorToString(v1) << std::endl;

	Vector v2 = Vector(5);
	v2[0] = 4;
	v2[4] = 2;
	v1 = v1 + v2;
	std::cout << "v2 = (4, 0, 0, 0, 2) and v1 = v1 + v2: " << vectorToString(v1) << std::endl;

	v1 = -(v1 * 3);
	std::cout << "v1 = -(v1 * 3): " << vectorToString(v1) << std::endl;

	Vector v3 = Vector(4);
	v3[0] = 3;
	v1 = v3;
	std::cout << "v3 = (3, 0, 0, 0) and v1 = v3: " << vectorToString(v1) << std::endl;

	Vector v4 = v3.Dot(v3);
	std::cout << "v4 = v3 dot v3: " << vectorToString(v4) << std::endl;

	Vector v5 = v4;
	v4[0] = 100;
	std::cout << "v5 = v4 and v4[0] = 100: " << vectorToString(v4) << ", " << vectorToString(v5) << std::endl;
}

void testSimpleMath2() {
	Matrix m1 = Matrix(4, 3);

	m1[0][0] = 3;
	m1[1][1] = 4;
	m1[1][2] = 5;
	m1[3][1] = 6;
	std::cout << "Set m1[0][0] = 3, m1[1][1] = 4, m1[1][2] = 5 and m1[3][1] = 6: " << matrixToString(m1) << std::endl;

	m1 = -(m1 * 2);
	std::cout << "m1 = -(m1 * 2): " << matrixToString(m1) << std::endl;

	Matrix m2 = m1.Transpose();
	std::cout << "m2 = m1.Transpose(): " << matrixToString(m2) << std::endl;

	Matrix m3 = Matrix(3, 2);
	m3[0][0] = 3;
	m3[0][1] = 2;
	m3[1][0] = 4;
	m3[2][1] = 5;
	m1 = m1 * m3;
	std::cout << "Set m3[0][0] = 3, m3[0][1] = 2, m3[1][0] = 4, m3[2][1] = 5 and m1 = m1 * m3: " << matrixToString(m1) << std::endl;

	Vector v1 = Vector(3);
	v1[0] = 1;
	v1[1] = 2;
	v1[2] = 3;
	Vector v2 = m1 * v1;
	std::cout << "v1 = (1, 2, 3) and v2 = m1 * v1: " << vectorToString(v2) << std::endl;

	Matrix m4 = m3;
	m4[0][0] = 100;
	m3[0][1] = 200;
	std::cout << "m4 = m3 and m4[0][0] = 100 and m3[0][1] = 200: " << matrixToString(m3) << ", " << matrixToString(m4) << std::endl;

	Matrix m5 = Matrix(4, 4);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m5[i][j] = i + j * j + ((i == j) ? 5 : 1);
		}
	}

	std::cout << "Generate m5: " << matrixToString(m5) << std::endl;

	Matrix m6 = m5 * m5.Inverse();
	std::cout << "m6 = m5 * m5.Inverse(): " << matrixToString(m5) << ", " << matrixToString(m5.Inverse()) << ", " << matrixToString(m6) << std::endl;
}
