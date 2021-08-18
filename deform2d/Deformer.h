#pragma once

#include <vector>
#include <set>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <sstream>

#include "SimpleMath.h"

using Time = std::chrono::steady_clock::time_point;

constexpr bool measureDuration = true;

Time getTime() {
	if (measureDuration) {
		return std::chrono::steady_clock::now();
	}
	else {
		// Do nothing.
	}
}

int getDuration(const Time& time1, const Time& time2) {
	if (measureDuration) {
		return std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count();
	}
	else {
		// Do nothing.
	}
}

void printDuration(const std::string& name, const Time& time1, const Time& time2) {
	if (measureDuration) {
		std::stringstream ss;
		ss << std::fixed << name << ": " << getDuration(time1, time2) / 1000000.0f << "s" << std::endl;
		std::cout << ss.str();
	}
	else {
		// Do nothing.
	}
}

struct LUData {
	Matrix mLU;
	std::vector<int> vPivots;
};

struct Vertex {
	Vector2 vPosition;
};

struct Triangle {
	unsigned int nVerts[3];

	// definition of each vertex in triangle-local coordinate system
	Vector2 vTriCoords[3];

	// un-scaled triangle
	Vector2 vScaled[3];

	// pre-computed matrices for triangle scaling step
	Matrix mF, mC;
};

struct Constraint {
	unsigned int nVertex;
	Vector2 vConstrainedPos;

	Constraint() {
		nVertex = 0;
		vConstrainedPos = { 0, 0 };
	}

	Constraint(unsigned int nVert, const Vector2& vPos) {
		nVertex = nVert;
		vConstrainedPos = vPos;
	}

	bool operator<(const Constraint& c2) const {
		return nVertex < c2.nVertex;
	}
};

void ExtractSubMatrix(Matrix& mFrom, int nRowOffset, int nColOffset, Matrix& mTo) {
	int nRows = mTo.GetRows();
	int nCols = mTo.GetColumns();

	for (int i = 0; i < nRows; ++i) {
		for (int j = 0; j < nCols; ++j) {
			mTo[i][j] = mFrom[i + nRowOffset][j + nColOffset];
		}
	}
}

bool LUDecompose(Matrix& mMatrix, LUData& vDecomposition)
{
	int nRows = mMatrix.GetRows();
	if (nRows != mMatrix.GetColumns())
		return false;

	vDecomposition.vPivots.resize(nRows);
	vDecomposition.mLU = Matrix(nRows, nRows);
	std::vector<int>& vPivots = vDecomposition.vPivots;
	Matrix& mLUMatrix = vDecomposition.mLU;

	mLUMatrix = mMatrix;

	float dRowSwaps = 1;

	// scaling of each row
	std::vector<float> vScale(nRows, 0);

	float dTemp;
	for (int i = 0; i < nRows; ++i) {			// find scaling for each row
		float dLargest = (float)0;
		for (int j = 0; j < nRows; ++j) {
			if ((dTemp = (float)abs(mLUMatrix[i][j])) > dLargest)
				dLargest = dTemp;
		}
		if (dLargest == 0)
			return false;
		vScale[i] = (float)1.0 / dLargest;
	}

	int niMax = 0;
	for (int j = 0; j < nRows; ++j) {		// loop over columns (Crout's method)

		// not entirely sure
		for (int i = 0; i < j; ++i) {
			float dSum = mLUMatrix[i][j];
			for (int k = 0; k < i; ++k)
				dSum -= mLUMatrix[i][k] * mLUMatrix[k][j];
			mLUMatrix[i][j] = dSum;
		}

		// find largest pivot element
		float dLargestPivot = (float)0;
		for (int i = j; i < nRows; ++i) {
			float dSum = mLUMatrix[i][j];
			for (int k = 0; k < j; ++k)
				dSum -= mLUMatrix[i][k] * mLUMatrix[k][j];
			mLUMatrix[i][j] = dSum;
			if ((dTemp = vScale[i] * (float)fabs(dSum)) > dLargestPivot) {
				dLargestPivot = dTemp;
				niMax = i;
			}
		}

		// swap rows if pivot is in another column
		if (j != niMax) {
			for (int k = 0; k < nRows; ++k) {
				float dSwap = mLUMatrix[niMax][k];
				mLUMatrix[niMax][k] = mLUMatrix[j][k];
				mLUMatrix[j][k] = dSwap;
			}
			dRowSwaps = -dRowSwaps;
			vScale[niMax] = vScale[j];
		}

		vPivots[j] = niMax;
		if (mLUMatrix[j][j] == 0)
			mLUMatrix[j][j] = 1.192092896e-07F;

		if (j != nRows - 1) {
			float dScale = (float)1.0 / mLUMatrix[j][j];
			for (int i = j + 1; i < nRows; ++i)
				mLUMatrix[i][j] *= dScale;
		}
	}

	return true;
}


// =================================================================

class Deformer
{
public:
	Deformer() {
		InvalidateSetup();
	}

	void ForceValidation() {
		ValidateSetup();
	}

	void RemoveHandle(unsigned int nHandle) {
		Constraint c(nHandle, { 0, 0 });
		m_vConstraints.erase(c);
		m_vDeformedVerts[nHandle].vPosition = m_vInitialVerts[nHandle].vPosition;
		InvalidateSetup();
	}

	/*
	 * interface stuff
	 */
	 //unsigned int GetNumHandles();

	 //const Vector2 & GetInitialHandle(unsigned int nHandle);
	 //const Vector2 & GetDeformedHandle( unsigned int nHandle );

	 //! nHandle is vertex ID
	void SetDeformedHandle(unsigned int nHandle, const Vector3* vHandle) {
		Constraint c(nHandle, { vHandle->x, vHandle->y });
		UpdateConstraint(c);
	}

	/*
	 * mesh handling
	 */
	void SetMesh(
		Vector3* vertices,
		unsigned int vertexCount,
		unsigned int* faces,
		unsigned int faceCount
	) {
		auto setMeshStart = getTime();

		m_vConstraints.clear();
		m_vInitialVerts.resize(0);
		m_vDeformedVerts.resize(0);
		m_vTriangles.resize(0);

		// copy vertices
		for (unsigned int i = 0; i < vertexCount; ++i) {
			Vertex v;
			v.vPosition = { vertices[i].x, vertices[i].y };
			m_vInitialVerts.push_back(v);
			m_vDeformedVerts.push_back(v);
		}

		// copy triangles
		for (unsigned int i = 0; i < faceCount; ++i) {
			Triangle t;
			t.nVerts[0] = faces[i * 3];
			t.nVerts[1] = faces[i * 3 + 1];
			t.nVerts[2] = faces[i * 3 + 2];
			m_vTriangles.push_back(t);
		}

		// set up triangle-local coordinate systems
		for (unsigned int i = 0; i < faceCount; ++i) {
			Triangle& t = m_vTriangles[i];

			for (int j = 0; j < 3; ++j) {
				unsigned int n0 = j;
				unsigned int n1 = (j + 1) % 3;
				unsigned int n2 = (j + 2) % 3;

				Vector2 v0 = GetInitialVert(t.nVerts[n0]);
				Vector2 v1 = GetInitialVert(t.nVerts[n1]);
				Vector2 v2 = GetInitialVert(t.nVerts[n2]);

				// find coordinate system
				Vector2 v01(v1 - v0);
				Vector2 v01N = v01.Normalized();
				Vector2 v01Rot90 = { v01.y, -v01.x };
				Vector2 v01Rot90N = v01Rot90.Normalized();

				// express v2 in coordinate system
				Vector2 vLocal(v2 - v0);
				float fX = vLocal.Dot(v01) / v01.SqrMagnitude();
				float fY = vLocal.Dot(v01Rot90) / v01Rot90.SqrMagnitude();

				// sanity check
				Vector2 v2test(v0 + v01 * fX + v01Rot90 * fY);
				float fLength = (v2test - v2).Magnitude();
				if (fLength > 0.001f)
					DebugBreak();

				t.vTriCoords[j] = { fX, fY };
			}
		}

		auto setMeshEnd = getTime();
		printDuration("SetMesh", setMeshStart, setMeshEnd);

	}

	void GetDeformedMesh(
		Vector3* vertices,
		unsigned int vertexCount,
		bool isRigid
	) {
		auto validateStart = getTime();
		ValidateDeformedMesh(isRigid);
		auto validateEnd = getTime();
		printDuration("ValidateDeformedMesh", validateStart, validateEnd);

		std::vector<Vertex>& vVerts = (m_vConstraints.size() > 1) ? m_vDeformedVerts : m_vInitialVerts;

		for (unsigned int i = 0; i < vertexCount; ++i) {
			Vector2 vNewPos = vVerts[i].vPosition;
			vertices[i].x = vNewPos.x;
			vertices[i].y = vNewPos.y;
		}
	}

	/*
	 * debug
	 */
	const Vector2* GetTriangleVerts(unsigned int nTriangle) {
		return m_vTriangles[nTriangle].vScaled;
	}

protected:
	void UpdateConstraint(Constraint& cons) {
		std::set<Constraint>::iterator found(m_vConstraints.find(cons));
		if (found != m_vConstraints.end()) {

			const_cast<Vector2&>((*found).vConstrainedPos) = cons.vConstrainedPos;
			m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;

		}
		else {
			m_vConstraints.insert(cons);
			m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;
			InvalidateSetup();
		}
	}

	void InvalidateSetup() {
		m_bSetupValid = false;
	}

	void ValidateSetup() {
		if (m_bSetupValid || m_vConstraints.size() < 2)
			return;

		std::printf("Computing matrices for mesh with %d verts....this might take a while...\n", m_vInitialVerts.size());

		auto orientationStart = getTime();
		PrecomputeOrientationMatrix();
		auto orientationEnd = getTime();
		printDuration("PrecomputeOrientationMatrix", orientationStart, orientationEnd);

		auto scalingStart = getTime();
		// ok, now scale triangles
		size_t nTris = m_vTriangles.size();
		for (unsigned int i = 0; i < nTris; ++i)
			PrecomputeScalingMatrices(i);
		auto scalingEnd = getTime();
		printDuration("PrecomputeScalingMatrix", scalingStart, scalingEnd);

		auto fittingStart = getTime();
		PrecomputeFittingMatrices();
		auto fittingEnd = getTime();;
		printDuration("PrecomputeFittingMatrix", fittingStart, fittingEnd);

		std::printf("Done!\n");


		m_bSetupValid = true;

	}

	void PrecomputeOrientationMatrix() {
		auto putStart = getTime();
		// put constraints into vector (will be useful)
		std::vector<Constraint> vConstraintsVec;
		std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
		while (cur != end)
			vConstraintsVec.push_back(*cur++);
		auto putEnd = getTime();
		printDuration("-- Put constraints", putStart, putEnd);

		auto resizeStart = getTime();
		// resize matrix and clear to zero
		unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
		m_mFirstMatrix.SetSize(2 * nVerts, 2 * nVerts);
		for (unsigned int i = 0; i < 2 * nVerts; ++i)
			for (unsigned int j = 0; j < 2 * nVerts; ++j)
				m_mFirstMatrix[i][j] = 0.0;

		size_t nConstraints = vConstraintsVec.size();
		unsigned int nFreeVerts = nVerts - nConstraints;

		auto resizeEnd = getTime();
		printDuration("-- Resize & clear matrix", resizeStart, resizeEnd);

		auto orderingStart = getTime();
		// figure out vertex ordering. first do free vertices, then constraints
		unsigned int nRow = 0;
		m_vVertexMap.resize(nVerts);
		for (unsigned int i = 0; i < nVerts; ++i) {
			Constraint c(i, { 0, 0 });
			if (m_vConstraints.find(c) != m_vConstraints.end())
				continue;
			m_vVertexMap[i] = nRow++;
		}
		if (nRow != nFreeVerts)	DebugBreak();
		for (unsigned int i = 0; i < nConstraints; ++i)
			m_vVertexMap[vConstraintsVec[i].nVertex] = nRow++;
		if (nRow != nVerts)	DebugBreak();		// bad!
		auto orderingEnd = getTime();
		printDuration("-- Figure out vertex ordering", orderingStart, orderingEnd);

		auto testStart = getTime();
		// test vector...
		Vector gUTest(nVerts * 2);
		for (unsigned int i = 0; i < nVerts; ++i) {
			Constraint c(i, { 0, 0 });
			if (m_vConstraints.find(c) != m_vConstraints.end())
				continue;
			int nRow = m_vVertexMap[i];
			gUTest[2 * nRow] = m_vInitialVerts[i].vPosition.x;
			gUTest[2 * nRow + 1] = m_vInitialVerts[i].vPosition.y;
		}
		for (unsigned int i = 0; i < nConstraints; ++i) {
			int nRow = m_vVertexMap[vConstraintsVec[i].nVertex];
			gUTest[2 * nRow] = vConstraintsVec[i].vConstrainedPos.x;
			gUTest[2 * nRow + 1] = vConstraintsVec[i].vConstrainedPos.y;
		}
		auto testEnd = getTime();
		printDuration("-- Test vector", testStart, testEnd);

		auto fillStart = getTime();
		// ok, now fill matrix (?)
		size_t nTriangles = m_vTriangles.size();
		for (unsigned int i = 0; i < nTriangles; ++i) {
			Triangle& t = m_vTriangles[i];

			//		_RMSInfo("Triangle %d: \n", i);
			double fTriSumErr = 0;
			for (int j = 0; j < 3; ++j) {
				double fTriErr = 0;

				int n0x = 2 * m_vVertexMap[t.nVerts[j]];
				int n0y = n0x + 1;
				int n1x = 2 * m_vVertexMap[t.nVerts[(j + 1) % 3]];
				int n1y = n1x + 1;
				int n2x = 2 * m_vVertexMap[t.nVerts[(j + 2) % 3]];
				int n2y = n2x + 1;
				float x = t.vTriCoords[j].x;
				float y = t.vTriCoords[j].y;


				// DEBUG
				Vector2 v0 = { (float)gUTest[n0x], (float)gUTest[n0y] };
				Vector2 v1 = { (float)gUTest[n1x], (float)gUTest[n1y] };
				Vector2 v2 = { (float)gUTest[n2x], (float)gUTest[n2y] };
				Vector2 v01 = v1 - v0;
				Vector2 v01Perp = { v01.y, -v01.x };
				Vector2 vTest = v0 + v01 * x + v01Perp * y;
				float fDist = (vTest - v2).Dot(vTest - v2);
				//if ( fDist > 0.0001f )
				//	DebugBreak();
				//DEBUG

				//double dTest =
				//(1 - 2*x + (x*x) + (y*y))*pow(gUTest[n0x],2) + (1 - 2*x + (x*x) + (y*y))*pow(gUTest[n0y],2) +
				//	((x*x) + (y*y))*pow(gUTest[n1x],2) + ((x*x) + (y*y))*pow(gUTest[n1y],2) +
				//	pow(gUTest[n2x],2) + pow(gUTest[n2y],2) + gUTest[n1y]*(-2*y*gUTest[n2x] - 2*x*gUTest[n2y]) +
				//	gUTest[n0y]*(-2*y*gUTest[n1x] + (2*x - 2*(x*x) - 2*(y*y))*gUTest[n1y] + 2*y*gUTest[n2x] +
				//	(-2 + 2*x)*gUTest[n2y]) + gUTest[n0x]*
				//	((2*x - 2*(x*x) - 2*(y*y))*gUTest[n1x] + 2*y*gUTest[n1y] + (-2 + 2*x)*gUTest[n2x] -
				//	2*y*gUTest[n2y]) + gUTest[n1x]*(-2*x*gUTest[n2x] + 2*y*gUTest[n2y]);
				//_RMSInfo("TEST IS %f %f\n", dTest, fDist);

				// n0x,n?? elems
				m_mFirstMatrix[n0x][n0x] += 1 - 2 * x + x * x + y * y;
				m_mFirstMatrix[n0x][n1x] += 2 * x - 2 * x * x - 2 * y * y;		//m_mFirstMatrix[n1x][n0x] += 2*x - 2*x*x - 2*y*y;
				m_mFirstMatrix[n0x][n1y] += 2 * y;						//m_mFirstMatrix[n1y][n0x] += 2*y;
				m_mFirstMatrix[n0x][n2x] += -2 + 2 * x;					//m_mFirstMatrix[n2x][n0x] += -2 + 2*x;
				m_mFirstMatrix[n0x][n2y] += -2 * y;						//m_mFirstMatrix[n2y][n0x] += -2 * y;

				fTriErr += (1 - 2 * x + x * x + y * y) * gUTest[n0x] * gUTest[n0x];
				fTriErr += (2 * x - 2 * x * x - 2 * y * y) * gUTest[n0x] * gUTest[n1x];
				fTriErr += (2 * y) * gUTest[n0x] * gUTest[n1y];
				fTriErr += (-2 + 2 * x) * gUTest[n0x] * gUTest[n2x];
				fTriErr += (-2 * y) * gUTest[n0x] * gUTest[n2y];

				// n0y,n?? elems
				m_mFirstMatrix[n0y][n0y] += 1 - 2 * x + x * x + y * y;
				m_mFirstMatrix[n0y][n1x] += -2 * y;						//m_mFirstMatrix[n1x][n0y] += -2*y;
				m_mFirstMatrix[n0y][n1y] += 2 * x - 2 * x * x - 2 * y * y;		//m_mFirstMatrix[n1y][n0y] += 2*x - 2*x*x - 2*y*y;
				m_mFirstMatrix[n0y][n2x] += 2 * y;						//m_mFirstMatrix[n2x][n0y] += 2*y;
				m_mFirstMatrix[n0y][n2y] += -2 + 2 * x;					//m_mFirstMatrix[n2y][n0y] += -2 + 2*x;

				fTriErr += (1 - 2 * x + x * x + y * y) * gUTest[n0y] * gUTest[n0y];
				fTriErr += (-2 * y) * gUTest[n0y] * gUTest[n1x];
				fTriErr += (2 * x - 2 * x * x - 2 * y * y) * gUTest[n0y] * gUTest[n1y];
				fTriErr += (2 * y) * gUTest[n0y] * gUTest[n2x];
				fTriErr += (-2 + 2 * x) * gUTest[n0y] * gUTest[n2y];

				// n1x,n?? elems
				m_mFirstMatrix[n1x][n1x] += x * x + y * y;
				m_mFirstMatrix[n1x][n2x] += -2 * x;						//m_mFirstMatrix[n2x][n1x] += -2*x;
				m_mFirstMatrix[n1x][n2y] += 2 * y;						//m_mFirstMatrix[n2y][n1x] += 2*y;

				fTriErr += (x * x + y * y) * gUTest[n1x] * gUTest[n1x];
				fTriErr += (-2 * x) * gUTest[n1x] * gUTest[n2x];
				fTriErr += (2 * y) * gUTest[n1x] * gUTest[n2y];

				//n1y,n?? elems
				m_mFirstMatrix[n1y][n1y] += x * x + y * y;
				m_mFirstMatrix[n1y][n2x] += -2 * y;						//m_mFirstMatrix[n2x][n1y] += -2*y;
				m_mFirstMatrix[n1y][n2y] += -2 * x;						//m_mFirstMatrix[n2y][n1y] += -2*x;


				fTriErr += (x * x + y * y) * gUTest[n1y] * gUTest[n1y];
				fTriErr += (-2 * y) * gUTest[n1y] * gUTest[n2x];
				fTriErr += (-2 * x) * gUTest[n1y] * gUTest[n2y];

				// final 2 elems
				m_mFirstMatrix[n2x][n2x] += 1;
				m_mFirstMatrix[n2y][n2y] += 1;

				fTriErr += gUTest[n2x] * gUTest[n2x] + gUTest[n2y] * gUTest[n2y];

				//_RMSInfo("  Error for vert %d (%d) - %f\n", j, t.nVerts[j], fTriErr);
				fTriSumErr += fTriErr;
			}
			//_RMSInfo("  Total Error: %f\n", fTriSumErr);
		}

		auto fillEnd = getTime();
		printDuration("-- Fill matrix", fillStart, fillEnd);

		auto residualStart = getTime();
		// test...
		Vector gUTemp = m_mFirstMatrix * gUTest;
		double fSum = gUTemp.Dot(gUTest);
		std::printf("    (test) Residual is %f\n", fSum);
		auto residualEnd = getTime();
		printDuration("-- Residual", residualStart, residualEnd);

		/*
			// just try printing out matrix...
			for ( unsigned int i = 0; i < 2*nFreeVerts; ++i ) {
				for ( unsigned int j = 0 ; j < 2*nFreeVerts; ++j )
					_RMSInfo("%5.2f ", m_mFirstMatrix(i,j));
				_RMSInfo("| ");
				for ( unsigned int j = 0; j < 2*nConstraints; ++j )
					_RMSInfo("%5.2f ", m_mFirstMatrix(i, 2*nFreeVerts+j));
				_RMSInfo("\n");
			}
			_RMSInfo("-------\n");
			for ( unsigned int i = 0; i < 2*nConstraints; ++i ) {
				for ( unsigned int j = 0 ; j < 2*nFreeVerts; ++j )
					_RMSInfo("%5.2f ", m_mFirstMatrix(i+2*nFreeVerts,j));
				_RMSInfo("| ");
				for ( unsigned int j = 0; j < 2*nConstraints; ++j )
					_RMSInfo("%5.2f ", m_mFirstMatrix(i+2*nFreeVerts, 2*nFreeVerts+j));
				_RMSInfo("\n");
			}
			_RMSInfo("\n\n");
		*/

		auto extractStart = getTime();
		// extract G00 matrix
		Matrix mG00(2 * nFreeVerts, 2 * nFreeVerts);
		ExtractSubMatrix(m_mFirstMatrix, 0, 0, mG00);

		// extract G01 and G10 matrices
		Matrix mG01(2 * (int)nFreeVerts, 2 * (int)nConstraints);
		ExtractSubMatrix(m_mFirstMatrix, 0, 2 * nFreeVerts, mG01);
		Matrix mG10(2 * (int)nConstraints, 2 * (int)nFreeVerts);
		ExtractSubMatrix(m_mFirstMatrix, 2 * nFreeVerts, 0, mG10);
		auto extractEnd = getTime();
		printDuration("-- Extract G01 and G10", extractStart, extractEnd);

		auto gPrimeStart = getTime();
		// ok, now compute GPrime = G00 + Transpose(G00) and B = G01 + Transpose(G10)
		Matrix mGPrime = mG00 + mG00.Transpose();
		Matrix mB = mG01 + mG10.Transpose();
		auto gPrimeEnd = getTime();
		printDuration("-- Compute GPrime", gPrimeStart, gPrimeEnd);

		std::cout << "mGPrime: " << mGPrime.GetRows() << " rows, " << mGPrime.GetColumns() << " columns" << std::endl;
		auto invertGPrimeStart = getTime();
		m_mFirstMatrix = -(mGPrime.Inverse() * mB);
		auto invertGPrimeEnd = getTime();
		printDuration("-- Invert GPrime", invertGPrimeStart, invertGPrimeEnd);

		auto finalStart = getTime();
		// now compute -GPrimeInverse * B


		//Matrix mFinal = mGPrimeInverse * mB;
		//mFinal *= -1;

		//m_mFirstMatrix = mGPrimeNegatedInverse * mB;		// [RMS: not efficient!]
		auto finalEnd = getTime();
		printDuration("-- Compute mFinal", finalStart, finalEnd);

	}

	void PrecomputeScalingMatrices(unsigned int nTriangle) {
		// TODO.
	}

	void PrecomputeFittingMatrices() {
		auto putConstraintsStart = getTime();
		// put constraints into vector (will be useful)
		std::vector<Constraint> vConstraintsVec;
		std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
		while (cur != end)
			vConstraintsVec.push_back(*cur++);
		auto putConstraintsEnd = getTime();
		printDuration("-- Put constraints", putConstraintsStart, putConstraintsEnd);

		auto clearMatrixStart = getTime();
		// resize matrix and clear to zero
		unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
		size_t nConstraints = vConstraintsVec.size();
		unsigned int nFreeVerts = nVerts - nConstraints;
		auto clearMatrixEnd = getTime();
		printDuration("-- Resize & clear matrix", clearMatrixStart, clearMatrixEnd);

		auto figureOrderingStart = getTime();
		// figure out vertex ordering. first do free vertices, then constraints
		unsigned int nRow = 0;
		m_vVertexMap.resize(nVerts);
		for (unsigned int i = 0; i < nVerts; ++i) {
			Constraint c(i, { 0, 0 });
			if (m_vConstraints.find(c) != m_vConstraints.end())
				continue;
			m_vVertexMap[i] = nRow++;
		}
		if (nRow != nFreeVerts)	DebugBreak();
		for (unsigned int i = 0; i < nConstraints; ++i)
			m_vVertexMap[vConstraintsVec[i].nVertex] = nRow++;
		if (nRow != nVerts)	DebugBreak();		// bad!
		auto figureOrderingEnd = getTime();
		printDuration("-- Figure out vertex ordering", figureOrderingStart, figureOrderingEnd);

		auto testStart = getTime();
		// test vector...
		Vector gUTestX(nVerts), gUTestY(nVerts);
		for (unsigned int i = 0; i < nVerts; ++i) {
			Constraint c(i, { 0, 0 });
			if (m_vConstraints.find(c) != m_vConstraints.end())
				continue;
			int nRow = m_vVertexMap[i];
			gUTestX[nRow] = m_vInitialVerts[i].vPosition.x;
			gUTestY[nRow] = m_vInitialVerts[i].vPosition.y;
		}
		for (unsigned int i = 0; i < nConstraints; ++i) {
			int nRow = m_vVertexMap[vConstraintsVec[i].nVertex];
			gUTestX[nRow] = vConstraintsVec[i].vConstrainedPos.x;
			gUTestY[nRow] = vConstraintsVec[i].vConstrainedPos.y;
		}
		auto testEnd = getTime();
		printDuration("-- Test vector", testStart, testEnd);

		auto hyhxStart = getTime();
		// make Hy and Hx matrices
		Matrix mHX(nVerts, nVerts);
		Matrix mHY(nVerts, nVerts);
		for (unsigned int i = 0; i < nVerts; ++i)
			for (unsigned int j = 0; j < nVerts; ++j)
				mHX[i][j] = mHY[i][j] = 0.0;
		auto hyhxEnd = getTime();
		printDuration("-- Make Hy and Hx", hyhxStart, hyhxEnd);

		auto fillStart = getTime();
		// ok, now fill matrix
		size_t nTriangles = m_vTriangles.size();
		for (unsigned int i = 0; i < nTriangles; ++i) {
			Triangle& t = m_vTriangles[i];

			//		_RMSInfo("Triangle %d: \n", i);
			double fTriSumErr = 0;
			for (int j = 0; j < 3; ++j) {
				double fTriErr = 0;

				int nA = m_vVertexMap[t.nVerts[j]];
				int nB = m_vVertexMap[t.nVerts[(j + 1) % 3]];

				// X elems
				mHX[nA][nA] += 2;
				mHX[nA][nB] += -2;
				mHX[nB][nA] += -2;
				mHX[nB][nB] += 2;

				//  Y elems
				mHY[nA][nA] += 2;
				mHY[nA][nB] += -2;
				mHY[nB][nA] += -2;
				mHY[nB][nB] += 2;
			}
		}
		auto fillEnd = getTime();
		printDuration("-- Fill matrix", fillStart, fillEnd);

		auto extractStart = getTime();
		// extract HX00 and  HY00 matrices
		Matrix mHX00((int)nFreeVerts, (int)nFreeVerts);
		Matrix mHY00((int)nFreeVerts, (int)nFreeVerts);
		ExtractSubMatrix(mHX, 0, 0, mHX00);
		ExtractSubMatrix(mHY, 0, 0, mHY00);

		// Extract HX01 and HX10 matrices
		Matrix mHX01((int)nFreeVerts, (int)nConstraints);
		Matrix mHX10((int)nConstraints, (int)nFreeVerts);
		ExtractSubMatrix(mHX, 0, nFreeVerts, mHX01);
		ExtractSubMatrix(mHX, nFreeVerts, 0, mHX10);

		// Extract HY01 and HY10 matrices
		Matrix mHY01((int)nFreeVerts, (int)nConstraints);
		Matrix mHY10((int)nConstraints, (int)nFreeVerts);
		ExtractSubMatrix(mHY, 0, nFreeVerts, mHY01);
		ExtractSubMatrix(mHY, nFreeVerts, 0, mHY10);

		// now compute HXPrime = HX00 + Transpose(HX00) (and HYPrime)
		//Matrix mHXPrime( mHX00 + mHX00.Transpose() );
		//Matrix mHYPrime( mHY00 + mHY00.Transpose() );
		m_mHXPrime = mHX00;
		m_mHYPrime = mHY00;

		// and then D = HX01 + Transpose(HX10)
		//Matrix mDX = mHX01 + mHX10.Transpose();
		//Matrix mDY = mHY01 + mHY10.Transpose();
		m_mDX = mHX01;
		m_mDY = mHY01;
		auto extractEnd = getTime();
		printDuration("-- Extract matrices", extractStart, extractEnd);

		auto luStart = getTime();
		// pre-compute LU decompositions
		bool bResult = LUDecompose(m_mHXPrime, m_mLUDecompX);
		if (!bResult)
			DebugBreak();
		bResult = LUDecompose(m_mHYPrime, m_mLUDecompY);
		if (!bResult)
			DebugBreak();
		auto luEnd = getTime();
		printDuration("-- LU decompositions", luStart, luEnd);

	}

	void ValidateDeformedMesh(bool bRigid) {
		// TODO.
	}

	void UpdateScaledTriangle(unsigned int nTriangle) {
		// TODO.
	}

	void ApplyFittingStep() {
		// put constraints into vector (will be useful)
		std::vector<Constraint> vConstraintsVec;
		std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
		while (cur != end)
			vConstraintsVec.push_back(*cur++);

		unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
		size_t nConstraints = vConstraintsVec.size();
		unsigned int nFreeVerts = nVerts - nConstraints;

		// make vector of deformed vertex weights
		Vector vFX(nVerts);
		Vector vFY(nVerts);
		for (int i = 0; i < (int)nVerts; ++i)
			vFX[i] = vFY[i] = 0.0;

		size_t nTriangles = m_vTriangles.size();
		for (unsigned int i = 0; i < nTriangles; ++i) {
			Triangle& t = m_vTriangles[i];

			for (int j = 0; j < 3; ++j) {

				int nA = m_vVertexMap[t.nVerts[j]];
				int nB = m_vVertexMap[t.nVerts[(j + 1) % 3]];

				Vector2 vDeformedA(t.vScaled[j]);
				Vector2 vDeformedB(t.vScaled[(j + 1) % 3]);

				// X elems
				vFX[nA] += -2 * vDeformedA.x + 2 * vDeformedB.x;
				vFX[nB] += 2 * vDeformedA.x - 2 * vDeformedB.x;

				//  Y elems
				vFY[nA] += -2 * vDeformedA.y + 2 * vDeformedB.y;
				vFY[nB] += 2 * vDeformedA.y - 2 * vDeformedB.y;
			}
		}

		// make F0 vectors
		Vector vF0X(nFreeVerts), vF0Y(nFreeVerts);
		for (int i = 0; i < (int)nFreeVerts; ++i) {
			vF0X[i] = vFX[i];
			vF0Y[i] = vFY[i];
		}

		// make Q vectors (vectors of constraints)
		Vector vQX((int)nConstraints), vQY((int)nConstraints);
		for (int i = 0; i < (int)nConstraints; ++i) {
			vQX[i] = vConstraintsVec[i].vConstrainedPos.x;
			vQY[i] = vConstraintsVec[i].vConstrainedPos.y;
		}

		// ok, compute RHS for X and solve
		Vector vRHSX(m_mDX * vQX);
		vRHSX = -(vRHSX + vF0X);
		Vector vSolutionX((int)nFreeVerts);

		//Wml::LinearSystemd::Solve( m_mHXPrime, vRHSX, vSolutionX );
		/*
		bool bResult = Wml::LinearSystemExtd::LUBackSub(m_mLUDecompX, vRHSX, vSolutionX);
		if (!bResult)
			DebugBreak();
		*/

		// now for Y
		Vector vRHSY(m_mDY * vQY);
		vRHSY = -(vRHSY + vF0Y);
		Vector vSolutionY((int)nFreeVerts);
		//	Wml::LinearSystemd::Solve( m_mHYPrime, vRHSY, vSolutionY );
		/*
		bResult = Wml::LinearSystemExtd::LUBackSub(m_mLUDecompY, vRHSY, vSolutionY);
		if (!bResult)
			DebugBreak();
		*/

		// done!
		for (unsigned int i = 0; i < nVerts; ++i) {
			Constraint c(i, { 0, 0 });
			if (m_vConstraints.find(c) != m_vConstraints.end())
				continue;
			int nRow = m_vVertexMap[i];
			m_vDeformedVerts[i].vPosition.x = (float)vSolutionX[nRow];
			m_vDeformedVerts[i].vPosition.y = (float)vSolutionY[nRow];
		}

	}

	Vector2 GetInitialVert(unsigned int nVert) {
		return { m_vInitialVerts[nVert].vPosition.x, m_vInitialVerts[nVert].vPosition.y };
	}

private:
	std::vector<Vertex> m_vInitialVerts;
	std::vector<Vertex> m_vDeformedVerts;

	std::vector<Triangle> m_vTriangles;

	// ===============================

	bool m_bSetupValid;

	// ===============================

	std::set<Constraint> m_vConstraints;
	Matrix m_mFirstMatrix;
	std::vector<unsigned int> m_vVertexMap;
	Matrix m_mHXPrime, m_mHYPrime;
	Matrix m_mDX, m_mDY;

	LUData m_mLUDecompX, m_mLUDecompY;
};
