#include "RigidMeshDeformer2D.h"

#include <chrono>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

static constexpr bool measureDuration = true;

using namespace rmsmesh;

using Time = std::chrono::steady_clock::time_point;

static Time getTime() {
    if (measureDuration) {
        return std::chrono::steady_clock::now();
    }
    else {
        // Do nothing.
    }
}

static int getDuration(const Time& time1, const Time& time2) {
    if (measureDuration) {
        return std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count();
    }
    else {
        // Do nothing.
    }
}

static void printDuration(const std::string& name, const Time& time1, const Time& time2) {
    if (measureDuration) {
        std::stringstream ss;
        ss << std::fixed << name << ": " << getDuration(time1, time2) / 1000000.0f << "s" << std::endl;
        std::cout << ss.str();
    }
    else {
        // Do nothing.
    }
}

// Instead of WildMagic's barycentric coordinate function...
// https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates/23745#23745.
static void Barycentric(
    const Eigen::Vector2f& p,
    const Eigen::Vector2f& a,
    const Eigen::Vector2f& b,
    const Eigen::Vector2f& c,
    float& u,
    float& v,
    float& w
) {
    auto v0 = b - a;
    auto v1 = c - a;
    auto v2 = p - a;
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

// Instead of WildMagic's Wml::Scale()...
// (We rewrote that function for Eigen.)
static void Scale(
    Eigen::Vector2f& vTriV0,
    Eigen::Vector2f& vTriV1,
    Eigen::Vector2f& vTriV2,
    float fScale
) {
    // find center of mass
    Eigen::Vector2f vCentroid(vTriV0 + vTriV1 + vTriV2);
    vCentroid *= 1.0f / 3.0f;

    // convert to vectors, scale and restore
    vTriV0 -= vCentroid;	vTriV0 *= fScale;	vTriV0 += vCentroid;
    vTriV1 -= vCentroid;	vTriV1 *= fScale;	vTriV1 += vCentroid;
    vTriV2 -= vCentroid;	vTriV2 *= fScale;	vTriV2 += vCentroid;
}

static void DebugBreak() {
#ifdef NDEBUG
    throw std::runtime_error("Error!");
#endif
}

RigidMeshDeformer2D::RigidMeshDeformer2D()
{
    InvalidateSetup();
}

void RigidMeshDeformer2D::SetDeformedHandle(unsigned int nHandle, const Deform2D_Vector3* vHandle)
{
    Constraint c(nHandle, { vHandle->x, vHandle->y });
    UpdateConstraint(c);
}

void RigidMeshDeformer2D::RemoveHandle(unsigned int nHandle)
{
    Constraint c(nHandle, { 0, 0 });
    m_vConstraints.erase(c);
    m_vDeformedVerts[nHandle].vPosition = m_vInitialVerts[nHandle].vPosition;
    InvalidateSetup();
}


void RigidMeshDeformer2D::UnTransformPoint(Deform2D_Vector3& vTransform)
{
    // find triangle
    size_t nTris = m_vTriangles.size();
    for (unsigned int i = 0; i < nTris; ++i) {
        Eigen::Vector2f v1(m_vDeformedVerts[m_vTriangles[i].nVerts[0]].vPosition);
        Eigen::Vector2f v2(m_vDeformedVerts[m_vTriangles[i].nVerts[1]].vPosition);
        Eigen::Vector2f v3(m_vDeformedVerts[m_vTriangles[i].nVerts[2]].vPosition);

        float fBary1, fBary2, fBary3;
        Barycentric({ vTransform.x, vTransform.y }, v1, v2, v3, fBary1, fBary2, fBary3);

        Eigen::Vector2f v1Init(m_vInitialVerts[m_vTriangles[i].nVerts[0]].vPosition);
        Eigen::Vector2f v2Init(m_vInitialVerts[m_vTriangles[i].nVerts[1]].vPosition);
        Eigen::Vector2f v3Init(m_vInitialVerts[m_vTriangles[i].nVerts[2]].vPosition);
        auto transform = fBary1 * v1Init + fBary2 * v2Init + fBary3 * v3Init;
        vTransform.x = transform[0];
        vTransform.y = transform[1];
        return;
    }

}

void rmsmesh::RigidMeshDeformer2D::SetMesh(
    Deform2D_Vector3* vertices,
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
        v.vPosition = Eigen::Vector2f(vertices[i].x, vertices[i].y);
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

            Eigen::Vector2f v0 = GetInitialVert(t.nVerts[n0]);
            Eigen::Vector2f v1 = GetInitialVert(t.nVerts[n1]);
            Eigen::Vector2f v2 = GetInitialVert(t.nVerts[n2]);

            // find coordinate system
            Eigen::Vector2f v01(v1 - v0);
            Eigen::Vector2f v01N(v01);  v01N.normalize();
            Eigen::Vector2f v01Rot90(v01.y(), -v01.x());
            Eigen::Vector2f v01Rot90N(v01Rot90);  v01Rot90N.normalize();

            // express v2 in coordinate system
            Eigen::Vector2f vLocal(v2 - v0);
            float fX = vLocal.dot(v01) / v01.squaredNorm();
            float fY = vLocal.dot(v01Rot90) / v01Rot90.squaredNorm();

            // sanity check
            Eigen::Vector2f v2test(v0 + fX * v01 + fY * v01Rot90);
            float fLength = (v2test - v2).norm();
            if (fLength > 0.001f)
                DebugBreak();

            t.vTriCoords[j] = Eigen::Vector2f(fX, fY);
        }
    }

    auto setMeshEnd = getTime();
    printDuration("SetMesh", setMeshStart, setMeshEnd);
}

void rmsmesh::RigidMeshDeformer2D::GetDeformedMesh(
    Deform2D_Vector3* vertices,
    unsigned int vertexCount,
    bool isRigid
) {
    auto validateStart = getTime();
    ValidateDeformedMesh(isRigid);
    auto validateEnd = getTime();
    printDuration("ValidateDeformedMesh", validateStart, validateEnd);

    std::vector<Vertex>& vVerts = (m_vConstraints.size() > 1) ? m_vDeformedVerts : m_vInitialVerts;

    for (unsigned int i = 0; i < vertexCount; ++i) {
        Eigen::Vector2f vNewPos(vVerts[i].vPosition);
        vertices[i].x = vNewPos.x();
        vertices[i].y = vNewPos.y();
    }
}

void RigidMeshDeformer2D::UpdateConstraint(Constraint& cons)
{
    std::set<Constraint>::iterator found(m_vConstraints.find(cons));
    if (found != m_vConstraints.end()) {

        const_cast<Eigen::Vector2f&>((*found).vConstrainedPos) = cons.vConstrainedPos;
        m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;

    }
    else {
        m_vConstraints.insert(cons);
        m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;
        InvalidateSetup();
    }

}




void ExtractSubMatrix(Eigen::MatrixXd& mFrom, int nRowOffset, int nColOffset, Eigen::MatrixXd& mTo)
{
    int nRows = mTo.rows();
    int nCols = mTo.cols();

    for (int i = 0; i < nRows; ++i) {
        for (int j = 0; j < nCols; ++j) {
            mTo(i, j) = mFrom(i + nRowOffset, j + nColOffset);
        }
    }
}


void RigidMeshDeformer2D::ValidateSetup()
{
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




void RigidMeshDeformer2D::PrecomputeFittingMatrices()
{
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
    Eigen::VectorXd gUTestX(nVerts), gUTestY(nVerts);
    for (unsigned int i = 0; i < nVerts; ++i) {
        Constraint c(i, { 0, 0 });
        if (m_vConstraints.find(c) != m_vConstraints.end())
            continue;
        int nRow = m_vVertexMap[i];
        gUTestX[nRow] = m_vInitialVerts[i].vPosition.x();
        gUTestY[nRow] = m_vInitialVerts[i].vPosition.y();
    }
    for (unsigned int i = 0; i < nConstraints; ++i) {
        int nRow = m_vVertexMap[vConstraintsVec[i].nVertex];
        gUTestX[nRow] = vConstraintsVec[i].vConstrainedPos.x();
        gUTestY[nRow] = vConstraintsVec[i].vConstrainedPos.y();
    }
    auto testEnd = getTime();
    printDuration("-- Test vector", testStart, testEnd);

    auto hyhxStart = getTime();
    // make Hy and Hx matrices
    Eigen::MatrixXd mHX(nVerts, nVerts);
    Eigen::MatrixXd mHY(nVerts, nVerts);
    for (unsigned int i = 0; i < nVerts; ++i)
        for (unsigned int j = 0; j < nVerts; ++j)
            mHX(i, j) = mHY(i, j) = 0.0;
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
            mHX(nA, nA) += 2;
            mHX(nA, nB) += -2;
            mHX(nB, nA) += -2;
            mHX(nB, nB) += 2;

            //  Y elems
            mHY(nA, nA) += 2;
            mHY(nA, nB) += -2;
            mHY(nB, nA) += -2;
            mHY(nB, nB) += 2;
        }
    }
    auto fillEnd = getTime();
    printDuration("-- Fill matrix", fillStart, fillEnd);

    auto extractStart = getTime();
    // extract HX00 and  HY00 matrices
    Eigen::MatrixXd mHX00((int)nFreeVerts, (int)nFreeVerts);
    Eigen::MatrixXd mHY00((int)nFreeVerts, (int)nFreeVerts);
    ExtractSubMatrix(mHX, 0, 0, mHX00);
    ExtractSubMatrix(mHY, 0, 0, mHY00);

    // Extract HX01 and HX10 matrices
    Eigen::MatrixXd mHX01((int)nFreeVerts, (int)nConstraints);
    Eigen::MatrixXd mHX10((int)nConstraints, (int)nFreeVerts);
    ExtractSubMatrix(mHX, 0, nFreeVerts, mHX01);
    ExtractSubMatrix(mHX, nFreeVerts, 0, mHX10);

    // Extract HY01 and HY10 matrices
    Eigen::MatrixXd mHY01((int)nFreeVerts, (int)nConstraints);
    Eigen::MatrixXd mHY10((int)nConstraints, (int)nFreeVerts);
    ExtractSubMatrix(mHY, 0, nFreeVerts, mHY01);
    ExtractSubMatrix(mHY, nFreeVerts, 0, mHY10);

    // now compute HXPrime = HX00 + Transpose(HX00) (and HYPrime)
    //Eigen::MatrixXd mHXPrime( mHX00 + mHX00.Transpose() );
    //Eigen::MatrixXd mHYPrime( mHY00 + mHY00.Transpose() );
    m_mHXPrime = mHX00;
    m_mHYPrime = mHY00;

    // and then D = HX01 + Transpose(HX10)
    //Eigen::MatrixXd mDX = mHX01 + mHX10.Transpose();
    //Eigen::MatrixXd mDY = mHY01 + mHY10.Transpose();
    m_mDX = mHX01;
    m_mDY = mHY01;
    auto extractEnd = getTime();
    printDuration("-- Extract matrices", extractStart, extractEnd);

    auto luStart = getTime();
    // pre-compute LU decompositions
    m_mHXPrimeSolver.compute(m_mHXPrime);
    m_mHYPrimeSolver.compute(m_mHYPrime);
    auto luEnd = getTime();
    printDuration("-- LU decompositions", luStart, luEnd);
}


void RigidMeshDeformer2D::ApplyFittingStep()
{
    // put constraints into vector (will be useful)
    std::vector<Constraint> vConstraintsVec;
    std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
    while (cur != end)
        vConstraintsVec.push_back(*cur++);

    unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
    size_t nConstraints = vConstraintsVec.size();
    unsigned int nFreeVerts = nVerts - nConstraints;

    // make vector of deformed vertex weights
    Eigen::VectorXd vFX(nVerts);
    Eigen::VectorXd vFY(nVerts);
    for (int i = 0; i < (int)nVerts; ++i)
        vFX[i] = vFY[i] = 0.0;

    size_t nTriangles = m_vTriangles.size();
    for (unsigned int i = 0; i < nTriangles; ++i) {
        Triangle& t = m_vTriangles[i];

        for (int j = 0; j < 3; ++j) {

            int nA = m_vVertexMap[t.nVerts[j]];
            int nB = m_vVertexMap[t.nVerts[(j + 1) % 3]];

            Eigen::Vector2f vDeformedA(t.vScaled[j]);
            Eigen::Vector2f vDeformedB(t.vScaled[(j + 1) % 3]);

            // X elems
            vFX[nA] += -2 * vDeformedA.x() + 2 * vDeformedB.x();
            vFX[nB] += 2 * vDeformedA.x() - 2 * vDeformedB.x();

            //  Y elems
            vFY[nA] += -2 * vDeformedA.y() + 2 * vDeformedB.y();
            vFY[nB] += 2 * vDeformedA.y() - 2 * vDeformedB.y();
        }
    }

    // make F0 vectors
    Eigen::VectorXd vF0X(nFreeVerts), vF0Y(nFreeVerts);
    for (int i = 0; i < (int)nFreeVerts; ++i) {
        vF0X[i] = vFX[i];
        vF0Y[i] = vFY[i];
    }

    // make Q vectors (vectors of constraints)
    Eigen::VectorXd vQX((int)nConstraints), vQY((int)nConstraints);
    for (int i = 0; i < (int)nConstraints; ++i) {
        vQX[i] = vConstraintsVec[i].vConstrainedPos.x();
        vQY[i] = vConstraintsVec[i].vConstrainedPos.y();
    }

    // ok, compute RHS for X and solve
    Eigen::VectorXd vRHSX(m_mDX * vQX);
    vRHSX += vF0X;
    vRHSX *= -1;
    //Eigen::VectorXd vSolutionX((int)nFreeVerts);

    //Wml::LinearSystemd::Solve( m_mHXPrime, vRHSX, vSolutionX );
    Eigen::VectorXd vSolutionX = m_mHXPrimeSolver.solve(vRHSX);

    // now for Y
    Eigen::VectorXd vRHSY(m_mDY * vQY);
    vRHSY += vF0Y;
    vRHSY *= -1;
    //Eigen::VectorXd vSolutionY((int)nFreeVerts);

    //	Wml::LinearSystemd::Solve( m_mHYPrime, vRHSY, vSolutionY );
    Eigen::VectorXd vSolutionY = m_mHYPrimeSolver.solve(vRHSY);

    // done!
    for (unsigned int i = 0; i < nVerts; ++i) {
        Constraint c(i, { 0, 0 });
        if (m_vConstraints.find(c) != m_vConstraints.end())
            continue;
        int nRow = m_vVertexMap[i];
        m_vDeformedVerts[i].vPosition.x() = (float)vSolutionX[nRow];
        m_vDeformedVerts[i].vPosition.y() = (float)vSolutionY[nRow];
    }

}












void RigidMeshDeformer2D::PrecomputeOrientationMatrix()
{
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

    m_mFirstMatrix.resize(2 * nVerts, 2 * nVerts);
    m_mFirstMatrix.setZero();

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
    Eigen::VectorXd gUTest(nVerts * 2);
    for (unsigned int i = 0; i < nVerts; ++i) {
        Constraint c(i, { 0, 0 });
        if (m_vConstraints.find(c) != m_vConstraints.end())
            continue;
        int nRow = m_vVertexMap[i];
        gUTest[2 * nRow] = m_vInitialVerts[i].vPosition.x();
        gUTest[2 * nRow + 1] = m_vInitialVerts[i].vPosition.y();
    }
    for (unsigned int i = 0; i < nConstraints; ++i) {
        int nRow = m_vVertexMap[vConstraintsVec[i].nVertex];
        gUTest[2 * nRow] = vConstraintsVec[i].vConstrainedPos.x();
        gUTest[2 * nRow + 1] = vConstraintsVec[i].vConstrainedPos.y();
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
            float x = t.vTriCoords[j].x();
            float y = t.vTriCoords[j].y();


            // DEBUG
            Eigen::Vector2f v0((float)gUTest[n0x], (float)gUTest[n0y]);
            Eigen::Vector2f v1((float)gUTest[n1x], (float)gUTest[n1y]);
            Eigen::Vector2f v2((float)gUTest[n2x], (float)gUTest[n2y]);
            Eigen::Vector2f v01(v1 - v0);
            Eigen::Vector2f v01Perp(v01.y(), -v01.x());
            Eigen::Vector2f vTest(v0 + x * v01 + y * v01Perp);
            float fDist = (vTest - v2).dot(vTest - v2);
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
            m_mFirstMatrix(n0x, n0x) += 1 - 2 * x + x * x + y * y;
            m_mFirstMatrix(n0x, n1x) += 2 * x - 2 * x * x - 2 * y * y;		//m_mFirstMatrix(n1x, n0x) += 2*x - 2*x*x - 2*y*y;
            m_mFirstMatrix(n0x, n1y) += 2 * y;						//m_mFirstMatrix(n1y, n0x) += 2*y;
            m_mFirstMatrix(n0x, n2x) += -2 + 2 * x;					//m_mFirstMatrix(n2x, n0x) += -2 + 2*x;
            m_mFirstMatrix(n0x, n2y) += -2 * y;						//m_mFirstMatrix(n2y, n0x) += -2 * y;

            fTriErr += (1 - 2 * x + x * x + y * y) * gUTest(n0x) * gUTest(n0x);
            fTriErr += (2 * x - 2 * x * x - 2 * y * y) * gUTest[n0x] * gUTest[n1x];
            fTriErr += (2 * y) * gUTest[n0x] * gUTest[n1y];
            fTriErr += (-2 + 2 * x) * gUTest[n0x] * gUTest[n2x];
            fTriErr += (-2 * y) * gUTest[n0x] * gUTest[n2y];

            // n0y,n?? elems
            m_mFirstMatrix(n0y, n0y) += 1 - 2 * x + x * x + y * y;
            m_mFirstMatrix(n0y, n1x) += -2 * y;						//m_mFirstMatrix(n1x, n0y) += -2*y;
            m_mFirstMatrix(n0y, n1y) += 2 * x - 2 * x * x - 2 * y * y;		//m_mFirstMatrix(n1y, n0y) += 2*x - 2*x*x - 2*y*y;
            m_mFirstMatrix(n0y, n2x) += 2 * y;						//m_mFirstMatrix(n2x, n0y) += 2*y;
            m_mFirstMatrix(n0y, n2y) += -2 + 2 * x;					//m_mFirstMatrix(n2y, n0y) += -2 + 2*x;

            fTriErr += (1 - 2 * x + x * x + y * y) * gUTest[n0y] * gUTest[n0y];
            fTriErr += (-2 * y) * gUTest[n0y] * gUTest[n1x];
            fTriErr += (2 * x - 2 * x * x - 2 * y * y) * gUTest[n0y] * gUTest[n1y];
            fTriErr += (2 * y) * gUTest[n0y] * gUTest[n2x];
            fTriErr += (-2 + 2 * x) * gUTest[n0y] * gUTest[n2y];

            // n1x,n?? elems
            m_mFirstMatrix(n1x, n1x) += x * x + y * y;
            m_mFirstMatrix(n1x, n2x) += -2 * x;						//m_mFirstMatrix(n2x, n1x) += -2*x;
            m_mFirstMatrix(n1x, n2y) += 2 * y;						//m_mFirstMatrix(n2y, n1x) += 2*y;

            fTriErr += (x * x + y * y) * gUTest[n1x] * gUTest[n1x];
            fTriErr += (-2 * x) * gUTest[n1x] * gUTest[n2x];
            fTriErr += (2 * y) * gUTest[n1x] * gUTest[n2y];

            //n1y,n?? elems
            m_mFirstMatrix(n1y, n1y) += x * x + y * y;
            m_mFirstMatrix(n1y, n2x) += -2 * y;						//m_mFirstMatrix(n2x, n1y) += -2*y;
            m_mFirstMatrix(n1y, n2y) += -2 * x;						//m_mFirstMatrix(n2y, n1y) += -2*x;


            fTriErr += (x * x + y * y) * gUTest[n1y] * gUTest[n1y];
            fTriErr += (-2 * y) * gUTest[n1y] * gUTest[n2x];
            fTriErr += (-2 * x) * gUTest[n1y] * gUTest[n2y];

            // final 2 elems
            m_mFirstMatrix(n2x, n2x) += 1;
            m_mFirstMatrix(n2y, n2y) += 1;

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
    Eigen::VectorXd gUTemp = m_mFirstMatrix * gUTest;
    double fSum = gUTemp.dot(gUTest);
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
    Eigen::MatrixXd mG00(2 * nFreeVerts, 2 * nFreeVerts);
    ExtractSubMatrix(m_mFirstMatrix, 0, 0, mG00);

    // extract G01 and G10 matrices
    Eigen::MatrixXd mG01(2 * (int)nFreeVerts, 2 * (int)nConstraints);
    ExtractSubMatrix(m_mFirstMatrix, 0, 2 * nFreeVerts, mG01);
    Eigen::MatrixXd mG10(2 * (int)nConstraints, 2 * (int)nFreeVerts);
    ExtractSubMatrix(m_mFirstMatrix, 2 * nFreeVerts, 0, mG10);
    auto extractEnd = getTime();
    printDuration("-- Extract G01 and G10", extractStart, extractEnd);

    auto gPrimeStart = getTime();
    // ok, now compute GPrime = G00 + Transpose(G00) and B = G01 + Transpose(G10)
    Eigen::MatrixXd mGPrime = mG00 + mG00.transpose();
    Eigen::MatrixXd mB = mG01 + mG10.transpose();
    auto gPrimeEnd = getTime();
    printDuration("-- Compute GPrime", gPrimeStart, gPrimeEnd);

    std::cout << "mGPrime: " << mGPrime.rows() << " rows, " << mGPrime.cols() << " columns" << std::endl;

    auto finalStart = getTime();
    m_mFirstMatrix = -mGPrime.inverse() * mB;
    //m_mFirstMatrix = mGPrimeNegatedInverse * mB;		// [RMS: not efficient!]
    auto finalEnd = getTime();
    printDuration("-- Compute mFinal", finalStart, finalEnd);
}








static RigidMeshDeformer2D::Triangle* g_pCurTriangle = NULL;
float g_fErrSum = 0;
void AccumErrorSum(int nRow, int nCol, float fAccum)
{
    Eigen::Vector2f& vRowVtx = g_pCurTriangle->vScaled[nRow / 2];
    Eigen::Vector2f& vColVtx = g_pCurTriangle->vScaled[nCol / 2];
    g_fErrSum += fAccum * vRowVtx[nRow % 2] * vColVtx[nCol % 2];
}

void AccumScaleEntry(Eigen::MatrixXd& mF, int nRow, int nCol, double fAccum)
{
    if (nRow < 4 && nCol < 4) {
        mF(nRow, nCol) += fAccum;
    }
    else {
        DebugBreak();
    }
}





void RigidMeshDeformer2D::PrecomputeScalingMatrices(unsigned int nTriangle)
{
    // ok now fill matrix
    Triangle& t = m_vTriangles[nTriangle];

    // create matrices and clear to zero
    t.mF = Eigen::MatrixXd(4, 4);
    t.mC = Eigen::MatrixXd(4, 6);

    // precompute coeffs
    double x01 = t.vTriCoords[0].x();
    double y01 = t.vTriCoords[0].y();
    double x12 = t.vTriCoords[1].x();
    double y12 = t.vTriCoords[1].y();
    double x20 = t.vTriCoords[2].x();
    double y20 = t.vTriCoords[2].y();

    double k1 = x12 * y01 + (-1 + x01) * y12;
    double k2 = -x12 + x01 * x12 - y01 * y12;
    double k3 = -y01 + x20 * y01 + x01 * y20;
    double k4 = -y01 + x01 * y01 + x01 * y20;
    double k5 = -x01 + x01 * x20 - y01 * y20;

    double a = -1 + x01;
    double a1 = pow(-1 + x01, 2) + pow(y01, 2);
    double a2 = pow(x01, 2) + pow(y01, 2);
    double b = -1 + x20;
    double b1 = pow(-1 + x20, 2) + pow(y20, 2);
    double c2 = pow(x12, 2) + pow(y12, 2);

    double r1 = 1 + 2 * a * x12 + a1 * pow(x12, 2) - 2 * y01 * y12 + a1 * pow(y12, 2);
    double r2 = -(b * x01) - b1 * pow(x01, 2) + y01 * (-(b1 * y01) + y20);
    double r3 = -(a * x12) - a1 * pow(x12, 2) + y12 * (y01 - a1 * y12);
    double r5 = a * x01 + pow(y01, 2);
    double r6 = -(b * y01) - x01 * y20;
    double r7 = 1 + 2 * b * x01 + b1 * pow(x01, 2) + b1 * pow(y01, 2) - 2 * y01 * y20;

    //  set up F matrix

    // row 0 mF
    t.mF(0, 0) = 2 * a1 + 2 * a1 * c2 + 2 * r7;
    t.mF(0, 1) = 0;
    t.mF(0, 2) = 2 * r2 + 2 * r3 - 2 * r5;
    t.mF(0, 3) = 2 * k1 + 2 * r6 + 2 * y01;

    // row 1
    t.mF(1, 0) = 0;
    t.mF(1, 1) = 2 * a1 + 2 * a1 * c2 + 2 * r7;
    t.mF(1, 2) = -2 * k1 + 2 * k3 - 2 * y01;
    t.mF(1, 3) = 2 * r2 + 2 * r3 - 2 * r5;

    // row 2
    t.mF(2, 0) = 2 * r2 + 2 * r3 - 2 * r5;
    t.mF(2, 1) = -2 * k1 + 2 * k3 - 2 * y01;
    t.mF(2, 2) = 2 * a2 + 2 * a2 * b1 + 2 * r1;
    t.mF(2, 3) = 0;

    //row 3
    t.mF(3, 0) = 2 * k1 - 2 * k3 + 2 * y01;
    t.mF(3, 1) = 2 * r2 + 2 * r3 - 2 * r5;
    t.mF(3, 2) = 0;
    t.mF(3, 3) = 2 * a2 + 2 * a2 * b1 + 2 * r1;

    // ok, now invert F
    t.mF = -t.mF.inverse();

    // set up C matrix

    // row 0 mC
    t.mC(0, 0) = 2 * k2;
    t.mC(0, 1) = -2 * k1;
    t.mC(0, 2) = 2 * (-1 - k5);
    t.mC(0, 3) = 2 * k3;
    t.mC(0, 4) = 2 * a;
    t.mC(0, 5) = -2 * y01;

    // row 1 mC
    t.mC(1, 0) = 2 * k1;
    t.mC(1, 1) = 2 * k2;
    t.mC(1, 2) = -2 * k3;
    t.mC(1, 3) = 2 * (-1 - k5);
    t.mC(1, 4) = 2 * y01;
    t.mC(1, 5) = 2 * a;

    // row 2 mC
    t.mC(2, 0) = 2 * (-1 - k2);
    t.mC(2, 1) = 2 * k1;
    t.mC(2, 2) = 2 * k5;
    t.mC(2, 3) = 2 * r6;
    t.mC(2, 4) = -2 * x01;
    t.mC(2, 5) = 2 * y01;

    // row 3 mC
    t.mC(3, 0) = 2 * k1;
    t.mC(3, 1) = 2 * (-1 - k2);
    t.mC(3, 2) = -2 * k3;
    t.mC(3, 3) = 2 * k5;
    t.mC(3, 4) = -2 * y01;
    t.mC(3, 5) = -2 * x01;


}


void RigidMeshDeformer2D::UpdateScaledTriangle(unsigned int nTriangle)
{
    // ok now fill matrix
    Triangle& t = m_vTriangles[nTriangle];

    // multiply mC by deformed vertex position
    const Eigen::Vector2f& vDeformedV0 = m_vDeformedVerts[t.nVerts[0]].vPosition;
    const Eigen::Vector2f& vDeformedV1 = m_vDeformedVerts[t.nVerts[1]].vPosition;
    const Eigen::Vector2f& vDeformedV2 = m_vDeformedVerts[t.nVerts[2]].vPosition;
    Eigen::VectorXd vDeformed{ { vDeformedV0.x(), vDeformedV0.y(),
                      vDeformedV1.x(), vDeformedV1.y(),
                      vDeformedV2.x(), vDeformedV2.y() } };
    Eigen::VectorXd mCVec = t.mC * vDeformed;

    // compute -MFInv * mC
    Eigen::VectorXd vSolution = t.mF * mCVec;

    // ok, grab deformed v0 and v1 from solution vector
    Eigen::Vector2f vFitted0((float)vSolution[0], (float)vSolution[1]);
    Eigen::Vector2f vFitted1((float)vSolution[2], (float)vSolution[3]);

    // figure out fitted2
    float x01 = t.vTriCoords[0].x();
    float y01 = t.vTriCoords[0].y();
    Eigen::Vector2f vFitted01(vFitted1 - vFitted0);
    Eigen::Vector2f vFitted01Perp(vFitted01.y(), -vFitted01.x());
    Eigen::Vector2f vFitted2(vFitted0 + (float)x01 * vFitted01 + (float)y01 * vFitted01Perp);

    // ok now determine scale
    Eigen::Vector2f& vOrigV0 = m_vInitialVerts[t.nVerts[0]].vPosition;
    Eigen::Vector2f& vOrigV1 = m_vInitialVerts[t.nVerts[1]].vPosition;
    float fScale = (vOrigV1 - vOrigV0).norm() / vFitted01.norm();

    // now scale triangle
    Scale(vFitted0, vFitted1, vFitted2, fScale);

    t.vScaled[0] = vFitted0;
    t.vScaled[1] = vFitted1;
    t.vScaled[2] = vFitted2;
}


void RigidMeshDeformer2D::ValidateDeformedMesh(bool bRigid)
{
    size_t nConstraints = m_vConstraints.size();
    if (nConstraints < 2)
        return;

    ValidateSetup();

    // make q vector of constraints
    Eigen::VectorXd vQ(2 * (int)nConstraints);
    int k = 0;
    std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
    while (cur != end) {
        Constraint& c = const_cast<Constraint&>(*cur++);
        vQ[2 * k] = c.vConstrainedPos.x();
        vQ[2 * k + 1] = c.vConstrainedPos.y();
        ++k;
    }

    Eigen::VectorXd vU = m_mFirstMatrix * vQ;
    size_t nVerts = m_vDeformedVerts.size();
    for (unsigned int i = 0; i < nVerts; ++i) {
        Constraint c(i, { 0, 0 });
        if (m_vConstraints.find(c) != m_vConstraints.end())
            continue;
        int nRow = m_vVertexMap[i];

        double fX = vU[2 * nRow];
        double fY = vU[2 * nRow + 1];
        m_vDeformedVerts[i].vPosition = Eigen::Vector2f((float)fX, (float)fY);
    }

    if (bRigid) {
        // ok, now scale triangles
        size_t nTris = m_vTriangles.size();
        for (unsigned int i = 0; i < nTris; ++i)
            UpdateScaledTriangle(i);


        ApplyFittingStep();
    }
}


